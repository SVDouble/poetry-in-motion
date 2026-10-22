//! Typed, ordered planar tasks. Posture and manipulability remain preferences below all task levels.
use super::{Plan, point_kinematics, wrap};
use crate::{MAXN, model::Model};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TaskFrame {
    Ground,
    Body(usize),
    Tool,
}
impl TaskFrame {
    fn index(self, model: &Model) -> Option<usize> {
        match self {
            Self::Ground => Some(0),
            Self::Body(body) if body < model.desc.nb() => Some(body + 1),
            Self::Tool if model.has_tool() => Some(model.desc.nb() + 1),
            _ => None,
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub enum TaskGoal {
    Orientation { frame: TaskFrame, target: f64, wn: f64 },
    Point { frame: TaskFrame, target: [f64; 2], wn: f64 },
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TaskRole {
    Orientation,
    Elbow,
    Additional,
}

/// Per-step task: row-major J, J̇q̇, desired acceleration ω²e − 2ωẋ and error.
#[derive(Clone, Copy, Debug)]
pub struct SubTask {
    pub role: TaskRole,
    pub rows: usize,
    pub j: [f64; 2 * MAXN],
    pub jdqd: [f64; 2],
    pub a: [f64; 2],
    pub e: [f64; 2],
}

impl TaskGoal {
    /// Inactive/invalid goals and frames absent from this mechanism contribute no task.
    #[must_use]
    pub fn evaluate(self, model: &mut Model, q: &[f64], qd: &[f64]) -> Option<SubTask> {
        let n = model.n;
        match self {
            Self::Orientation { frame, target, wn } => {
                if !wn.is_finite() || wn <= 0.0 || !target.is_finite() {
                    return None;
                }
                let angle = &model.frame_angles[frame.index(model)?];
                let mut j = [0.0; 2 * MAXN];
                for (i, &k) in angle.k.iter().enumerate().take(n) {
                    j[i] = f64::from(k);
                }
                let e = wrap(target - angle.eval(q));
                let rate: f64 = (0..n).map(|i| j[i] * qd[i]).sum();
                Some(SubTask {
                    role: TaskRole::Additional,
                    rows: 1,
                    j,
                    jdqd: [0.0; 2],
                    a: [wn * wn * e - 2.0 * wn * rate, 0.0],
                    e: [e, 0.0],
                })
            }
            Self::Point { frame, target, wn } => {
                if !wn.is_finite() || wn <= 0.0 || !target.iter().all(|v| v.is_finite()) {
                    return None;
                }
                let (p, j, jdqd) = point_kinematics(model, frame.index(model)?, q, qd);
                let v: [f64; 2] = core::array::from_fn(|r| (0..n).map(|c| j[r * n + c] * qd[c]).sum());
                let e = [target[0] - p[0], target[1] - p[1]];
                let a = core::array::from_fn(|r| wn * wn * e[r] - 2.0 * wn * v[r]);
                Some(SubTask { role: TaskRole::Additional, rows: 2, j, jdqd, a, e })
            }
        }
    }
}

impl Plan {
    /// Built-in orientation and elbow controls precede any additional ordered tasks.
    pub fn sub_tasks(&self, model: &mut Model, q: &[f64], qd: &[f64]) -> Vec<SubTask> {
        let g = self.config.goals;
        [
            (
                TaskRole::Orientation,
                TaskGoal::Orientation { frame: TaskFrame::Tool, target: g.orient_target, wn: g.orient },
            ),
            (
                TaskRole::Elbow,
                TaskGoal::Point { frame: TaskFrame::Body(g.elbow_body), target: g.elbow_target, wn: g.elbow },
            ),
        ]
        .into_iter()
        .chain(self.config.tasks.iter().copied().map(|goal| (TaskRole::Additional, goal)))
        .filter_map(|(role, goal)| {
            goal.evaluate(model, q, qd).map(|mut task| {
                task.role = role;
                task
            })
        })
        .collect()
    }
}

pub const MAX_TASKS: usize = 16;
pub const TASK_WIDTH: usize = 5;
impl TaskGoal {
    /// Compact runtime codec: [kind (0 angle / 1 point), frame (−2 tool / −1 ground / body), x, y, ω].
    /// # Errors
    /// Malformed dimensions, tags, frames, non-finite targets or negative frequencies.
    pub fn parse(flat: &[f64], model: &Model) -> Result<Vec<Self>, String> {
        let (rows, remainder) = flat.as_chunks::<TASK_WIDTH>();
        if !remainder.is_empty() || rows.len() > MAX_TASKS || flat.iter().any(|v| !v.is_finite()) {
            return Err("invalid task list length or values".into());
        }
        rows.iter()
            .map(|&[kind, frame, x, y, wn]| {
                let frame = match frame {
                    -2.0 if model.has_tool() => TaskFrame::Tool,
                    -1.0 => TaskFrame::Ground,
                    body if body >= 0.0 && body.fract() == 0.0 && body < model.desc.nb() as f64 => {
                        TaskFrame::Body(body as usize)
                    }
                    _ => return Err("task frame does not exist".into()),
                };
                if wn < 0.0 {
                    return Err("task frequency must be nonnegative".into());
                }
                match kind {
                    0.0 => Ok(Self::Orientation { frame, target: x, wn }),
                    1.0 => Ok(Self::Point { frame, target: [x, y], wn }),
                    _ => Err("unknown task kind".into()),
                }
            })
            .collect()
    }
    #[must_use]
    pub fn values(self) -> [f64; TASK_WIDTH] {
        let (tag, frame, x, y, wn) = match self {
            Self::Orientation { frame, target, wn } => (0.0, frame, target, 0.0, wn),
            Self::Point { frame, target, wn } => (1.0, frame, target[0], target[1], wn),
        };
        let frame = match frame {
            TaskFrame::Ground => -1.0,
            TaskFrame::Tool => -2.0,
            TaskFrame::Body(i) => i as f64,
        };
        [tag, frame, x, y, wn]
    }
}
