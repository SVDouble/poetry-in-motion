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

/// Per-step task: row-major J, J̇q̇, desired acceleration ω²e − 2ωẋ and error.
#[derive(Clone, Copy, Debug)]
pub struct SubTask {
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
                Some(SubTask { rows: 2, j, jdqd, a, e })
            }
        }
    }
}

impl Plan {
    /// Built-in orientation and elbow controls precede any additional ordered tasks.
    pub fn sub_tasks(&self, model: &mut Model, q: &[f64], qd: &[f64]) -> Vec<SubTask> {
        let g = self.config.goals;
        [
            TaskGoal::Orientation { frame: TaskFrame::Tool, target: g.orient_target, wn: g.orient },
            TaskGoal::Point { frame: TaskFrame::Body(g.elbow_body), target: g.elbow_target, wn: g.elbow },
        ]
        .into_iter()
        .chain(self.config.tasks.iter().copied())
        .filter_map(|goal| goal.evaluate(model, q, qd))
        .collect()
    }
}
