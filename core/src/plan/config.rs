//! Goal and tuning configuration, independent of cached design and filter state.
use super::TaskGoal;
use crate::dynamics::VecN;

/// Defaults of a fresh plan: reference filter frequency (rad/s), push stiffness (N/m), posture goal (rad/s).
///
/// The browser takes its defaults from these too (`layout::Flag::default_value`).
pub const REF_WN: f64 = 4.0;
pub const PUSH_K: f64 = 200.0;
pub const POSTURE: f64 = 4.0;

/// The numbers that shape how the controllers and the simulation behave beyond the physics.
///
/// They are user settings like the goals, the same for every mechanism; the defaults are what the
/// experiments are verified with.
#[derive(Clone, Copy, Debug)]
pub struct Tuning {
    /// Joint damping (1/s) in what the tool task does not serve (its null space, and near a singularity
    /// the directions its damped inverse lets go of).
    pub null_damping: f64,
    /// Damped least squares of the tool level: below `eps` (relative to the task's size) the damping grows
    /// to `max`.
    pub dls: [f64; 2],
    /// The same for the levels below the tool.
    pub dls_lower: [f64; 2],
    /// Band of relative sizes over which a level fades a used-up direction out of the levels below.
    pub fade: [f64; 2],
    /// How far inside the workspace an unreachable target is aimed, as a fraction of the arm's size.
    pub reach_margin: f64,
    /// Largest change of one coordinate per IK iteration (rad or m), and the IK's least damping.
    pub ik_step: f64,
    pub ik_damping: f64,
    /// How far ahead of the state a regulator's design point may lead on the way to its target (rad | m).
    pub lead: f64,
    /// Largest force of the push on the tool (N): a hand, not a winch.
    pub push_max: f64,
    /// No-load speed of the motors (rad/s | m/s): the drive they can deliver falls linearly to zero at it
    /// (braking is always at full strength), like a DC motor's torque–speed line.
    pub motor_speed: f64,
    /// End stops of limited prismatic joints: stiffness (N/m), damping (N s/m) at the typical
    /// penetration `stop_depth` (m).
    pub stop_k: f64,
    pub stop_c: f64,
    pub stop_depth: f64,
    /// Baumgarte stabilisation rate of the loop-closure constraints (1/s).
    pub baumgarte: f64,
}

impl Default for Tuning {
    fn default() -> Self {
        Self {
            null_damping: 8.0,
            dls: [0.02, 0.01],
            dls_lower: [0.1, 0.05],
            fade: [1e-4, 1e-3],
            reach_margin: 0.005,
            ik_step: 0.05,
            ik_damping: 0.01,
            lead: 0.1,
            push_max: 30.0,
            motor_speed: 100.0,
            stop_k: 2.0e4,
            stop_c: 300.0,
            stop_depth: 0.01,
            baumgarte: 20.0,
        }
    }
}

/// Secondary goals pursued in the null space of the tool task.
#[derive(Clone, Copy, Debug)]
pub struct Goals {
    /// Posture: natural frequency (rad/s) of the pull towards q* (0 = off).
    pub posture: f64,
    /// Manipulability: gain of the gradient ascent on w = √det(J Jᵀ) (0 = off).
    pub manipulability: f64,
    /// Priority 2: tool orientation φ_t → `orient_target`, natural frequency (rad/s, 0 = off).
    pub orient: f64,
    pub orient_target: f64,
    /// Priority 3: the elbow (the origin of body `elbow_body`) → `elbow_target`, natural frequency
    /// (rad/s, 0 = off). The planar counterpart of the arm angle that resolves a 7-DOF arm's redundancy.
    pub elbow: f64,
    pub elbow_target: [f64; 2],
    pub elbow_body: usize,
}

#[derive(Clone, Debug)]
pub struct Configuration {
    /// Additional ordered body/tool tasks, reset on mechanism replacement.
    pub tasks: Vec<TaskGoal>,
    pub q_target: VecN,
    pub tool_target: [f64; 2],
    /// Follow `tool_target` (task controllers directly, joint controllers through IK) instead of q*.
    pub use_tool: bool,
    /// Where a regulating controller (LQR, MPC) is designed while it travels to its target (None: at q*).
    pub setpoint: Option<VecN>,
    pub ref_wn: f64,
    /// Pointer position of a push on the tool, and the push stiffness.
    pub push: Option<[f64; 2]>,
    pub push_k: f64,
    pub goals: Goals,
    pub tuning: Tuning,
}
