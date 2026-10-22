//! Per-slot reference filter and inverse-kinematics state, restarted on state jumps.
use crate::dynamics::VecN;

#[derive(Clone, Debug)]
pub struct References {
    pub q_ref: VecN,
    pub qd_ref: VecN,
    pub qdd_ref: VecN,
    pub ik_goal: VecN,
    pub reachable: bool,
    /// Where the IK aims the tool: the target itself when it is reachable, otherwise (just inside) the
    /// nearest point the mechanism can reach.
    pub ik_point: [f64; 2],
    /// The adaptive part of the IK's damping.
    pub(super) ik_mu: f64,
    pub x_ref: [f64; 2],
    pub xd_ref: [f64; 2],
    pub xdd_ref: [f64; 2],
    /// External force on the tool this step (world frame).
    pub f_ext: [f64; 2],
    pub(super) synced: bool,
}
