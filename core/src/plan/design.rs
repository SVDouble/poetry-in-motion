//! Cached equilibrium and linear design, rebuilt after physical or target changes.
use crate::dynamics::{MatN, VecN};
use crate::mat::DMat;

/// Linearisation in tangent coordinates x = (ξ, ξ̇) with q ≈ q* + Z ξ (Z = I for open chains).
#[derive(Clone, Debug)]
pub struct Lin {
    pub z: DMat,
    pub a: DMat,
    pub b: DMat,
}

#[derive(Clone, Debug)]
pub struct Design {
    /// The equilibrium the controllers aim for (the joint targets on the manifold, passive joints settled).
    pub q_star: VecN,
    /// Where the linearisation, u* and the gains are made: q*, or on the way there the setpoint.
    pub q_lin: VecN,
    /// Tool position at q* (task target when no tool target is set).
    pub x_star: [f64; 2],
    pub u_ff: VecN,
    pub eq_residual: f64,
    /// [B −Aᵀ] spans ℝⁿ: any generalised force can be produced (full actuation on the manifold).
    pub ct_ok: bool,
    /// Degrees of freedom (dimension of the tangent space).
    pub dof: usize,
    pub lin: Option<Lin>,
    /// Can linear feedback hold q*? False when an unstable mode of the linearisation is out of reach of
    /// every input (e.g. a passive bar standing on a point that the inputs only move along it).
    pub stabilisable: bool,
    /// M(q*): shapes joint-space gains (K = ω² M(q*)), so that near q* every closed-loop mode has the
    /// designed natural frequency and damping.
    pub m_star: MatN,
    pub v_star: f64,
    /// Energy scale of the passive bodies, 2 Σ m g |r| (swing-up).
    pub e_ref: f64,
    pub(super) designed_target: Option<VecN>,
}

/// Cache dependency order. A stronger invalidation cannot be cleared by a weaker one.
///
/// Model parameters, joint targets, regulator setpoints and Baumgarte tuning rebuild Plan, then controller.
/// Timestep, controller kind and redesign-marked controller parameters rebuild only the controller.
/// Tool targets and secondary goals affect live references/tasks; they do not change the equilibrium
/// until a regulator moves its setpoint. Persistent settings application invalidates both layers.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum Invalidated {
    Clean,
    Controller,
    Plan,
}
