//! Controllers. Each one maps the current state (and the already-evaluated M, C, N) to actuator inputs u.

pub mod arm;
pub mod cartpole;

use crate::dynamics::{Terms, VecN};
use crate::model::Model;

/// Everything a controller may look at during one control step.
#[derive(Debug)]
pub struct Ctx<'a> {
    pub model: &'static Model,
    pub p: &'a [f64],
    pub q: &'a [f64],
    pub qd: &'a [f64],
    pub terms: &'a Terms,
    pub dt: f64,
}

#[derive(Debug, Clone)]
pub enum Controller {
    CartPole(cartpole::CartPoleCtrl),
    Arm(arm::ArmCtrl),
}

impl Controller {
    /// Returns u (actuator space) for the given mode; mode 0 is always "manual / passive".
    pub fn update(&mut self, mode: u32, ctx: &Ctx<'_>) -> VecN {
        match self {
            Self::CartPole(c) => c.update(mode, ctx),
            Self::Arm(c) => c.update(mode, ctx),
        }
    }
}

/// Wrap an angle to (−π, π].
#[must_use]
pub fn wrap(a: f64) -> f64 {
    use core::f64::consts::{PI, TAU};
    let w = (a + PI).rem_euclid(TAU) - PI;
    if w <= -PI { w + TAU } else { w }
}
