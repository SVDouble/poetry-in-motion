//! Two-link arm controllers (MLS ch. 4 §5): PD, PD + gravity compensation and computed torque,
//! tracking a smooth joint reference produced from a Cartesian target by inverse kinematics.

use super::{Ctx, wrap};
use crate::dynamics::{VecN, mat_vec};

pub const MODE_PASSIVE: u32 = 0;
pub const MODE_PD: u32 = 1;
pub const MODE_PD_GRAVITY: u32 = 2;
pub const MODE_COMPUTED_TORQUE: u32 = 3;

#[derive(Debug, Clone)]
pub struct ArmCtrl {
    pub target: [f64; 2],
    /// +1 / −1 selects the elbow branch of the inverse kinematics.
    pub elbow: f64,
    pub reachable: bool,
    pub goal: [f64; 2],
    /// Second-order reference filter state θ_d, θ̇_d, θ̈_d.
    pub q_ref: [f64; 2],
    pub qd_ref: [f64; 2],
    pub qdd_ref: [f64; 2],
    pub ref_wn: f64,
    pub ct_wn: f64,
    pub kp: [f64; 2],
    pub kd: [f64; 2],
    initialised: bool,
}

impl Default for ArmCtrl {
    fn default() -> Self {
        Self {
            target: [1.2, 0.8],
            elbow: 1.0,
            reachable: true,
            goal: [0.0; 2],
            q_ref: [0.0; 2],
            qd_ref: [0.0; 2],
            qdd_ref: [0.0; 2],
            ref_wn: 4.0,
            ct_wn: 12.0,
            kp: [120.0, 60.0],
            kd: [20.0, 8.0],
            initialised: false,
        }
    }
}

/// Closed-form inverse kinematics of the planar RR arm. Unreachable targets are projected onto the
/// workspace boundary; returns `(θ, reachable)`.
#[must_use]
pub fn inverse_kinematics(l1: f64, l2: f64, x: f64, y: f64, elbow: f64) -> ([f64; 2], bool) {
    let d = (x * x + y * y - l1 * l1 - l2 * l2) / (2.0 * l1 * l2);
    let reachable = (-1.0..=1.0).contains(&d);
    let d = d.clamp(-1.0, 1.0);
    let t2 = elbow.signum() * (1.0 - d * d).max(0.0).sqrt().atan2(d);
    let t1 = y.atan2(x) - (l2 * t2.sin()).atan2(l1 + l2 * t2.cos());
    ([t1, t2], reachable)
}

impl ArmCtrl {
    /// Restart the reference at the current configuration (after reset or mode change).
    pub const fn sync(&mut self, q: &[f64]) {
        self.q_ref = [q[0], q[1]];
        self.qd_ref = [0.0; 2];
        self.qdd_ref = [0.0; 2];
        self.initialised = true;
    }

    fn advance_reference(&mut self, l1: f64, l2: f64, dt: f64) {
        let (goal, ok) = inverse_kinematics(l1, l2, self.target[0], self.target[1], self.elbow);
        self.reachable = ok;
        // choose the 2π-equivalent goal closest to the current reference (no needless full turns)
        for i in 0..2 {
            self.goal[i] = self.q_ref[i] + wrap(goal[i] - self.q_ref[i]);
        }
        let wn = self.ref_wn;
        for i in 0..2 {
            self.qdd_ref[i] = wn * wn * (self.goal[i] - self.q_ref[i]) - 2.0 * wn * self.qd_ref[i];
            self.qd_ref[i] += self.qdd_ref[i] * dt;
            self.q_ref[i] += self.qd_ref[i] * dt;
        }
    }

    pub fn update(&mut self, mode: u32, ctx: &Ctx<'_>) -> VecN {
        let (q, qd, p, t) = (ctx.q, ctx.qd, ctx.p, ctx.terms);
        if !self.initialised {
            self.sync(q);
        }
        self.advance_reference(p[2], p[3], ctx.dt);
        let e = [self.q_ref[0] - q[0], self.q_ref[1] - q[1]];
        let ed = [self.qd_ref[0] - qd[0], self.qd_ref[1] - qd[1]];

        let mut u = [0.0; 4];
        match mode {
            MODE_PD | MODE_PD_GRAVITY => {
                let mut dv = [0.0; 2];
                if mode == MODE_PD_GRAVITY {
                    (ctx.model.dvdq)(p, q, &mut dv);
                }
                for i in 0..2 {
                    u[i] = self.kp[i] * e[i] + self.kd[i] * ed[i] + dv[i];
                }
            }
            MODE_COMPUTED_TORQUE => {
                // τ = M(θ)(θ̈_d + K_v ė + K_p e) + C θ̇ + N  ⇒  ë + K_v ė + K_p e = 0
                let (kp, kv) = (self.ct_wn * self.ct_wn, 2.0 * self.ct_wn);
                let v = [self.qdd_ref[0] + kv * ed[0] + kp * e[0], self.qdd_ref[1] + kv * ed[1] + kp * e[1]];
                let mv = mat_vec(2, &t.m, &v);
                let cqd = t.c_qd(2, qd);
                for i in 0..2 {
                    u[i] = mv[i] + cqd[i] + t.n[i];
                }
            }
            _ => {}
        }
        u
    }
}
