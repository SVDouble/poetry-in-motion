//! Cart-pole controllers: LQR balancing about the upright equilibrium and energy-based swing-up
//! (Åström & Furuta) with collocated partial feedback linearisation.

use super::{Ctx, wrap};
use crate::dynamics::{Terms, VecN};
use crate::lqr::{c2d, dlqr};
use crate::mat::Mat;
use crate::model::Model;

pub const MODE_MANUAL: u32 = 0;
pub const MODE_LQR: u32 = 1;
pub const MODE_SWING_UP: u32 = 2;

pub const PHASE_IDLE: u32 = 0;
pub const PHASE_SWING: u32 = 1;
pub const PHASE_BALANCE: u32 = 2;

#[derive(Debug, Clone)]
pub struct CartPoleCtrl {
    /// Linearisation about q = 0, q̇ = 0: ẋ = A x + B u with x = (x, θ, ẋ, θ̇).
    pub a: Mat<4, 4>,
    pub b: Mat<4, 1>,
    pub k: Mat<1, 4>,
    pub lqr_ok: bool,
    pub q_x: f64,
    pub q_th: f64,
    pub r: f64,
    pub x_ref: f64,
    pub phase: u32,
    /// Pole energy relative to upright, E_p = ½ J θ̇² + m_p g r (cos θ − 1).
    pub e_pole: f64,
    /// Swing-up gains.
    pub k_e: f64,
    pub k_x: f64,
    pub k_v: f64,
    pub a_max: f64,
}

impl Default for CartPoleCtrl {
    fn default() -> Self {
        Self {
            a: Mat::ZERO,
            b: Mat::ZERO,
            k: Mat::ZERO,
            lqr_ok: false,
            q_x: 10.0,
            q_th: 100.0,
            r: 0.1,
            x_ref: 0.0,
            phase: PHASE_IDLE,
            e_pole: 0.0,
            k_e: 12.0,
            k_x: 2.0,
            k_v: 2.5,
            a_max: 12.0,
        }
    }
}

impl CartPoleCtrl {
    /// Linearise about the upright equilibrium and design the LQR gain for the simulation step `dt`.
    pub fn design(&mut self, model: &Model, p: &[f64], dt: f64) {
        let zero = [0.0; 2];
        let t = Terms::eval(model, p, &zero, &zero);
        let mut dn = [0.0; 4];
        (model.dndq)(p, &zero, &mut dn);
        let m0 = Mat([[t.m[0], t.m[1]], [t.m[2], t.m[3]]]);
        let Some(mi) = m0.inverse() else {
            self.lqr_ok = false;
            return;
        };
        let stiff = Mat([[dn[0], dn[1]], [dn[2], dn[3]]]);
        let damp = Mat([[p[model.damping[0]], 0.0], [0.0, p[model.damping[1]]]]);
        let a21 = -(mi * stiff);
        let a22 = -(mi * damp);
        let bu = mi * Mat([[model.b[0]], [model.b[1]]]);
        self.a = Mat::from_fn(|i, j| match (i, j) {
            (0, 2) | (1, 3) => 1.0,
            (2 | 3, 0 | 1) => a21[(i - 2, j)],
            (2 | 3, 2 | 3) => a22[(i - 2, j - 2)],
            _ => 0.0,
        });
        self.b = Mat::from_fn(|i, _| if i >= 2 { bu[(i - 2, 0)] } else { 0.0 });

        let q = Mat::from_fn(|i, j| if i == j { [self.q_x, self.q_th, 1.0, 1.0][i] * dt } else { 0.0 });
        let r = Mat([[self.r * dt]]);
        let (ad, bd) = c2d(&self.a, &self.b, dt);
        match dlqr(&ad, &bd, &q, &r) {
            Some(k) => {
                self.k = k;
                self.lqr_ok = true;
            }
            None => self.lqr_ok = false,
        }
    }

    fn lqr(&self, q: &[f64], qd: &[f64]) -> f64 {
        let e = [q[0] - self.x_ref, wrap(q[1]), qd[0], qd[1]];
        -(0..4).map(|i| self.k[(0, i)] * e[i]).sum::<f64>()
    }

    /// Cart force that realises a desired cart acceleration `a` (collocated partial feedback linearisation):
    /// row 2 of M q̈ + h = τ gives θ̈ = −(h₂ + M₂₁ a) / M₂₂, and row 1 then yields F.
    fn pfl(t: &Terms, h: &[f64; 2], a: f64) -> f64 {
        let thdd = -(h[1] + t.m[2] * a) / t.m[3];
        t.m[0] * a + t.m[1] * thdd + h[0]
    }

    pub fn update(&mut self, mode: u32, ctx: &Ctx<'_>) -> VecN {
        let (q, qd, p) = (ctx.q, ctx.qd, ctx.p);
        let (mp, r, ip, g) = (p[1], p[2], p[3], p[4]);
        let jp = ip + mp * r * r;
        let th = wrap(q[1]);
        self.e_pole = 0.5 * jp * qd[1] * qd[1] + mp * g * r * (th.cos() - 1.0);

        let mut u = [0.0; 4];
        match mode {
            MODE_LQR if self.lqr_ok => {
                self.phase = PHASE_BALANCE;
                u[0] = self.lqr(q, qd);
            }
            MODE_SWING_UP if self.lqr_ok => {
                // Hysteresis between the energy pump and the LQR catch.
                let near = th.abs() < 0.35 && qd[1].abs() < 4.0;
                if self.phase == PHASE_BALANCE && th.abs() > 0.7 || self.phase != PHASE_BALANCE && !near {
                    self.phase = PHASE_SWING;
                } else if near {
                    self.phase = PHASE_BALANCE;
                }
                if self.phase == PHASE_BALANCE {
                    u[0] = self.lqr(q, qd);
                } else {
                    // dE_p/dt = m_p r a θ̇ cos θ  ⇒  a = −k E θ̇ cos θ drives E_p → 0 (the upright orbit).
                    let e_ref = 2.0 * mp * g * r;
                    let mut a = -self.k_e * (self.e_pole / e_ref) * qd[1] * th.cos()
                        - self.k_x * (q[0] - self.x_ref)
                        - self.k_v * qd[0];
                    if qd[1].abs() < 1e-3 && th.cos() < -0.99 {
                        a += 0.5 * self.a_max; // break the symmetry when resting at the bottom
                    }
                    let a = a.clamp(-self.a_max, self.a_max);
                    let cqd = ctx.terms.c_qd(2, qd);
                    let h = [cqd[0] + ctx.terms.n[0], cqd[1] + ctx.terms.n[1]];
                    u[0] = Self::pfl(ctx.terms, &h, a);
                }
            }
            _ => self.phase = PHASE_IDLE,
        }
        u
    }
}
