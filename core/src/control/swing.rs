//! Energy-based swing-up for underactuated mechanisms (Åström–Furuta / Spong), with an LQR catch near q*.

use super::optimal::Lqr;
use super::{Controller, Ctx, Telemetry};
use crate::MAXN;
use crate::dynamics::{MAXK, VecN, lstsq};
use crate::model::Model;
use crate::plan::Plan;

const PUMPING: u32 = 1;
const CATCHING: u32 = 2;

/// Swing-up: choose the actuated acceleration v so that the power fed into the passive coordinates,
/// −q̇_uᵀ M_ua v, drives the energy towards that of the target; realise v by collocated partial feedback
/// linearisation (generalised to loops); hand over to an LQR near q*.
#[derive(Debug, Default)]
pub struct SwingUp {
    lqr: Lqr,
    phase: u32,
    energy: f64,
    energy_ref: f64,
    coupling: f64,
}

impl Controller for SwingUp {
    fn reset(&mut self) {
        self.phase = 0;
    }

    fn design(&mut self, plan: &Plan, model: &mut Model, _p: &[f64], dt: f64) {
        // the catch uses the LQR with its default weights
        self.lqr.design(plan, model, &super::Kind::Lqr.defaults(), dt);
        self.energy_ref = plan.e_ref;
        self.coupling = 0.0;
    }

    fn update(&mut self, plan: &Plan, model: &mut Model, c: &Ctx<'_>) -> VecN {
        let n = model.n;
        let (q, qd, t) = (c.q, c.qd, c.terms);
        // energy relative to the target: of the passive motion (open chains) or total (loops)
        let en = model.energies(q, qd);
        let un = model.unactuated.clone();
        let ke = if model.closed() {
            en[0]
        } else {
            un.iter()
                .flat_map(|&i| un.iter().map(move |&j| (i, j)))
                .map(|(i, j)| 0.5 * t.m[i * n + j] * qd[i] * qd[j])
                .sum()
        };
        self.energy = ke + en[1] - plan.v_star;
        if self.coupling == 0.0 {
            self.coupling = un
                .iter()
                .flat_map(|&u| model.actuated.iter().map(move |&a| (u, a)))
                .map(|(u, a)| t.m[u * n + a].abs())
                .fold(1e-6, f64::max);
        }

        let watched: Vec<usize> = if model.closed() { (0..n).collect() } else { un };
        let err = |i: usize| Plan::error(model, i, q[i], plan.q_star[i]).abs();
        let near = watched.iter().all(|&i| err(i) < 0.35 && qd[i].abs() < 4.0);
        let far = watched.iter().any(|&i| err(i) > 0.7);
        if self.phase == CATCHING && far || self.phase != CATCHING && !near {
            self.phase = PUMPING;
        } else if near {
            self.phase = CATCHING;
        }
        if self.phase == CATCHING {
            return self.lqr.update(plan, model, c);
        }
        self.pump(plan, model, c)
    }

    fn telemetry(&self, t: &mut Telemetry<'_>) {
        t.put("energy", &[self.energy]);
        t.put("energy_ref", &[self.energy_ref]);
    }

    fn phase(&self) -> u32 {
        self.phase
    }
}

impl SwingUp {
    /// Partial feedback linearisation generalised to loops: solve
    /// `M q̈ + h + Aᵀλ = B u`, `A q̈ + Ȧ q̇ = 0`, `q̈_a = v` for (q̈_u, λ, u).
    fn pump(&self, plan: &Plan, model: &Model, c: &Ctx<'_>) -> VecN {
        let (q, qd, t) = (c.q, c.qd, c.terms);
        let n = model.n;
        let (ke, kc, kv, a_max) = (c.params[0], c.params[1], c.params[2], c.params[3]);
        let (act, un) = (&model.actuated, &model.unactuated);
        let at_rest = qd[..n].iter().all(|v| v.abs() < 1e-3);
        let mut v = [0.0; MAXN];
        for (a, &i) in act.iter().enumerate() {
            let w: f64 = -un.iter().map(|&k| t.m[k * n + i] * qd[k]).sum::<f64>();
            let pump = ke * (-self.energy / self.energy_ref) * w / self.coupling;
            let centre = kc * Plan::error(model, i, q[i], plan.q_star[i]) - kv * qd[i];
            let mut va = pump + centre;
            if at_rest && self.energy < -0.5 * self.energy_ref {
                va += 0.5 * a_max; // break the symmetry when resting at the bottom
            }
            v[a] = va.clamp(-a_max, a_max);
        }
        let h = t.h(n, qd);
        let m = c.k.map_or(0, |k| k.m);
        let (nuu, nu) = (un.len(), act.len());
        let (rows, cols) = (n + m, nuu + m + nu);
        let mut g = [0.0; MAXK * (MAXN + MAXK)];
        let mut rhs = [0.0; MAXK];
        let a_of = |r: usize, j: usize| c.k.map_or(0.0, |k| k.a[r * n + j]);
        for i in 0..n {
            for (col, &j) in un.iter().enumerate() {
                g[i * cols + col] = t.m[i * n + j];
            }
            for r in 0..m {
                g[i * cols + nuu + r] = a_of(r, i);
            }
            for a in 0..nu {
                g[i * cols + nuu + m + a] = -model.b[i * nu + a];
            }
            rhs[i] = -h[i] - act.iter().enumerate().map(|(a, &j)| t.m[i * n + j] * v[a]).sum::<f64>();
        }
        for r in 0..m {
            for (col, &j) in un.iter().enumerate() {
                g[(n + r) * cols + col] = a_of(r, j);
            }
            rhs[n + r] = -c.k.map_or(0.0, |k| k.adq[r])
                - act.iter().enumerate().map(|(a, &j)| a_of(r, j) * v[a]).sum::<f64>();
        }
        let (x, _) = lstsq(rows, cols, &g, &rhs);
        core::array::from_fn(|a| if a < nu { x[nuu + m + a] } else { 0.0 })
    }
}
