//! Optimal control about the equilibrium q*: infinite-horizon LQR and receding-horizon MPC with input bounds.
//! Both use the plan's linearisation in tangent coordinates x = (ξ, ξ̇), so they work with loops too.

use super::{Controller, Ctx, Telemetry};
use crate::MAXN;
use crate::dynamics::VecN;
use crate::lqr::{c2d, dare, dlqr};
use crate::mat::DMat;
use crate::model::Model;
use crate::plan::Plan;
use crate::qp;

fn weights(r: usize, qx: f64, qv: f64, scale: f64) -> DMat {
    DMat::from_fn(2 * r, 2 * r, |i, j| {
        if i != j {
            0.0
        } else if i < r {
            qx * scale
        } else {
            qv * scale
        }
    })
}

/// u = u* − K x with K from the discrete Riccati equation at the simulation step.
#[derive(Debug, Default)]
pub struct Lqr {
    k: Option<DMat>,
    cost: f64,
}

impl Controller for Lqr {
    fn design(&mut self, plan: &Plan, model: &mut Model, p: &[f64], dt: f64) {
        self.k = plan.lin.as_ref().and_then(|l| {
            let r = l.z.cols;
            let (ad, bd) = c2d(&l.a, &l.b, dt);
            dlqr(&ad, &bd, &weights(r, p[0], p[1], dt), &DMat::identity(model.nu).scale(p[2] * dt))
        });
    }

    fn update(&mut self, plan: &Plan, model: &mut Model, c: &Ctx<'_>) -> VecN {
        let mut u = [0.0; MAXN];
        let Some(k) = &self.k else { return u };
        let x = plan.tangent_state(model, c.q, c.qd);
        let r = x.len() / 2;
        for a in 0..model.nu {
            u[a] = plan.u_ff[a] - (0..x.len()).map(|j| k[(a, j)] * x[j]).sum::<f64>();
        }
        let (qx, qv, rw) = (c.params[0], c.params[1], c.params[2]);
        self.cost = (0..r).map(|i| qx * x[i] * x[i] + qv * x[r + i] * x[r + i]).sum::<f64>()
            + (0..model.nu).map(|a| rw * (u[a] - plan.u_ff[a]).powi(2)).sum::<f64>();
        u
    }

    fn telemetry(&self, t: &mut Telemetry<'_>) {
        if let Some(k) = &self.k {
            t.put("k", &k.data);
        }
        t.put("cost", &[self.cost]);
    }
}

/// Largest problem the online solver accepts (horizon × inputs); longer horizons are shortened.
const MAX_VARS: usize = 180;

/// Linear MPC in condensed form. Over the horizon X = Φ x₀ + Γ U, the cost
/// `Σ xₖᵀ Q xₖ + uₖᵀ R uₖ + x_Nᵀ P x_N` becomes `½ Uᵀ H U + (F x₀)ᵀ U` with H = Γᵀ Q̄ Γ + R̄, F = Γᵀ Q̄ Φ,
/// minimised subject to the actuator bounds every Δt; the first input is applied (receding horizon).
#[derive(Debug, Default)]
pub struct Mpc {
    nx: usize,
    nu: usize,
    steps: usize,
    stride: usize,
    h: Vec<f64>,
    f: Vec<f64>,
    phi: Vec<f64>,
    gamma: Vec<f64>,
    lip: f64,
    du: Vec<f64>,
    scratch: Vec<f64>,
    since: usize,
    cost: f64,
    iterations: usize,
    active: usize,
    plan_u: Vec<f64>,
    predicted: Vec<f64>,
}

impl Controller for Mpc {
    fn reset(&mut self) {
        self.du.iter_mut().for_each(|v| *v = 0.0);
        self.since = usize::MAX;
    }

    fn design(&mut self, plan: &Plan, model: &mut Model, p: &[f64], dt: f64) {
        let Some(l) = &plan.lin else {
            self.steps = 0;
            return;
        };
        let (nx, nu) = (l.a.rows, model.nu);
        let step = p[1].max(dt);
        let steps = (p[0].round() as usize).clamp(1, (MAX_VARS / nu.max(1)).max(1));
        let (ad, bd) = c2d(&l.a, &l.b, step);
        let q = weights(nx / 2, p[2], p[3], step);
        let r = DMat::identity(nu).scale(p[4] * step);
        let terminal = dare(&ad, &bd, &q, &r).unwrap_or_else(|| q.clone());

        // prediction matrices: x_{k+1} = Φ_k x₀ + Σ_{j ≤ k} Γ_{k,j} u_j
        let (rows, cols) = (steps * nx, steps * nu);
        let mut phi = vec![0.0; rows * nx];
        let mut gamma = vec![0.0; rows * cols];
        let mut pow = ad.clone(); // Ad^{k+1}
        let mut blocks: Vec<DMat> = vec![bd]; // Ad^i Bd
        for k in 0..steps {
            for i in 0..nx {
                for j in 0..nx {
                    phi[(k * nx + i) * nx + j] = pow[(i, j)];
                }
            }
            for jstep in 0..=k {
                let blk = &blocks[k - jstep];
                for i in 0..nx {
                    for a in 0..nu {
                        gamma[(k * nx + i) * cols + jstep * nu + a] = blk[(i, a)];
                    }
                }
            }
            pow = &pow * &ad;
            let next = &ad * &blocks[k];
            blocks.push(next);
        }
        // H = Γᵀ Q̄ Γ + R̄, F = Γᵀ Q̄ Φ (Q̄ = diag(Q, …, Q, P))
        let qbar = |k: usize, i: usize, j: usize| if k + 1 == steps { terminal[(i, j)] } else { q[(i, j)] };
        let mut qg = vec![0.0; rows * cols]; // Q̄ Γ
        let mut qp = vec![0.0; rows * nx]; // Q̄ Φ
        for k in 0..steps {
            for i in 0..nx {
                for j in 0..nx {
                    let w = qbar(k, i, j);
                    if w == 0.0 {
                        continue;
                    }
                    for c in 0..cols {
                        qg[(k * nx + i) * cols + c] += w * gamma[(k * nx + j) * cols + c];
                    }
                    for c in 0..nx {
                        qp[(k * nx + i) * nx + c] += w * phi[(k * nx + j) * nx + c];
                    }
                }
            }
        }
        let mut h = vec![0.0; cols * cols];
        let mut f = vec![0.0; cols * nx];
        for a in 0..cols {
            for b in 0..cols {
                h[a * cols + b] =
                    (0..rows).map(|row| gamma[row * cols + a] * qg[row * cols + b]).sum::<f64>()
                        + if a == b { r[(a % nu, a % nu)] } else { 0.0 };
            }
            for b in 0..nx {
                f[a * nx + b] = (0..rows).map(|row| gamma[row * cols + a] * qp[row * nx + b]).sum();
            }
        }
        self.lip = qp::lipschitz(cols, &h);
        *self = Self {
            nx,
            nu,
            steps,
            stride: ((step / dt).round() as usize).max(1),
            h,
            f,
            phi,
            gamma,
            lip: self.lip,
            du: vec![0.0; cols],
            scratch: vec![0.0; 3 * cols],
            since: usize::MAX,
            plan_u: vec![0.0; steps],
            predicted: vec![0.0; steps],
            ..Self::default()
        };
    }

    fn update(&mut self, plan: &Plan, model: &mut Model, c: &Ctx<'_>) -> VecN {
        let mut u = [0.0; MAXN];
        if self.steps == 0 {
            return u;
        }
        let (nx, nu, cols) = (self.nx, self.nu, self.steps * self.nu);
        self.since = self.since.saturating_add(1);
        if self.since >= self.stride {
            self.since = 0;
            let x0 = plan.tangent_state(model, c.q, c.qd);
            let g: Vec<f64> = (0..cols).map(|a| (0..nx).map(|b| self.f[a * nx + b] * x0[b]).sum()).collect();
            let lo: Vec<f64> = (0..cols).map(|a| -c.umax - plan.u_ff[a % nu]).collect();
            let hi: Vec<f64> = (0..cols).map(|a| c.umax - plan.u_ff[a % nu]).collect();
            // warm start: shift last solution by one stage
            self.du.copy_within(nu.., 0);
            let iters = c.params[5].round() as usize;
            self.iterations =
                qp::solve_box(&self.h, &g, &lo, &hi, self.lip, &mut self.du, &mut self.scratch, iters);
            self.active =
                (0..cols).filter(|&a| self.du[a] <= lo[a] + 1e-9 || self.du[a] >= hi[a] - 1e-9).count();
            let hu: Vec<f64> =
                (0..cols).map(|a| (0..cols).map(|b| self.h[a * cols + b] * self.du[b]).sum()).collect();
            self.cost = (0..cols).map(|a| 0.5 * self.du[a] * hu[a] + g[a] * self.du[a]).sum();
            let r = nx / 2;
            for k in 0..self.steps {
                self.plan_u[k] = plan.u_ff[0] + self.du[k * nu];
                let row = |i: usize| {
                    (0..nx).map(|j| self.phi[(k * nx + i) * nx + j] * x0[j]).sum::<f64>()
                        + (0..cols).map(|j| self.gamma[(k * nx + i) * cols + j] * self.du[j]).sum::<f64>()
                };
                self.predicted[k] = (0..r).map(|i| row(i).powi(2)).sum::<f64>().sqrt();
            }
        }
        for a in 0..nu {
            u[a] = plan.u_ff[a] + self.du[a];
        }
        u
    }

    fn telemetry(&self, t: &mut Telemetry<'_>) {
        t.put("plan", &self.plan_u);
        t.put("predicted", &self.predicted);
        t.put("cost", &[self.cost]);
        t.put("iterations", &[self.iterations as f64]);
        t.put("active", &[self.active as f64]);
        t.put("size", &[self.steps as f64]);
    }
}
