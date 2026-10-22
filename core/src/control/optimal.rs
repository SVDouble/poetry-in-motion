//! Optimal control about the equilibrium q*: infinite-horizon LQR and receding-horizon MPC with input bounds.
//! Both use the plan's linearisation in tangent coordinates x = (ξ, ξ̇), so they work with loops too.

use super::{Controller, Ctx, Kind, Telemetry};
use crate::MAXN;
use crate::dynamics::VecN;
use crate::lqr::{c2d, dare, dlqr};
use crate::mat::DMat;
use crate::model::Model;
use crate::plan::Plan;
use crate::qp;

/// State and input weights in the metric of the mechanism's own inertia: Q = diag(Q_x M, Q_ẋ M) with M the
/// inertia in tangent coordinates, R = R Bₐᵀ M Bₐ with Bₐ the accelerations the inputs produce (for a fully
/// actuated chain R M⁻¹). A weight then means the same response on a light pendulum and a heavy arm, as
/// K = ω² M(q*) does for the joint-space gains; plain identities would leave a heavy arm feebly held.
fn weights(plan: &Plan, l: &crate::plan::Lin, (qx, qv, rw): (f64, f64, f64), scale: f64) -> (DMat, DMat) {
    let (r, n, nu) = (l.z.cols, l.z.rows, l.b.cols);
    let m = &plan.m_star;
    let mz = DMat::from_fn(r, r, |a, b| {
        (0..n).map(|i| l.z[(i, a)] * (0..n).map(|j| m[i * n + j] * l.z[(j, b)]).sum::<f64>()).sum()
    });
    let q = DMat::from_fn(2 * r, 2 * r, |i, j| match (i < r, j < r) {
        (true, true) => qx * scale * mz[(i, j)],
        (false, false) => qv * scale * mz[(i - r, j - r)],
        _ => 0.0,
    });
    let acc = |i: usize, a: usize| l.b[(r + i, a)];
    let rm = DMat::from_fn(nu, nu, |a, b| {
        let w: f64 = (0..r).map(|i| acc(i, a) * (0..r).map(|j| mz[(i, j)] * acc(j, b)).sum::<f64>()).sum();
        // a whisker of the identity keeps it definite should an input reach no direction at all
        rw * scale * (w + if a == b { 1e-9 } else { 0.0 })
    });
    (q, rm)
}

/// Scale a QP ½ vᵀHv + (Fx)ᵀv to unit diagonal: returns D = diag(H)^(−1/2) and leaves D H D and D F.
fn jacobi(cols: usize, nx: usize, h: &mut [f64], f: &mut [f64]) -> Vec<f64> {
    let scale: Vec<f64> = (0..cols).map(|a| 1.0 / h[a * cols + a].max(1e-300).sqrt()).collect();
    for a in 0..cols {
        for b in 0..cols {
            h[a * cols + b] *= scale[a] * scale[b];
        }
        for b in 0..nx {
            f[a * nx + b] *= scale[a];
        }
    }
    scale
}

/// u = u* − K x with K from the discrete Riccati equation at the simulation step.
#[derive(Debug, Default)]
pub struct Lqr {
    k: Option<DMat>,
    cost: f64,
}

impl Controller for Lqr {
    fn regulates(&self) -> bool {
        true
    }

    fn design(&mut self, plan: &Plan, _model: &mut Model, p: &[f64], dt: f64) {
        self.k = plan
            .lin
            .as_ref()
            .and_then(|l| {
                let (ad, bd) = c2d(&l.a, &l.b, dt);
                let (q, rm) = weights(
                    plan,
                    l,
                    (Kind::Lqr.param(p, "qx"), Kind::Lqr.param(p, "qv"), Kind::Lqr.param(p, "r")),
                    dt,
                );
                dlqr(&ad, &bd, &q, &rm)
            })
            .filter(|k| k.max_abs().is_finite());
    }

    fn update(&mut self, plan: &Plan, model: &mut Model, c: &Ctx<'_>) -> VecN {
        let mut u = [0.0; MAXN];
        let Some(k) = &self.k else { return u };
        let x = plan.tangent_state(model, c.q, c.qd);
        let r = x.len() / 2;
        for a in 0..model.nu {
            u[a] = plan.u_ff[a] - (0..x.len()).map(|j| k[(a, j)] * x[j]).sum::<f64>();
        }
        let (qx, qv, rw) = (
            Kind::Lqr.param(c.params, "qx"),
            Kind::Lqr.param(c.params, "qv"),
            Kind::Lqr.param(c.params, "r"),
        );
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
    /// Jacobi scaling of the input sequence: the QP is solved for v with Δu = D v, D = diag(H)^(−1/2), so its
    /// curvature is near one in every direction (inputs with very different costs — a heavy shoulder and a
    /// light finger — would otherwise leave the gradient method crawling along the cheap ones).
    scale: Vec<f64>,
    /// The scaled input sequence v.
    du: Vec<f64>,
    /// −H⁻¹F: the plan without bounds as a map of the state. Each solve starts from it, clipped to the bounds:
    /// exact when no bound is active (MPC is then LQR), and a near start when some are.
    free: Vec<f64>,
    scratch: Vec<f64>,
    since: usize,
    cost: f64,
    iterations: usize,
    active: usize,
    plan_u: Vec<f64>,
    predicted: Vec<f64>,
}

impl Controller for Mpc {
    fn regulates(&self) -> bool {
        true
    }

    fn reset(&mut self) {
        self.du.iter_mut().for_each(|v| *v = 0.0);
        self.since = usize::MAX;
    }

    fn design(&mut self, plan: &Plan, model: &mut Model, p: &[f64], dt: f64) {
        let get = |key| Kind::Mpc.param(p, key);
        let Some(l) = &plan.lin else {
            self.steps = 0;
            return;
        };
        let (nx, nu) = (l.a.rows, model.nu);
        let step = get("step").max(dt);
        let steps = (get("horizon").round() as usize).clamp(1, (MAX_VARS / nu.max(1)).max(1));
        let (ad, bd) = c2d(&l.a, &l.b, step);
        let (q, r) = weights(plan, l, (get("qx"), get("qv"), get("r")), step);
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
                        + if a / nu == b / nu { r[(a % nu, b % nu)] } else { 0.0 };
            }
            for b in 0..nx {
                f[a * nx + b] = (0..rows).map(|row| gamma[row * cols + a] * qp[row * nx + b]).sum();
            }
        }
        if !h.iter().chain(&f).all(|v| v.is_finite()) {
            // the prediction overflowed (a strongly unstable linearisation): no MPC here
            self.steps = 0;
            return;
        }
        let scale = jacobi(cols, nx, &mut h, &mut f);
        let free = qp::unconstrained(cols, nx, &h, &f).unwrap_or_default();
        self.lip = qp::lipschitz(cols, &h);
        // a redesign along the way (the set-point moved, a weight changed) keeps the current plan as the warm
        // start and its timing: a cold start would need many more iterations than one step allows
        let same = self.du.len() == cols && self.steps == steps;
        let (du, since) = if same {
            let du = (0..cols).map(|a| self.du[a] * self.scale[a] / scale[a]).collect();
            (du, self.since)
        } else {
            (vec![0.0; cols], usize::MAX)
        };
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
            scale,
            free,
            du,
            scratch: vec![0.0; 3 * cols],
            since,
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
            let lo: Vec<f64> = (0..cols).map(|a| (-c.umax - plan.u_ff[a % nu]) / self.scale[a]).collect();
            let hi: Vec<f64> = (0..cols).map(|a| (c.umax - plan.u_ff[a % nu]) / self.scale[a]).collect();
            if self.free.is_empty() {
                // warm start: shift last solution by one stage
                self.du.copy_within(nu.., 0);
            } else {
                for a in 0..cols {
                    let v: f64 = (0..nx).map(|b| self.free[a * nx + b] * x0[b]).sum();
                    self.du[a] = v.clamp(lo[a], hi[a]);
                }
            }
            let iters = Kind::Mpc.param(c.params, "iters").round() as usize;
            self.iterations =
                qp::solve_box(&self.h, &g, &lo, &hi, self.lip, &mut self.du, &mut self.scratch, iters);
            if !self.du.iter().all(|v| v.is_finite()) {
                self.du.fill(0.0); // never apply a broken plan: hold u* and start the next solve cold
            }
            self.active =
                (0..cols).filter(|&a| self.du[a] <= lo[a] + 1e-9 || self.du[a] >= hi[a] - 1e-9).count();
            let hu: Vec<f64> =
                (0..cols).map(|a| (0..cols).map(|b| self.h[a * cols + b] * self.du[b]).sum()).collect();
            self.cost = (0..cols).map(|a| 0.5 * self.du[a] * hu[a] + g[a] * self.du[a]).sum();
            let r = nx / 2;
            let du: Vec<f64> = (0..cols).map(|a| self.scale[a] * self.du[a]).collect();
            for k in 0..self.steps {
                self.plan_u[k] = plan.u_ff[0] + du[k * nu];
                let row = |i: usize| {
                    (0..nx).map(|j| self.phi[(k * nx + i) * nx + j] * x0[j]).sum::<f64>()
                        + (0..cols).map(|j| self.gamma[(k * nx + i) * cols + j] * du[j]).sum::<f64>()
                };
                self.predicted[k] = (0..r).map(|i| row(i).powi(2)).sum::<f64>().sqrt();
            }
        }
        for a in 0..nu {
            u[a] = plan.u_ff[a] + self.scale[a] * self.du[a];
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
