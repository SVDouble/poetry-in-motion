//! Task-space controllers acting on the tool point: virtual model control (energy shaping), impedance and
//! admittance.
//!
//! Further goals are stacked below the tool position with strict priorities: the tool
//! orientation, then the elbow, then posture and manipulability. Each level acts only in the null space of
//! the levels above it, in the metric of M (dynamically consistent), so it cannot disturb them.

use super::{Controller, Ctx, Kind, Telemetry};
use crate::MAXN;
use crate::dynamics::{Constraints, Terms, VecN, inverse_dynamics, mat_vec};
use crate::mat::cholesky_solve;
use crate::model::Model;
use crate::plan::{Plan, SubTask, Tool, Tuning, manipulability, resolve_accel};

type MatN = [f64; MAXN * MAXN];

/// Task-space inertia Λ = (J M⁻¹ Jᵀ)⁻¹ (2 × 2) and M⁻¹ Jᵀ (n × 2, row-major).
fn task_inertia(n: usize, t: &Terms, jp: &[f64]) -> Option<([f64; 4], [f64; 2 * MAXN])> {
    let mut minv_jt = [0.0; 2 * MAXN];
    for r in 0..2 {
        let rhs: VecN = core::array::from_fn(|c| if c < n { jp[r * n + c] } else { 0.0 });
        let mut col = [0.0; MAXN];
        if !cholesky_solve(n, &t.m, &rhs, &mut col) {
            return None;
        }
        for c in 0..n {
            minv_jt[c * 2 + r] = col[c];
        }
    }
    let li = |a: usize, b: usize| (0..n).map(|c| jp[a * n + c] * minv_jt[c * 2 + b]).sum::<f64>();
    let (a, b, d) = (li(0, 0) + 1e-9, li(0, 1), li(1, 1) + 1e-9);
    let det = a.mul_add(d, -(b * b));
    Some(([d / det, -b / det, -b / det, a / det], minv_jt))
}

/// Effective task mass for the damping of a virtual spring: the harmonic mean of Λ's eigenvalues,
/// 2 / tr(J M⁻¹ Jᵀ). (Their plain mean is unbounded at a singular pose, and so would be the damping gain:
/// far too stiff for a 1 ms controller.)
fn task_mass(n: usize, t: &Terms, jp: &[f64]) -> f64 {
    task_inertia(n, t, jp).map_or(1.0, |(_, minv_jt)| {
        let tr: f64 = (0..2).map(|r| (0..n).map(|c| jp[r * n + c] * minv_jt[c * 2 + r]).sum::<f64>()).sum();
        2.0 / tr.max(1e-12)
    })
}

/// One level of a [`Stack`]: its restricted Jacobian Ĵ = J P (rows × n), the damped task inertia Λ̂ and the
/// dynamically consistent inverse Ĵ# = M⁻¹ Ĵᵀ Λ̂ (n × rows, `pinv[c * 2 + r]`) used for its command.
#[derive(Clone, Copy, Debug)]
struct Level {
    rows: usize,
    jhat: [f64; 2 * MAXN],
    lam: [f64; 4],
    pinv: [f64; 2 * MAXN],
}

/// Adaptive damped least squares (Nakamura & Hanafusa; Chiaverini): for an eigen-direction μ of Ĵ M⁻¹ Ĵᵀ
/// the command uses μ / (μ² + δ²) with δ = 0 while μ ≥ ε (exact) and δ = δ_max (1 − μ/ε) below it, so the
/// gain stays bounded (≈ 1 / 2δ_max) and fades to zero at a singularity instead of exploding next to it.
/// ε and δ_max are fractions of the task's own (unprojected) size; lower levels are best-effort, damped
/// earlier and more.
fn damped_inverse(mu: f64, eps: f64, dmax: f64) -> f64 {
    if mu <= 0.0 {
        return 0.0;
    }
    let d = if mu < eps { dmax * (1.0 - mu / eps) } else { 0.0 };
    mu / (mu * mu + d * d)
}

/// How much of an eigen-direction (relative size r = μ / task size) a level keeps out of the levels below:
/// all of it above `hi`, none below `lo`, a smoothstep in between (continuous, so nothing switches).
fn keep_weight(r: f64, [lo, hi]: [f64; 2]) -> f64 {
    let t = ((r - lo) / (hi - lo).max(1e-300)).clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

/// Strict task priorities in the metric of M (Khatib; Siciliano–Slotine): level k acts only in the null
/// space of levels 1 … k−1, through the motion projector `P_k = P_{k−1} − Ĵ#_k Ĵ_k` (P₀ = I). Torques of
/// the form Ĵ_kᵀ F or P_kᵀ τ produce no acceleration of any level above k.
#[derive(Debug)]
struct Stack {
    n: usize,
    tuning: Tuning,
    minv: MatN,
    p: MatN,
    levels: usize,
}

impl Stack {
    fn new(n: usize, m: &MatN, tuning: Tuning) -> Option<Self> {
        let mut minv = [0.0; MAXN * MAXN];
        for c in 0..n {
            let e: VecN = core::array::from_fn(|k| f64::from(u8::from(k == c)));
            let mut col = [0.0; MAXN];
            if !cholesky_solve(n, m, &e, &mut col) {
                return None;
            }
            for r in 0..n {
                minv[r * n + c] = col[r];
            }
        }
        let p = core::array::from_fn(|k| if k < n * n && k % (n + 1) == 0 { 1.0 } else { 0.0 });
        Some(Self { n, tuning, minv, p, levels: 0 })
    }

    /// In constraint tangent coordinates Z: M_r = Zᵀ M Z and H = Z M_r⁻¹ Zᵀ.
    /// H maps torques to feasible accelerations; P = H M projects preferences onto the tangent.
    /// All later task corrections use H, so A Δq̈ = 0 at every priority level.
    fn constrained(n: usize, m: &MatN, k: Option<&Constraints>, tuning: Tuning) -> Option<Self> {
        let Some(k) = k else { return Self::new(n, m, tuning) };
        let z = k.tangent_basis();
        let r = z.cols;
        let mut mr = [0.0; MAXN * MAXN];
        for i in 0..r {
            for j in 0..r {
                mr[i * r + j] = (0..n)
                    .flat_map(|a| (0..n).map(move |b| (a, b)))
                    .map(|(a, b)| z[(a, i)] * m[a * n + b] * z[(b, j)])
                    .sum();
            }
        }
        let reduced = Self::new(r, &mr, tuning)?;
        let mut minv = [0.0; MAXN * MAXN];
        for a in 0..n {
            for b in 0..n {
                minv[a * n + b] = (0..r)
                    .flat_map(|i| (0..r).map(move |j| (i, j)))
                    .map(|(i, j)| z[(a, i)] * reduced.minv[i * r + j] * z[(b, j)])
                    .sum();
            }
        }
        let p = core::array::from_fn(|ix| {
            let (i, j) = (ix / n, ix % n);
            if i < n { (0..n).map(|a| minv[i * n + a] * m[a * n + j]).sum() } else { 0.0 }
        });
        Some(Self { n, tuning, minv, p, levels: 0 })
    }

    /// Add the next-lower level with Jacobian `j` (rows × n) and shrink the null space.
    fn push(&mut self, rows: usize, j: &[f64]) -> Level {
        let n = self.n;
        let mut jhat = [0.0; 2 * MAXN];
        for r in 0..rows {
            for c in 0..n {
                jhat[r * n + c] = (0..n).map(|k| j[r * n + k] * self.p[k * n + c]).sum();
            }
        }
        // M⁻¹ Ĵᵀ (n × rows) and A = Ĵ M⁻¹ Ĵᵀ. Λ̂ is A's truncated inverse: directions the levels above leave
        // free are inverted exactly (so the projection is exact), directions they have used up (eigenvalues
        // below a fraction of the task's own, unprojected size) are dropped instead of amplified
        let mut mj = [0.0; 2 * MAXN];
        for c in 0..n {
            for r in 0..rows {
                mj[c * 2 + r] = (0..n).map(|k| self.minv[c * n + k] * jhat[r * n + k]).sum();
            }
        }
        let a = |r: usize, s: usize| (0..n).map(|c| jhat[r * n + c] * mj[c * 2 + s]).sum::<f64>();
        let scale: f64 = (0..rows)
            .map(|r| {
                (0..n)
                    .map(|c| j[r * n + c] * (0..n).map(|k| self.minv[c * n + k] * j[r * n + k]).sum::<f64>())
                    .sum::<f64>()
            })
            .sum::<f64>()
            .max(1e-300);
        // per eigen-direction μ of A: the projector uses the exact inverse 1/μ (lower levels must be confined
        // exactly), faded out smoothly for directions the levels above (or a singular pose) have all but used
        // up — a hard cut-off would switch that direction in and out of the null space as the arm hovers at
        // the threshold, kicking lower levels on and off (a limit cycle at a stretched arm). The command uses
        // the damped μ/(μ² + δ²): any Λ̂ keeps it out of the levels above, since M⁻¹Ĵᵀ lies in their null space
        let tu = self.tuning;
        let fade = |mu: f64| keep_weight(mu / scale, tu.fade);
        let [eps, dmax] = if self.levels == 0 { tu.dls } else { tu.dls_lower };
        let (eps, dmax) = (eps * scale, dmax * scale);
        self.levels += 1;
        let (exact, cmd) = if rows == 1 {
            let x = a(0, 0);
            (
                [if x > 0.0 { fade(x) / x } else { 0.0 }, 0.0, 0.0, 0.0],
                [damped_inverse(x, eps, dmax), 0.0, 0.0, 0.0],
            )
        } else {
            let (x, y, z) = (a(0, 0), a(0, 1), a(1, 1));
            let (mean, half) = (f64::midpoint(x, z), (0.5 * (x - z)).hypot(y));
            let (mut exact, mut cmd) = ([0.0; 4], [0.0; 4]);
            for mu in [mean + half, mean - half] {
                // eigenvector of [[x, y], [y, z]] for μ: both (y, μ − x) and (μ − z, y) solve it; take the
                // better conditioned one (either may vanish), and a unit vector when the matrix is diagonal
                let (c1, c2) = ((y, mu - x), (mu - z, y));
                let (vx, vy) = if c1.0.hypot(c1.1) >= c2.0.hypot(c2.1) { c1 } else { c2 };
                let norm = vx.hypot(vy);
                let (vx, vy, norm) = if norm > 1e-300 * (1.0 + mu.abs()) {
                    (vx, vy, norm)
                } else if (x - mu).abs() <= (z - mu).abs() {
                    (1.0, 0.0, 1.0)
                } else {
                    (0.0, 1.0, 1.0)
                };
                let (vx, vy) = (vx / norm, vy / norm);
                let outer = [vx * vx, vx * vy, vx * vy, vy * vy];
                // a direction (or a whole task) that cannot move at all contributes nothing — not 0/0
                let (e, c) =
                    if mu > 0.0 { (fade(mu) / mu, damped_inverse(mu, eps, dmax)) } else { (0.0, 0.0) };
                for k in 0..4 {
                    exact[k] += outer[k] * e;
                    cmd[k] += outer[k] * c;
                }
            }
            (exact, cmd)
        };
        // the projector update with the exact inverse
        let mut pexact = [0.0; 2 * MAXN];
        for c in 0..n {
            for r in 0..rows {
                pexact[c * 2 + r] = (0..rows).map(|s| mj[c * 2 + s] * exact[s * 2 + r]).sum();
            }
        }
        for i in 0..n {
            for k in 0..n {
                self.p[i * n + k] -= (0..rows).map(|r| pexact[i * 2 + r] * jhat[r * n + k]).sum::<f64>();
            }
        }
        let lam = cmd;
        let mut pinv = [0.0; 2 * MAXN];
        for c in 0..n {
            for r in 0..rows {
                pinv[c * 2 + r] = (0..rows).map(|s| mj[c * 2 + s] * lam[s * 2 + r]).sum();
            }
        }
        Level { rows, jhat, lam, pinv }
    }

    /// Pᵀ τ: a torque projected so that it accelerates none of the levels pushed so far.
    fn project_torque(&self, tau: &VecN) -> VecN {
        let n = self.n;
        core::array::from_fn(|i| if i < n { (0..n).map(|k| self.p[k * n + i] * tau[k]).sum() } else { 0.0 })
    }

    /// P (pref − q̈): the part of a preferred acceleration that leaves every level pushed so far untouched.
    fn project_accel(&self, pref: &VecN, qdd: &VecN) -> VecN {
        let n = self.n;
        core::array::from_fn(|i| {
            if i < n { (0..n).map(|k| self.p[i * n + k] * (pref[k] - qdd[k])).sum() } else { 0.0 }
        })
    }

    /// ‖J M⁻¹ τ‖: the acceleration a torque causes in a task with Jacobian `j` (leakage).
    fn leak(&self, rows: usize, j: &[f64], tau: &VecN) -> f64 {
        let n = self.n;
        let a: VecN = core::array::from_fn(|i| {
            if i < n { (0..n).map(|k| self.minv[i * n + k] * tau[k]).sum() } else { 0.0 }
        });
        (0..rows).map(|r| (0..n).map(|c| j[r * n + c] * a[c]).sum::<f64>().powi(2)).sum::<f64>().sqrt()
    }
}

impl Level {
    /// v − Ĵ# (J v): what this level's (damped) command leaves of v.
    fn soft_complement(&self, n: usize, j: &[f64], v: &VecN) -> VecN {
        let jv: [f64; 2] =
            core::array::from_fn(
                |r| if r < self.rows { (0..n).map(|c| j[r * n + c] * v[c]).sum() } else { 0.0 },
            );
        core::array::from_fn(|i| {
            if i < n { v[i] - (0..self.rows).map(|r| self.pinv[i * 2 + r] * jv[r]).sum::<f64>() } else { 0.0 }
        })
    }

    /// Jᵀ Λ̂ J v (n): the part of M v this level's command accounts for (for passive soft damping).
    fn task_part(&self, n: usize, j: &[f64], v: &VecN) -> VecN {
        let jv: [f64; 2] =
            core::array::from_fn(
                |r| if r < self.rows { (0..n).map(|c| j[r * n + c] * v[c]).sum() } else { 0.0 },
            );
        let f: [f64; 2] = core::array::from_fn(|r| (0..self.rows).map(|s| self.lam[r * 2 + s] * jv[s]).sum());
        core::array::from_fn(|i| if i < n { (0..self.rows).map(|r| j[r * n + i] * f[r]).sum() } else { 0.0 })
    }

    /// Ĵᵀ Λ̂ a: the torque that gives this level the acceleration `a` without touching the levels above.
    fn torque(&self, n: usize, a: [f64; 2]) -> VecN {
        let f: [f64; 2] = core::array::from_fn(|r| (0..self.rows).map(|s| self.lam[r * 2 + s] * a[s]).sum());
        core::array::from_fn(|i| {
            if i < n { (0..self.rows).map(|r| self.jhat[r * n + i] * f[r]).sum() } else { 0.0 }
        })
    }

    /// q̈ + Ĵ# (b − J q̈): correct q̈ so that this level gets J q̈ = b, within the null space above it.
    fn resolve(&self, n: usize, j: &[f64], b: [f64; 2], qdd: &mut VecN) {
        let res: [f64; 2] = core::array::from_fn(|r| {
            if r < self.rows { b[r] - (0..n).map(|c| j[r * n + c] * qdd[c]).sum::<f64>() } else { 0.0 }
        });
        for (c, v) in qdd.iter_mut().enumerate().take(n) {
            *v += (0..self.rows).map(|r| self.pinv[c * 2 + r] * res[r]).sum::<f64>();
        }
    }
}

/// What a stacked controller reports about its secondary tasks.
#[derive(Clone, Copy, Debug, Default)]
struct Secondary {
    orient_err: f64,
    elbow_err: [f64; 2],
    /// Largest acceleration the lower levels cause in any secondary task above them.
    leak_sec: f64,
}

impl Secondary {
    fn record(&mut self, tasks: &[SubTask]) {
        *self = Self::default();
        for t in tasks {
            if t.rows == 1 {
                self.orient_err = t.e[0];
            } else {
                self.elbow_err = t.e;
            }
        }
    }

    fn put(&self, t: &mut Telemetry<'_>) {
        t.put("orient_err", &[self.orient_err]);
        t.put("elbow_err", &self.elbow_err);
        t.put("leak_sec", &[self.leak_sec]);
    }
}

/// Virtual model control / energy shaping (Takegaki–Arimoto): a virtual spring–damper pulls the tool to its
/// target and gravity is compensated,
/// `τ = Jᵀ(K (x_d − x) − D ẋ) + ∂V/∂q + N τ₀`.
/// With V_L = T + ½ eᵀ K e (kinetic energy plus spring), V̇_L = −ẋᵀ D ẋ − q̇ᵀ diag(β) q̇ ≤ 0.
#[derive(Debug, Default)]
pub struct Vmc {
    x: [f64; 2],
    xd: [f64; 2],
    force: [f64; 2],
    tau_task: VecN,
    tau_null: VecN,
    /// N (n × n), empty when not used.
    projector: Vec<f64>,
    leak: f64,
    w: f64,
    lyap: f64,
    lyap_prev: Option<f64>,
    lyap_dot: f64,
    sec: Secondary,
    tau_soft: VecN,
}

impl Controller for Vmc {
    fn reset(&mut self) {
        self.lyap_prev = None;
    }

    fn update(&mut self, plan: &Plan, model: &mut Model, c: &Ctx<'_>) -> VecN {
        let n = model.n;
        let Some(tool) = c.tool else { return [0.0; MAXN] };
        let (k, zeta, k_null) = (
            Kind::Vmc.param(c.params, "k"),
            Kind::Vmc.param(c.params, "zeta"),
            Kind::Vmc.param(c.params, "null"),
        );
        let d = 2.0 * zeta * (k * task_mass(n, c.terms, &tool.jp)).sqrt();
        let (xd, xd_dot, _) = plan.task_ref();
        self.x = tool.p;
        self.xd = xd;
        self.force = core::array::from_fn(|r| k * (xd[r] - tool.p[r]) - d * (tool.v[r] - xd_dot[r]));
        self.tau_task = core::array::from_fn(|i| {
            if i < n { (0..2).map(|r| tool.jp[r * n + i] * self.force[r]).sum() } else { 0.0 }
        });
        // the levels below the tool (open chains): orientation, elbow, then posture — each through the
        // null space of everything above it
        self.projector.clear();
        self.tau_null = [0.0; MAXN];
        self.tau_soft = [0.0; MAXN];
        self.sec = Secondary::default();
        self.leak = 0.0;
        if let Some(mut stack) = Stack::constrained(n, &c.terms.m, c.k, plan.config.tuning) {
            let tasks = plan.sub_tasks(model, c.q, c.qd);
            self.sec.record(&tasks);
            let primary = stack.push(2, &tool.jp);
            // passive damping of what the tool task does not see: −d (M − Jᵀ Λ̂ J) q̇ is symmetric positive
            // semi-definite (Λ̂ ≤ Λ), vanishes along the task and catches the elbow mode at a singular pose
            let qd: VecN = core::array::from_fn(|i| if i < n { c.qd[i] } else { 0.0 });
            let (mq, task) = (mat_vec(n, &c.terms.m, &qd), primary.task_part(n, &tool.jp, &qd));
            self.tau_soft = core::array::from_fn(|i| {
                if i < n { -plan.config.tuning.null_damping * (mq[i] - task[i]) } else { 0.0 }
            });
            let mut below: Vec<VecN> = Vec::with_capacity(tasks.len() + 1);
            for t in &tasks {
                let lv = stack.push(t.rows, &t.j);
                below.push(lv.torque(n, t.a));
            }
            let tau0 = plan.secondary(model, c.q, c.qd);
            below.push(stack.project_torque(&tau0.map(|v| k_null * v)));
            let sum = |taus: &[VecN]| -> VecN { core::array::from_fn(|i| taus.iter().map(|t| t[i]).sum()) };
            self.tau_null = sum(&below);
            self.leak = stack.leak(2, &tool.jp, &self.tau_null);
            for (k, t) in tasks.iter().enumerate() {
                self.sec.leak_sec = self.sec.leak_sec.max(stack.leak(t.rows, &t.j, &sum(&below[k + 1..])));
            }
            // Pᵀ of the whole stack: what is left for the posture goal
            self.projector.extend((0..n * n).map(|ik| stack.p[(ik % n) * n + ik / n]));
        }
        self.w = manipulability(n, &tool.jp);
        let dv = model.potential_grad(c.q);
        let f: VecN =
            core::array::from_fn(|i| self.tau_task[i] + dv[i] + self.tau_null[i] + self.tau_soft[i]);

        let mq = mat_vec(n, &c.terms.m, c.qd);
        let ke = 0.5 * (0..n).map(|i| c.qd[i] * mq[i]).sum::<f64>();
        let e2 = (xd[0] - tool.p[0]).powi(2) + (xd[1] - tool.p[1]).powi(2);
        self.lyap = ke + 0.5 * k * e2;
        self.lyap_dot = self.lyap_prev.map_or(0.0, |p| (self.lyap - p) / c.dt);
        self.lyap_prev = Some(self.lyap);
        inverse_dynamics(model, c.k, &f).0
    }

    fn telemetry(&self, t: &mut Telemetry<'_>) {
        t.put("x", &self.x);
        t.put("xd", &self.xd);
        t.put("force", &self.force);
        t.put("tau_task", &self.tau_task);
        t.put("tau_null", &self.tau_null);
        t.put("projector", &self.projector);
        t.put("leak", &[self.leak]);
        t.put("w", &[self.w]);
        t.put("lyap", &[self.lyap]);
        t.put("lyap_dot", &[self.lyap_dot]);
        self.sec.put(t);
    }
}

/// Accelerations that realise the tool acceleration `xdd` (rows J, right-hand side xdd − J̇q̇), closest in the
/// M-metric to the null-space preference `pref`. Returns (q̈, q̈ without the preference).
fn task_accel(n: usize, c: &Ctx<'_>, tool: &Tool, xdd: [f64; 2], pref: &VecN) -> (VecN, VecN) {
    let b = [xdd[0] - tool.jdqd[0], xdd[1] - tool.jdqd[1]];
    let with = resolve_accel(n, c.terms, c.k, (&tool.jp, &b), pref);
    let without = resolve_accel(n, c.terms, c.k, (&tool.jp, &b), &[0.0; MAXN]);
    (with, without)
}

/// The joint accelerations of the full stack in the constraint tangent space: the tool acceleration `xdd`, then the secondary
/// tasks, then the preference `pref` in what remains. Returns (q̈, q̈ of the tool level alone, report, the
/// tool's leakage). Constraint curvature supplies the affine base acceleration.
fn stacked_accel(
    plan: &Plan,
    model: &mut Model,
    c: &Ctx<'_>,
    tool: &Tool,
    xdd: [f64; 2],
    pref: &VecN,
) -> (VecN, VecN, Secondary, f64) {
    let n = model.n;
    let mut sec = Secondary::default();
    let stack = Stack::constrained(n, &c.terms.m, c.k, plan.config.tuning);
    let Some(mut stack) = stack else {
        let (with, without) = task_accel(n, c, tool, xdd, pref);
        let dq: VecN = core::array::from_fn(|i| with[i] - without[i]);
        let leak = (0..2)
            .map(|r| (0..n).map(|j| tool.jp[r * n + j] * dq[j]).sum::<f64>().powi(2))
            .sum::<f64>()
            .sqrt();
        return (with, without, sec, leak);
    };
    let tasks = plan.sub_tasks(model, c.q, c.qd);
    sec.record(&tasks);
    let velocity = mat_vec(n, &stack.p, c.qd);
    let mut qdd = c.k.map_or([0.0; MAXN], |k| k.consistent(&[0.0; MAXN]));
    let primary = stack.push(2, &tool.jp);
    primary.resolve(n, &tool.jp, [xdd[0] - tool.jdqd[0], xdd[1] - tool.jdqd[1]], &mut qdd);
    let task_only = qdd;
    // damping in what the tool task does not serve: its null space (the levels below override it along
    // their own directions), and near a singularity the directions its damped command lets go of
    let soft: VecN =
        core::array::from_fn(|i| if i < n { -plan.config.tuning.null_damping * velocity[i] } else { 0.0 });
    let damp = primary.soft_complement(n, &tool.jp, &soft);
    for i in 0..n {
        qdd[i] += damp[i];
    }
    let mut after = Vec::with_capacity(tasks.len());
    for t in &tasks {
        let b = [t.a[0] - t.jdqd[0], t.a[1] - t.jdqd[1]];
        stack.push(t.rows, &t.j).resolve(n, &t.j, b, &mut qdd);
        after.push(qdd);
    }
    // what remains of the null space is set to the preference — damped, so that it is never left free
    let pref: VecN = core::array::from_fn(|i| pref[i] + soft[i]);
    let fix = stack.project_accel(&pref, &qdd);
    for i in 0..n {
        qdd[i] += fix[i];
    }
    // joints beyond the tool are no part of any task: they take their preference as it is (no task sees
    // them, and the oblique projection would feed them what the chain cannot follow)
    for i in (0..n).filter(|&i| c.k.is_none() && model.beyond_tool[i]) {
        qdd[i] = pref[i];
    }
    let moved = |j: &[f64], rows: usize, from: &VecN| {
        (0..rows)
            .map(|r| (0..n).map(|c| j[r * n + c] * (qdd[c] - from[c])).sum::<f64>().powi(2))
            .sum::<f64>()
            .sqrt()
    };
    for (t, a) in tasks.iter().zip(&after) {
        sec.leak_sec = sec.leak_sec.max(moved(&t.j, t.rows, a));
    }
    let leak = moved(&tool.jp, 2, &task_only);
    (qdd, task_only, sec, leak)
}

/// Inputs for `B u − Aᵀλ = bias + commanded` with the commanded part scaled by the largest α ∈ [0, 1] that
/// keeps every |u| ≤ u_max. Under torque limits the arm then accelerates more gently along the same
/// direction; clipping each joint on its own would distort the motion and break the cancellation the
/// law relies on (an unreachable target far away would set the arm spinning).
fn feasible(model: &Model, c: &Ctx<'_>, bias: &VecN, commanded: &VecN) -> VecN {
    let (ub, _) = inverse_dynamics(model, c.k, bias);
    let (uc, _) = inverse_dynamics(model, c.k, commanded);
    let alpha = (0..model.nu)
        .map(|a| {
            if uc[a] > 1e-12 {
                (c.umax - ub[a]) / uc[a]
            } else if uc[a] < -1e-12 {
                (-c.umax - ub[a]) / uc[a]
            } else {
                1.0
            }
        })
        .fold(1.0_f64, f64::min)
        .clamp(0.0, 1.0);
    core::array::from_fn(|a| ub[a] + alpha * uc[a])
}

/// M⁻¹ τ₀ scaled: the null-space preference as an acceleration.
fn preference(plan: &Plan, model: &mut Model, c: &Ctx<'_>, gain: f64) -> VecN {
    let tau0 = plan.secondary(model, c.q, c.qd);
    let mut a = [0.0; MAXN];
    if !cholesky_solve(model.n, &c.terms.m, &tau0, &mut a) {
        return [0.0; MAXN];
    }
    // joints beyond the tool follow their own posture (see `Plan::secondary`)
    let own = plan.secondary_accel(model, c.q, c.qd);
    for i in (0..model.n).filter(|&i| model.beyond_tool[i]) {
        a[i] = own[i];
    }
    a.map(|v| v * gain)
}

/// Impedance control (Hogan): make the tool behave like a mass–spring–damper driven by the external force,
/// `Λ_d ë + D_d ė + K_d e = F_ext` with e = x − x_d. Resolve ẍ_cmd = ẍ_d + Λ_d⁻¹(F_ext − D_d ė − K_d e) into
/// joint accelerations and cancel the measured F_ext: `B u − Aᵀλ = M q̈ + C q̇ + N − Jᵀ F_ext`.
#[derive(Debug, Default)]
pub struct Impedance {
    x: [f64; 2],
    xd: [f64; 2],
    e: [f64; 2],
    fext: [f64; 2],
    xdd_cmd: [f64; 2],
    tau_null: VecN,
    leak: f64,
    w: f64,
    sec: Secondary,
}

impl Controller for Impedance {
    fn update(&mut self, plan: &Plan, model: &mut Model, c: &Ctx<'_>) -> VecN {
        let n = model.n;
        let Some(tool) = c.tool else { return [0.0; MAXN] };
        let (mass, k, zeta, k_null) = (
            Kind::Impedance.param(c.params, "mass"),
            Kind::Impedance.param(c.params, "k"),
            Kind::Impedance.param(c.params, "zeta"),
            Kind::Impedance.param(c.params, "null"),
        );
        let d = 2.0 * zeta * (k * mass).sqrt();
        let (xd, xd_dot, xd_ddot) = plan.task_ref();
        self.x = tool.p;
        self.xd = xd;
        self.fext = plan.reference.f_ext;
        self.e = [tool.p[0] - xd[0], tool.p[1] - xd[1]];
        self.xdd_cmd = core::array::from_fn(|r| {
            xd_ddot[r] + (plan.reference.f_ext[r] - d * (tool.v[r] - xd_dot[r]) - k * self.e[r]) / mass
        });
        let pref = preference(plan, model, c, k_null);
        let (qdd, qdd_task, sec, leak) = stacked_accel(plan, model, c, tool, self.xdd_cmd, &pref);
        self.sec = sec;
        let mq = mat_vec(n, &c.terms.m, &qdd);
        let h = c.terms.h(n, c.qd);
        // M q̈ + C q̇ + N − Jᵀ F_ext: the commanded part M q̈, and what holds the arm and cancels the push
        let bias: VecN = core::array::from_fn(|i| {
            if i < n {
                h[i] - (0..2).map(|r| tool.jp[r * n + i] * plan.reference.f_ext[r]).sum::<f64>()
            } else {
                0.0
            }
        });
        let dq: VecN = core::array::from_fn(|i| qdd[i] - qdd_task[i]);
        self.tau_null = mat_vec(n, &c.terms.m, &dq);
        self.leak = leak;
        self.w = manipulability(n, &tool.jp);
        feasible(model, c, &bias, &mq)
    }

    fn telemetry(&self, t: &mut Telemetry<'_>) {
        t.put("x", &self.x);
        t.put("xd", &self.xd);
        t.put("e", &self.e);
        t.put("fext", &self.fext);
        t.put("xdd_cmd", &self.xdd_cmd);
        t.put("tau_null", &self.tau_null);
        t.put("leak", &[self.leak]);
        t.put("w", &[self.w]);
        self.sec.put(t);
    }
}

/// Admittance control: integrate a virtual mass–spring–damper driven by the measured force,
/// `Λ_d ẍ_v + D_d (ẋ_v − ẋ_d) + K_d (x_v − x_d) = F_ext`, and track x_v with a stiff inner position loop.
#[derive(Debug, Default)]
pub struct Admittance {
    xv: Option<[f64; 2]>,
    xv_dot: [f64; 2],
    fext: [f64; 2],
    x: [f64; 2],
}

impl Controller for Admittance {
    fn reset(&mut self) {
        self.xv = None;
        self.xv_dot = [0.0; 2];
    }

    fn update(&mut self, plan: &Plan, model: &mut Model, c: &Ctx<'_>) -> VecN {
        let n = model.n;
        let Some(tool) = c.tool else { return [0.0; MAXN] };
        let (mass, k, zeta, wn) = (
            Kind::Admittance.param(c.params, "mass"),
            Kind::Admittance.param(c.params, "k"),
            Kind::Admittance.param(c.params, "zeta"),
            Kind::Admittance.param(c.params, "wn_in"),
        );
        let d = 2.0 * zeta * (k.max(1.0) * mass).sqrt();
        let (xd, xd_dot, _) = plan.task_ref();
        let mut xv = self.xv.unwrap_or(tool.p);
        let xv_ddot: [f64; 2] = core::array::from_fn(|r| {
            (plan.reference.f_ext[r] - d * (self.xv_dot[r] - xd_dot[r]) - k * (xv[r] - xd[r])) / mass
        });
        for r in 0..2 {
            self.xv_dot[r] += xv_ddot[r] * c.dt;
            xv[r] += self.xv_dot[r] * c.dt;
        }
        self.xv = Some(xv);
        self.fext = plan.reference.f_ext;
        self.x = tool.p;
        // inner loop: track the virtual trajectory
        let xdd: [f64; 2] = core::array::from_fn(|r| {
            xv_ddot[r] + wn * wn * (xv[r] - tool.p[r]) + 2.0 * wn * (self.xv_dot[r] - tool.v[r])
        });
        let pref = preference(plan, model, c, 1.0);
        let (qdd, ..) = stacked_accel(plan, model, c, tool, xdd, &pref);
        let mq = mat_vec(n, &c.terms.m, &qdd);
        let h = c.terms.h(n, c.qd);
        feasible(model, c, &h, &mq)
    }

    fn telemetry(&self, t: &mut Telemetry<'_>) {
        t.put("xv", &self.xv.unwrap_or(self.x));
        t.put("xv_dot", &self.xv_dot);
        t.put("fext", &self.fext);
        t.put("x", &self.x);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn lower_levels_never_disturb_higher_ones_and_feasible_levels_are_met() {
        check_hierarchy(false);
        check_hierarchy(true);
    }

    fn check_hierarchy(closed: bool) {
        let n = 5;
        // a symmetric positive definite M = AᵀA + I
        let a: Vec<f64> = (0..n * n).map(|k| ((k * 37 % 11) as f64 - 5.0) / 7.0).collect();
        let mut m = [0.0; MAXN * MAXN];
        for i in 0..n {
            for j in 0..n {
                m[i * n + j] =
                    (0..n).map(|k| a[k * n + i] * a[k * n + j]).sum::<f64>() + f64::from(u8::from(i == j));
            }
        }
        let j1: Vec<f64> = (0..2 * n).map(|k| ((k * 13 % 7) as f64 - 3.0) / 3.0).collect();
        let j2: Vec<f64> = (0..n).map(|k| ((k * 5 % 3) as f64) - 1.0 + 0.3).collect();
        let j3: Vec<f64> = (0..2 * n).map(|k| ((k * k * 17 % 7) as f64 - 3.0) / 2.0).collect();
        let (b1, b2, b3) = ([0.7, -1.1], [0.4, 0.0], [-0.3, 0.9]);
        let mut constraint = Constraints {
            n,
            m: 1,
            phi: [0.0; crate::mech::MAXM],
            a: [0.0; crate::mech::MAXM * MAXN],
            adq: [0.0; crate::mech::MAXM],
        };
        constraint.a[0] = 1.0;
        constraint.a[n - 1] = 0.3;
        constraint.adq[0] = 0.25;
        let k = closed.then_some(&constraint);
        let mut stack = Stack::constrained(n, &m, k, Tuning::default()).unwrap();
        let mut qdd = k.map_or([0.0; MAXN], |k| k.consistent(&[0.0; MAXN]));
        stack.push(2, &j1).resolve(n, &j1, b1, &mut qdd);
        let after1 = qdd;
        stack.push(1, &j2).resolve(n, &j2, b2, &mut qdd);
        let after2 = qdd;
        stack.push(2, &j3).resolve(n, &j3, b3, &mut qdd);
        let got = |j: &[f64], r: usize, q: &VecN| (0..n).map(|c| j[r * n + c] * q[c]).sum::<f64>();
        // exact: a lower level's correction leaves every level above untouched
        for r in 0..2 {
            assert!((got(&j1, r, &qdd) - got(&j1, r, &after1)).abs() < 1e-12, "level 1 disturbed");
        }
        assert!((got(&j2, 0, &qdd) - got(&j2, 0, &after2)).abs() < 1e-12, "level 2 disturbed");
        // the primary is met up to its small damping bias; a lower level is damped more (best-effort, more
        // so where the levels above have used up its directions) but always moves towards its goal
        for r in 0..2 {
            assert!((got(&j1, r, &qdd) - b1[r]).abs() < 1e-3 * (1.0 + b1[r].abs()), "primary missed");
        }
        let miss = |j: &[f64], b: [f64; 2], rows: usize, q: &VecN| {
            (0..rows).map(|r| (got(j, r, q) - b[r]).powi(2)).sum::<f64>().sqrt()
        };
        assert!(miss(&j2, b2, 1, &after2) < miss(&j2, b2, 1, &after1), "level 2 made no progress");
        assert!(miss(&j3, b3, 2, &qdd) < miss(&j3, b3, 2, &after2), "level 3 made no progress");
        if closed {
            assert!((constraint.apply(&qdd)[0] + constraint.adq[0]).abs() < 1e-10, "constraint disturbed");
        }
        // All available directions used: nothing left for posture.
        assert!(stack.p[..n * n].iter().all(|v| v.abs() < 1e-9));
    }
}
