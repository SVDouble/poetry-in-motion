//! Task-space controllers acting on the tool point: virtual model control (energy shaping), impedance and
//! admittance. Secondary goals (posture, manipulability) act in the null space of the tool task.

use super::{Controller, Ctx, Telemetry};
use crate::MAXN;
use crate::dynamics::{Terms, VecN, inverse_dynamics, mat_vec};
use crate::mat::cholesky_solve;
use crate::model::Model;
use crate::plan::{Plan, Tool, manipulability, resolve_accel};

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

/// Dynamically consistent null-space projector N = I − Jᵀ J̄ᵀ with J̄ = M⁻¹ Jᵀ Λ: torques N τ₀ produce no
/// acceleration of the tool.
fn null_projector(n: usize, t: &Terms, jp: &[f64]) -> Option<MatN> {
    let (lam, minv_jt) = task_inertia(n, t, jp)?;
    // J̄ (n × 2) = M⁻¹ Jᵀ Λ
    let jbar = |c: usize, r: usize| minv_jt[c * 2] * lam[r] + minv_jt[c * 2 + 1] * lam[2 + r];
    let mut p = [0.0; MAXN * MAXN];
    for i in 0..n {
        for j in 0..n {
            p[i * n + j] =
                f64::from(u8::from(i == j)) - (0..2).map(|r| jp[r * n + i] * jbar(j, r)).sum::<f64>();
        }
    }
    Some(p)
}

/// Acceleration of the tool caused by a joint torque: ‖J M⁻¹ τ‖ (the task "leakage" of a null-space torque).
fn leakage(n: usize, t: &Terms, jp: &[f64], tau: &VecN) -> f64 {
    let mut a = [0.0; MAXN];
    if !cholesky_solve(n, &t.m, tau, &mut a) {
        return f64::NAN;
    }
    (0..2).map(|r| (0..n).map(|c| jp[r * n + c] * a[c]).sum::<f64>().powi(2)).sum::<f64>().sqrt()
}

/// Effective task mass for the damping of a virtual spring: mean eigenvalue of Λ.
fn task_mass(n: usize, t: &Terms, jp: &[f64]) -> f64 {
    task_inertia(n, t, jp).map_or(1.0, |(l, _)| f64::midpoint(l[0], l[3])).max(1e-6)
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
}

impl Controller for Vmc {
    fn reset(&mut self) {
        self.lyap_prev = None;
    }

    fn update(&mut self, plan: &Plan, model: &mut Model, c: &Ctx<'_>) -> VecN {
        let n = model.n;
        let Some(tool) = c.tool else { return [0.0; MAXN] };
        let (k, zeta, k_null) = (c.params[0], c.params[1], c.params[2]);
        let d = 2.0 * zeta * (k * task_mass(n, c.terms, &tool.jp)).sqrt();
        let (xd, xd_dot, _) = plan.task_ref();
        self.x = tool.p;
        self.xd = xd;
        self.force = core::array::from_fn(|r| k * (xd[r] - tool.p[r]) - d * (tool.v[r] - xd_dot[r]));
        self.tau_task = core::array::from_fn(|i| {
            if i < n { (0..2).map(|r| tool.jp[r * n + i] * self.force[r]).sum() } else { 0.0 }
        });
        // secondary goals, projected so they cannot move the tool (open chains)
        self.projector.clear();
        self.tau_null = [0.0; MAXN];
        if c.k.is_none()
            && let Some(p) = null_projector(n, c.terms, &tool.jp)
        {
            let tau0 = plan.secondary(model, c.q, c.qd);
            self.projector.extend_from_slice(&p[..n * n]);
            self.tau_null = core::array::from_fn(|i| {
                if i < n { k_null * (0..n).map(|j| p[i * n + j] * tau0[j]).sum::<f64>() } else { 0.0 }
            });
        }
        self.leak = leakage(n, c.terms, &tool.jp, &self.tau_null);
        self.w = manipulability(n, &tool.jp);
        let mut dv = [0.0; MAXN];
        model.gravity.eval(c.q, c.qd, &mut dv);
        let f: VecN = core::array::from_fn(|i| self.tau_task[i] + dv[i] + self.tau_null[i]);

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

/// M⁻¹ τ₀ scaled: the null-space preference as an acceleration.
fn preference(plan: &Plan, model: &mut Model, c: &Ctx<'_>, gain: f64) -> VecN {
    let tau0 = plan.secondary(model, c.q, c.qd);
    let mut a = [0.0; MAXN];
    if !cholesky_solve(model.n, &c.terms.m, &tau0, &mut a) {
        return [0.0; MAXN];
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
}

impl Controller for Impedance {
    fn update(&mut self, plan: &Plan, model: &mut Model, c: &Ctx<'_>) -> VecN {
        let n = model.n;
        let Some(tool) = c.tool else { return [0.0; MAXN] };
        let (mass, k, zeta, k_null) = (c.params[0], c.params[1], c.params[2], c.params[3]);
        let d = 2.0 * zeta * (k * mass).sqrt();
        let (xd, xd_dot, xd_ddot) = plan.task_ref();
        self.x = tool.p;
        self.xd = xd;
        self.fext = plan.f_ext;
        self.e = [tool.p[0] - xd[0], tool.p[1] - xd[1]];
        self.xdd_cmd = core::array::from_fn(|r| {
            xd_ddot[r] + (plan.f_ext[r] - d * (tool.v[r] - xd_dot[r]) - k * self.e[r]) / mass
        });
        let pref = preference(plan, model, c, k_null);
        let (qdd, qdd_task) = task_accel(n, c, tool, self.xdd_cmd, &pref);
        let mq = mat_vec(n, &c.terms.m, &qdd);
        let h = c.terms.h(n, c.qd);
        let f: VecN = core::array::from_fn(|i| {
            if i < n {
                mq[i] + h[i] - (0..2).map(|r| tool.jp[r * n + i] * plan.f_ext[r]).sum::<f64>()
            } else {
                0.0
            }
        });
        let dq: VecN = core::array::from_fn(|i| qdd[i] - qdd_task[i]);
        self.tau_null = mat_vec(n, &c.terms.m, &dq);
        self.leak = (0..2)
            .map(|r| (0..n).map(|j| tool.jp[r * n + j] * dq[j]).sum::<f64>().powi(2))
            .sum::<f64>()
            .sqrt();
        self.w = manipulability(n, &tool.jp);
        inverse_dynamics(model, c.k, &f).0
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
        let (mass, k, zeta, wn) = (c.params[0], c.params[1], c.params[2], c.params[3]);
        let d = 2.0 * zeta * (k.max(1.0) * mass).sqrt();
        let (xd, xd_dot, _) = plan.task_ref();
        let mut xv = self.xv.unwrap_or(tool.p);
        let xv_ddot: [f64; 2] = core::array::from_fn(|r| {
            (plan.f_ext[r] - d * (self.xv_dot[r] - xd_dot[r]) - k * (xv[r] - xd[r])) / mass
        });
        for r in 0..2 {
            self.xv_dot[r] += xv_ddot[r] * c.dt;
            xv[r] += self.xv_dot[r] * c.dt;
        }
        self.xv = Some(xv);
        self.fext = plan.f_ext;
        self.x = tool.p;
        // inner loop: track the virtual trajectory
        let xdd: [f64; 2] = core::array::from_fn(|r| {
            xv_ddot[r] + wn * wn * (xv[r] - tool.p[r]) + 2.0 * wn * (self.xv_dot[r] - tool.v[r])
        });
        let pref = preference(plan, model, c, 1.0);
        let (qdd, _) = task_accel(n, c, tool, xdd, &pref);
        let mq = mat_vec(n, &c.terms.m, &qdd);
        let h = c.terms.h(n, c.qd);
        let f: VecN = core::array::from_fn(|i| mq[i] + h[i]);
        inverse_dynamics(model, c.k, &f).0
    }

    fn telemetry(&self, t: &mut Telemetry<'_>) {
        t.put("xv", &self.xv.unwrap_or(self.x));
        t.put("xv_dot", &self.xv_dot);
        t.put("fext", &self.fext);
        t.put("x", &self.x);
    }
}
