//! Joint-space controllers: PD, PD + gravity compensation, computed torque and passivity-based tracking.
//! All of them work with loops: inputs come from the constrained inverse dynamics `B u − Aᵀλ = f`.

use super::{Controller, Ctx, Kind, Telemetry};
use crate::MAXN;
use crate::dynamics::{MatN, VecN, inverse_dynamics, mat_vec};
use crate::model::Model;
use crate::plan::{Plan, resolve_accel};

/// Tracking error e = q_d − q and ė = q̇_d − q̇ against the smooth joint reference.
fn errors(plan: &Plan, model: &Model, c: &Ctx<'_>) -> (VecN, VecN) {
    let n = model.n;
    let e = core::array::from_fn(|i| if i < n { Plan::error(model, i, c.q[i], plan.q_ref[i]) } else { 0.0 });
    let ed = core::array::from_fn(|i| if i < n { plan.qd_ref[i] - c.qd[i] } else { 0.0 });
    (e, ed)
}

#[derive(Debug)]
pub struct Passive;

impl Controller for Passive {
    fn update(&mut self, _: &Plan, _: &mut Model, _: &Ctx<'_>) -> VecN {
        [0.0; MAXN]
    }
}

/// u = K_p e + K_v ė (+ u_g) with constant symmetric gains shaped by the inertia at the target,
/// `K_p = ω² M(q*)`, `K_v = 2ζω M(q*)`: near q* every mode has the designed ω and ζ.
#[derive(Debug)]
pub struct Pd {
    gravity: bool,
    kp: MatN,
    kd: MatN,
    ug: VecN,
}

impl Pd {
    pub const fn new(gravity: bool) -> Self {
        Self { gravity, kp: [0.0; MAXN * MAXN], kd: [0.0; MAXN * MAXN], ug: [0.0; MAXN] }
    }
}

impl Controller for Pd {
    fn update(&mut self, plan: &Plan, model: &mut Model, c: &Ctx<'_>) -> VecN {
        let n = model.n;
        let (wn, zeta) = (Kind::Pd.param(c.params, "wn"), Kind::Pd.param(c.params, "zeta"));
        let (mut e, mut ed) = errors(plan, model, c);
        if c.k.is_none() {
            // an open chain: PD is a collocated servo on the actuated joints; feeding the passive joints'
            // errors through the coupling of M(q*) would turn it into a (poor) balancing controller
            for &i in &model.unactuated {
                (e[i], ed[i]) = (0.0, 0.0);
            }
        }
        for k in 0..n * n {
            self.kp[k] = wn * wn * plan.m_star[k];
            self.kd[k] = 2.0 * zeta * wn * plan.m_star[k];
        }
        let (pe, ve) = (mat_vec(n, &self.kp, &e), mat_vec(n, &self.kd, &ed));
        let f: VecN = core::array::from_fn(|i| pe[i] + ve[i]);
        // the inputs that best produce f (exactly the actuated rows for open chains)
        let (mut u, _) = inverse_dynamics(model, c.k, &f);
        if self.gravity {
            // exact gravity (and spring) compensation: B u_g − Aᵀλ = ∂V/∂q
            let dv = model.potential_grad(c.q);
            (self.ug, _) = inverse_dynamics(model, c.k, &dv);
            for a in 0..model.nu {
                u[a] += self.ug[a];
            }
        }
        u
    }

    fn telemetry(&self, t: &mut Telemetry<'_>) {
        // row-major n × n, packed
        t.put("kp", &self.kp);
        t.put("kd", &self.kd);
        if self.gravity {
            t.put("ug", &self.ug);
        }
    }
}

/// Computed torque: the actuated coordinates follow q̈_d + K_v ė + K_p e exactly (the rest follows from the
/// loops), then `B u − Aᵀλ = M q̈ + C q̇ + N`.
#[derive(Debug, Default)]
pub struct ComputedTorque {
    qdd: VecN,
}

impl Controller for ComputedTorque {
    fn update(&mut self, plan: &Plan, model: &mut Model, c: &Ctx<'_>) -> VecN {
        let n = model.n;
        let (wn, zeta) =
            (Kind::ComputedTorque.param(c.params, "wn"), Kind::ComputedTorque.param(c.params, "zeta"));
        let (kp, kv) = (wn * wn, 2.0 * zeta * wn);
        let (e, ed) = errors(plan, model, c);
        let v: VecN = core::array::from_fn(|i| plan.qdd_ref[i] + kv * ed[i] + kp * e[i]);
        let mut rows = [0.0; MAXN * MAXN];
        let mut b = [0.0; MAXN];
        for (a, &i) in model.actuated.iter().enumerate() {
            rows[a * n + i] = 1.0;
            b[a] = v[i];
        }
        self.qdd = resolve_accel(n, c.terms, c.k, (&rows, &b[..model.nu]), &v);
        let mq = mat_vec(n, &c.terms.m, &self.qdd);
        let h = c.terms.h(n, c.qd);
        let f: VecN = core::array::from_fn(|i| mq[i] + h[i]);
        inverse_dynamics(model, c.k, &f).0
    }

    fn telemetry(&self, t: &mut Telemetry<'_>) {
        t.put("qdd", &self.qdd);
    }
}

/// Slotine–Li passivity-based tracking: with the reference velocity q̇_r = q̇_d + Λe and the sliding variable
/// s = q̇ − q̇_r, apply `f = M q̈_r + C q̇_r + N − K_D s`. Uses the skew-symmetry of Ṁ − 2C instead of
/// cancelling the dynamics: V = ½ sᵀ M s + eᵀ Λ K_D e decreases.
#[derive(Debug, Default)]
pub struct Passivity {
    s: VecN,
    lyap: f64,
    lyap_prev: Option<f64>,
    lyap_dot: f64,
}

impl Controller for Passivity {
    fn update(&mut self, plan: &Plan, model: &mut Model, c: &Ctx<'_>) -> VecN {
        let n = model.n;
        let (lambda, kd) = (Kind::Passivity.param(c.params, "lambda"), Kind::Passivity.param(c.params, "kd"));
        let (e, ed) = errors(plan, model, c);
        let qd_r: VecN = core::array::from_fn(|i| plan.qd_ref[i] + lambda * e[i]);
        let qdd_r: VecN = core::array::from_fn(|i| plan.qdd_ref[i] + lambda * ed[i]);
        // K_D = k_D M(q*): symmetric positive definite, shaped like the inertia
        let kd_m: MatN = core::array::from_fn(|k| kd * plan.m_star[k]);
        self.s = core::array::from_fn(|i| if i < n { c.qd[i] - qd_r[i] } else { 0.0 });
        let mq = mat_vec(n, &c.terms.m, &qdd_r);
        let cq = mat_vec(n, &c.terms.c, &qd_r);
        let ks = mat_vec(n, &kd_m, &self.s);
        let f: VecN =
            core::array::from_fn(|i| if i < n { mq[i] + cq[i] + c.terms.n[i] - ks[i] } else { 0.0 });
        let ms = mat_vec(n, &c.terms.m, &self.s);
        let ke = mat_vec(n, &kd_m, &e);
        self.lyap = 0.5 * (0..n).map(|i| self.s[i] * ms[i]).sum::<f64>()
            + lambda * (0..n).map(|i| e[i] * ke[i]).sum::<f64>();
        self.lyap_dot = self.lyap_prev.map_or(0.0, |p| (self.lyap - p) / c.dt);
        self.lyap_prev = Some(self.lyap);
        inverse_dynamics(model, c.k, &f).0
    }

    fn telemetry(&self, t: &mut Telemetry<'_>) {
        t.put("s", &self.s);
        t.put("lyap", &[self.lyap]);
        t.put("lyap_dot", &[self.lyap_dot]);
    }
}
