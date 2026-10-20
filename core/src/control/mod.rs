//! Generic controllers for any built mechanism (MLS ch. 4 §5 and standard underactuated control).
//!
//! | mode | law | applies to |
//! |---|---|---|
//! | passive | u = 0 | all |
//! | PD | u = K_p e + K_v ė | actuated joints |
//! | PD + gravity | u = K_p e + K_v ė + ∂V/∂q | actuated joints |
//! | computed torque | τ = M(q̈_d + K_v ė + K_p e) + C q̇ + N | fully actuated |
//! | LQR | u = u* − K (x − x*) about q* | any, if q* is an equilibrium |
//! | swing-up | energy pumping via collocated partial feedback linearisation, LQR catch | underactuated |

use crate::MAXN;
use crate::dynamics::{Terms, VecN, mat_vec};
use crate::lqr::{c2d, dlqr};
use crate::mat::{DMat, cholesky_solve};
use crate::model::Model;

pub const PASSIVE: u32 = 0;
pub const PD: u32 = 1;
pub const PD_GRAVITY: u32 = 2;
pub const COMPUTED_TORQUE: u32 = 3;
pub const LQR: u32 = 4;
pub const SWING_UP: u32 = 5;

pub const PHASE_IDLE: u32 = 0;
pub const PHASE_SWING: u32 = 1;
pub const PHASE_BALANCE: u32 = 2;

/// Wrap an angle to (−π, π].
#[must_use]
pub fn wrap(a: f64) -> f64 {
    use core::f64::consts::{PI, TAU};
    let w = (a + PI).rem_euclid(TAU) - PI;
    if w <= -PI { w + TAU } else { w }
}

/// Everything a controller may look at during one control step.
#[derive(Debug)]
pub struct Ctx<'a> {
    pub p: &'a [f64],
    pub q: &'a [f64],
    pub qd: &'a [f64],
    pub terms: &'a Terms,
    pub dt: f64,
}

#[derive(Clone, Debug)]
pub struct LqrDesign {
    /// Linearisation ẋ = A x + B u, x = (q − q*, q̇).
    pub a: DMat,
    pub b: DMat,
    pub k: DMat,
    /// Feed-forward holding the equilibrium: B u* = ∂V/∂q(q*).
    pub u_ff: VecN,
}

#[derive(Clone, Debug)]
pub struct Ctrl {
    pub q_target: VecN,
    pub tool_target: [f64; 2],
    /// Track `tool_target` by inverse kinematics instead of `q_target`.
    pub ik: bool,
    pub ik_goal: VecN,
    pub reachable: bool,
    pub q_ref: VecN,
    pub qd_ref: VecN,
    pub qdd_ref: VecN,
    pub ref_wn: f64,
    pub ct_wn: f64,
    pub pd_wn: f64,
    pub kp: VecN,
    pub kd: VecN,
    pub lqr: Option<LqrDesign>,
    pub lqr_qx: f64,
    pub lqr_qv: f64,
    pub lqr_r: f64,
    /// Largest |∂V/∂qᵢ(q*)| over unactuated joints: q* is an equilibrium only if this is ~0.
    pub eq_residual: f64,
    pub phase: u32,
    /// Energy of the unactuated motion relative to the target, E_u = ½ q̇_uᵀ M_uu q̇_u + V(q) − V(q*).
    pub e_u: f64,
    pub e_ref: f64,
    pub v_target: f64,
    pub k_e: f64,
    pub k_c: f64,
    pub k_v: f64,
    pub a_max: f64,
    pub coupling: f64,
    initialised: bool,
}

impl Ctrl {
    #[must_use]
    pub fn new(model: &Model) -> Self {
        let mut q_target = [0.0; MAXN];
        q_target[..model.n].copy_from_slice(&model.q_target);
        Self {
            q_target,
            tool_target: [0.0; 2],
            ik: false,
            ik_goal: q_target,
            reachable: true,
            q_ref: [0.0; MAXN],
            qd_ref: [0.0; MAXN],
            qdd_ref: [0.0; MAXN],
            ref_wn: 4.0,
            ct_wn: 12.0,
            pd_wn: 6.0,
            kp: [0.0; MAXN],
            kd: [0.0; MAXN],
            lqr: None,
            lqr_qx: 10.0,
            lqr_qv: 1.0,
            lqr_r: 1.0,
            eq_residual: 0.0,
            phase: PHASE_IDLE,
            e_u: 0.0,
            e_ref: 1.0,
            v_target: 0.0,
            k_e: 12.0,
            k_c: 2.0,
            k_v: 2.5,
            a_max: 12.0,
            coupling: 1.0,
            initialised: false,
        }
    }

    /// Restart the reference at the current configuration (after reset or mode change).
    pub fn sync(&mut self, q: &[f64]) {
        let n = q.len().min(MAXN);
        self.q_ref[..n].copy_from_slice(&q[..n]);
        self.qd_ref = [0.0; MAXN];
        self.qdd_ref = [0.0; MAXN];
        self.ik_goal[..n].copy_from_slice(&q[..n]);
        self.initialised = true;
    }

    /// Recompute everything that depends on parameters or targets: PD gains, the LQR, energy scales.
    pub fn design(&mut self, model: &mut Model, p: &[f64], dt: f64) {
        let (n, nu) = (model.n, model.nu);
        let zero = [0.0; MAXN];
        let qs = &self.q_target[..n];
        let t = Terms::eval(&mut model.dynamics, n, qs, &zero);
        let mut aux = vec![0.0; model.aux_off.len];
        model.aux.eval(qs, &zero, &mut aux);
        let o = model.aux_off;
        let dv = &aux[o.dv..o.dv + n];
        let dndq = &aux[o.dndq..o.dndq + n * n];
        self.v_target = aux[o.energy + 1];

        for i in 0..n {
            let mii = t.m[i * n + i].max(1e-9);
            self.kp[i] = self.pd_wn * self.pd_wn * mii;
            self.kd[i] = 2.0 * self.pd_wn * mii;
        }

        // energy scale: twice the gravitational "height energy" of the unactuated bodies
        let vars = model.vars;
        self.e_ref = model
            .unactuated
            .iter()
            .map(|&j| model.coord_body[j])
            .map(|i| 2.0 * p[vars.m(i)] * p[vars.g()] * p[vars.r(i)].abs())
            .sum::<f64>()
            .max(1e-6);
        self.coupling = model
            .unactuated
            .iter()
            .flat_map(|&u| model.actuated.iter().map(move |&a| (u, a)))
            .map(|(u, a)| t.m[u * n + a].abs())
            .fold(0.0, f64::max)
            .max(1e-6);

        self.eq_residual = model.unactuated.iter().map(|&u| dv[u].abs()).fold(0.0, f64::max);
        self.lqr = None;
        if nu == 0 || model.closed() {
            return; // the LQR below linearises the tree dynamics; loops would need the constrained model
        }
        let Some(minv) = DMat::from_fn(n, n, |i, j| t.m[i * n + j]).inverse() else { return };
        let stiff = DMat::from_fn(n, n, |i, j| dndq[i * n + j]);
        let damp = DMat::from_fn(n, n, |i, j| if i == j { p[model.beta_index[i]] } else { 0.0 });
        let bsel = DMat::from_fn(n, nu, |i, j| model.b[i * nu + j]);
        let a21 = (&minv * &stiff).scale(-1.0);
        let a22 = (&minv * &damp).scale(-1.0);
        let b2 = &minv * &bsel;
        let a = DMat::from_fn(2 * n, 2 * n, |i, j| match (i < n, j < n) {
            (true, true) => 0.0,
            (true, false) => f64::from(u8::from(j - n == i)),
            (false, true) => a21[(i - n, j)],
            (false, false) => a22[(i - n, j - n)],
        });
        let b = DMat::from_fn(2 * n, nu, |i, j| if i < n { 0.0 } else { b2[(i - n, j)] });
        let q = DMat::from_fn(2 * n, 2 * n, |i, j| {
            if i != j {
                0.0
            } else if i < n {
                self.lqr_qx * dt
            } else {
                self.lqr_qv * dt
            }
        });
        let r = DMat::identity(nu).scale(self.lqr_r * dt);
        let (ad, bd) = c2d(&a, &b, dt);
        if let Some(k) = dlqr(&ad, &bd, &q, &r) {
            let mut u_ff = [0.0; MAXN];
            for (col, &i) in model.actuated.iter().enumerate() {
                u_ff[col] = dv[i];
            }
            self.lqr = Some(LqrDesign { a, b, k, u_ff });
        }
    }

    fn error(model: &Model, i: usize, q: f64, target: f64) -> f64 {
        if model.revolute[i] { wrap(target - q) } else { target - q }
    }

    fn lqr_law(&self, model: &Model, q: &[f64], qd: &[f64], u: &mut VecN) {
        let Some(l) = &self.lqr else { return };
        let n = model.n;
        let mut x = [0.0; 2 * MAXN];
        for i in 0..n {
            x[i] = -Self::error(model, i, q[i], self.q_target[i]);
            x[n + i] = qd[i];
        }
        for a in 0..model.nu {
            u[a] = l.u_ff[a] - (0..2 * n).map(|j| l.k[(a, j)] * x[j]).sum::<f64>();
        }
    }

    /// One damped-least-squares step of inverse kinematics on `ik_goal` towards `tool_target`.
    fn ik_step(&mut self, model: &mut Model) {
        let n = model.n;
        let Some(tool) = &mut model.tool else { return };
        let mut buf = [0.0; 2 + 2 * MAXN];
        let zero = [0.0; MAXN];
        for _ in 0..2 {
            tool.eval(&self.ik_goal[..n], &zero, &mut buf);
            let e = [self.tool_target[0] - buf[0], self.tool_target[1] - buf[1]];
            let jac = |r: usize, c: usize| buf[2 + r * n + c];
            // Δq = Jᵀ (J Jᵀ + λ² I)⁻¹ e
            let lambda2 = 0.01;
            let (mut a, mut b, mut d) = (lambda2, 0.0, lambda2);
            for c in 0..n {
                a += jac(0, c) * jac(0, c);
                b += jac(0, c) * jac(1, c);
                d += jac(1, c) * jac(1, c);
            }
            #[allow(clippy::suspicious_operation_groupings)] // det of the symmetric 2 × 2 J Jᵀ
            let det = a * d - b * b;
            let y = [(d * e[0] - b * e[1]) / det, (a * e[1] - b * e[0]) / det];
            for c in 0..n {
                let step = jac(0, c) * y[0] + jac(1, c) * y[1];
                self.ik_goal[c] += step.clamp(-0.05, 0.05);
            }
            self.reachable = e[0].hypot(e[1]) < 2e-3;
        }
    }

    fn advance_reference(&mut self, model: &mut Model, dt: f64) {
        let n = model.n;
        if self.ik && model.fully_actuated() {
            self.ik_step(model);
        }
        let wn = self.ref_wn;
        for i in 0..n {
            let goal = if self.ik && model.fully_actuated() { self.ik_goal[i] } else { self.q_target[i] };
            // the 2π-equivalent goal closest to the reference (no needless full turns)
            let goal = self.q_ref[i] + Self::error(model, i, self.q_ref[i], goal);
            self.qdd_ref[i] = wn * wn * (goal - self.q_ref[i]) - 2.0 * wn * self.qd_ref[i];
            self.qd_ref[i] += self.qdd_ref[i] * dt;
            self.q_ref[i] += self.qd_ref[i] * dt;
        }
    }

    /// Returns u (actuator space) for the given mode.
    #[allow(clippy::too_many_lines)]
    pub fn update(&mut self, mode: u32, model: &mut Model, ctx: &Ctx<'_>) -> VecN {
        let (n, nu) = (model.n, model.nu);
        let (q, qd, t) = (ctx.q, ctx.qd, ctx.terms);
        if !self.initialised {
            self.sync(q);
        }
        self.advance_reference(model, ctx.dt);

        // energy of the unactuated motion (also shown in the UI)
        let mut en = [0.0; 2];
        model.energy.eval(q, qd, &mut en);
        let ke_u: f64 = model
            .unactuated
            .iter()
            .flat_map(|&i| model.unactuated.iter().map(move |&j| (i, j)))
            .map(|(i, j)| 0.5 * t.m[i * n + j] * qd[i] * qd[j])
            .sum();
        self.e_u = ke_u + en[1] - self.v_target;

        let mut u = [0.0; MAXN];
        let mut e = [0.0; MAXN];
        let mut ed = [0.0; MAXN];
        for i in 0..n {
            e[i] = Self::error(model, i, q[i], self.q_ref[i]);
            ed[i] = self.qd_ref[i] - qd[i];
        }
        let mode = if model.closed() && matches!(mode, COMPUTED_TORQUE | SWING_UP) { PASSIVE } else { mode };
        match mode {
            PD | PD_GRAVITY => {
                let mut dv = [0.0; MAXN];
                if mode == PD_GRAVITY {
                    model.gravity.eval(q, qd, &mut dv);
                }
                for (a, &i) in model.actuated.iter().enumerate() {
                    u[a] = self.kp[i] * e[i] + self.kd[i] * ed[i] + dv[i];
                }
                self.phase = PHASE_IDLE;
            }
            COMPUTED_TORQUE if model.fully_actuated() => {
                let (kp, kv) = (self.ct_wn * self.ct_wn, 2.0 * self.ct_wn);
                let v: VecN = core::array::from_fn(|i| self.qdd_ref[i] + kv * ed[i] + kp * e[i]);
                let mv = mat_vec(n, &t.m, &v);
                let cqd = t.c_qd(n, qd);
                for i in 0..n {
                    u[i] = mv[i] + cqd[i] + t.n[i];
                }
                self.phase = PHASE_IDLE;
            }
            LQR => {
                self.lqr_law(model, q, qd, &mut u);
                self.phase = if self.lqr.is_some() { PHASE_BALANCE } else { PHASE_IDLE };
            }
            SWING_UP if nu > 0 && nu < n => {
                let near = model.unactuated.iter().all(|&i| {
                    Self::error(model, i, q[i], self.q_target[i]).abs() < 0.35 && qd[i].abs() < 4.0
                });
                let far = model
                    .unactuated
                    .iter()
                    .any(|&i| Self::error(model, i, q[i], self.q_target[i]).abs() > 0.7);
                if self.phase == PHASE_BALANCE && far || self.phase != PHASE_BALANCE && !near {
                    self.phase = PHASE_SWING;
                } else if near && self.lqr.is_some() {
                    self.phase = PHASE_BALANCE;
                }
                if self.phase == PHASE_BALANCE {
                    self.lqr_law(model, q, qd, &mut u);
                } else {
                    self.swing(model, q, qd, t, &mut u);
                }
            }
            _ => self.phase = PHASE_IDLE,
        }
        u
    }

    /// Energy pumping: choose the actuated acceleration v so the power the actuated motion feeds into the
    /// unactuated coordinates, −q̇_uᵀ M_ua v, drives E_u → 0; realise v by partial feedback linearisation.
    fn swing(&self, model: &Model, q: &[f64], qd: &[f64], t: &Terms, u: &mut VecN) {
        let n = model.n;
        let (act, un) = (&model.actuated, &model.unactuated);
        let at_rest = qd[..n].iter().all(|v| v.abs() < 1e-3);
        let mut v = [0.0; MAXN];
        for (a, &i) in act.iter().enumerate() {
            let w: f64 = -un.iter().map(|&k| t.m[k * n + i] * qd[k]).sum::<f64>();
            let pump = self.k_e * (-self.e_u / self.e_ref) * w / self.coupling;
            let centre = -self.k_c * -Self::error(model, i, q[i], self.q_target[i]) - self.k_v * qd[i];
            let mut va = pump + centre;
            if at_rest && self.e_u < -0.5 * self.e_ref {
                va += 0.5 * self.a_max; // break the symmetry when resting at the bottom
            }
            v[a] = va.clamp(-self.a_max, self.a_max);
        }
        // h = C q̇ + N
        let cqd = t.c_qd(n, qd);
        let h: VecN = core::array::from_fn(|i| if i < n { cqd[i] + t.n[i] } else { 0.0 });
        // M_uu q̈_u = −(h_u + M_ua v)
        let nuu = un.len();
        let mut muu = [0.0; MAXN * MAXN];
        let mut rhs = [0.0; MAXN];
        for (r, &i) in un.iter().enumerate() {
            for (c, &j) in un.iter().enumerate() {
                muu[r * nuu + c] = t.m[i * n + j];
            }
            rhs[r] = -h[i] - act.iter().enumerate().map(|(a, &j)| t.m[i * n + j] * v[a]).sum::<f64>();
        }
        let mut qdd_u = [0.0; MAXN];
        if !cholesky_solve(nuu, &muu, &rhs, &mut qdd_u) {
            return;
        }
        // τ_a = M_aa v + M_au q̈_u + h_a
        for (a, &i) in act.iter().enumerate() {
            u[a] = h[i]
                + act.iter().enumerate().map(|(b, &j)| t.m[i * n + j] * v[b]).sum::<f64>()
                + un.iter().enumerate().map(|(r, &j)| t.m[i * n + j] * qdd_u[r]).sum::<f64>();
        }
    }
}
