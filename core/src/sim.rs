//! Fixed-step simulator for any built mechanism: control (zero-order hold) + RK4 + snapshot publication.

use crate::MAXN;
use crate::control::{Ctrl, Ctx, LQR, SWING_UP};
use crate::dynamics::{
    Constraints, MatN, Terms, VecN, forward, forward_constrained, mat_vec, sym_eigenvalues,
};
use crate::layout::{Flag, MAXF, off};
use crate::mat::solve_dense;
use crate::model::Model;

/// End-stop stiffness (N/m) and damping (N s/m) of prismatic joints with limits.
const STOP_K: f64 = 2.0e4;
const STOP_C: f64 = 300.0;
/// Baumgarte stabilisation rate of the loop-closure constraints (1/s).
const BAUMGARTE: f64 = 20.0;

#[derive(Debug, Clone)]
pub struct Sim {
    pub model: Model,
    pub p: Vec<f64>,
    pub q: VecN,
    pub qd: VecN,
    /// Manual (user) input, added to the controller output in actuator space.
    pub u_manual: VecN,
    pub mode: u32,
    pub coriolis: bool,
    pub umax: f64,
    pub dt: f64,
    pub t: f64,
    pub steps: u64,
    pub ctrl: Ctrl,
    u: VecN,
    tau_u: VecN,
    tau_ext: VecN,
    lambda: [f64; 2 * crate::mech::MAXC],
    dirty: bool,
    aux: Vec<f64>,
    out: Box<[f64]>,
}

impl Sim {
    #[must_use]
    pub fn new(model: Model) -> Self {
        let ctrl = Ctrl::new(&model);
        let mut sim = Self {
            p: model.defaults.clone(),
            aux: vec![0.0; model.aux_off.len],
            model,
            q: [0.0; MAXN],
            qd: [0.0; MAXN],
            u_manual: [0.0; MAXN],
            mode: 0,
            coriolis: true,
            umax: 80.0,
            dt: 1e-3,
            t: 0.0,
            steps: 0,
            ctrl,
            u: [0.0; MAXN],
            tau_u: [0.0; MAXN],
            tau_ext: [0.0; MAXN],
            lambda: [0.0; 2 * crate::mech::MAXC],
            dirty: true,
            out: vec![0.0; off::LEN].into_boxed_slice(),
        };
        sim.reset();
        sim
    }

    #[must_use]
    pub fn n(&self) -> usize {
        self.model.n
    }

    pub fn reset(&mut self) {
        self.q = [0.0; MAXN];
        self.qd = [0.0; MAXN];
        self.q[..self.model.n].copy_from_slice(&self.model.q0);
        self.project();
        self.t = 0.0;
        self.steps = 0;
        self.ctrl.sync(&self.q[..self.n()]);
    }

    pub fn set_state(&mut self, i: usize, q: f64, qd: f64) {
        if i < self.n() {
            self.q[i] = q;
            self.qd[i] = qd;
            self.project();
            self.ctrl.sync(&self.q[..self.n()]);
        }
    }

    /// Move the state onto the constraint manifold: Gauss–Newton on φ(q) = 0 (minimal-norm steps),
    /// then remove the velocity component that violates A(q) q̇ = 0.
    pub fn project(&mut self) {
        let n = self.n();
        let m = 2 * self.model.nc;
        let Some(tape) = &mut self.model.constraints else { return };
        let zero = [0.0; MAXN];
        // y = (A Aᵀ + εI)⁻¹ r, then δ = Aᵀ y
        let min_norm = |k: &Constraints, r: &[f64]| -> VecN {
            let mut aat = [0.0; 4 * crate::mech::MAXC * crate::mech::MAXC];
            let mut y = [0.0; 2 * crate::mech::MAXC];
            for i in 0..m {
                for j in 0..m {
                    aat[i * m + j] = (0..n).map(|c| k.a[i * n + c] * k.a[j * n + c]).sum::<f64>()
                        + if i == j { 1e-12 } else { 0.0 };
                }
                y[i] = r[i];
            }
            let mut d = [0.0; MAXN];
            if solve_dense(m, &mut aat[..m * m], &mut y[..m]) {
                for c in 0..n {
                    d[c] = (0..m).map(|i| k.a[i * n + c] * y[i]).sum();
                }
            }
            d
        };
        for _ in 0..100 {
            let k = Constraints::eval(tape, n, m, &self.q, &zero);
            if k.max_violation() < 1e-12 {
                break;
            }
            let d = min_norm(&k, &k.phi[..m]);
            for c in 0..n {
                self.q[c] -= d[c].clamp(-0.5, 0.5);
            }
        }
        let k = Constraints::eval(tape, n, m, &self.q, &zero);
        let mut aqd = [0.0; 2 * crate::mech::MAXC];
        for i in 0..m {
            aqd[i] = (0..n).map(|c| k.a[i * n + c] * self.qd[c]).sum();
        }
        let d = min_norm(&k, &aqd[..m]);
        for c in 0..n {
            self.qd[c] -= d[c];
        }
    }

    pub fn kick(&mut self, i: usize, dqd: f64) {
        if i < self.n() {
            self.qd[i] += dqd;
            self.project();
        }
    }

    pub fn set_param(&mut self, i: usize, v: f64) {
        if i < self.p.len() && v.is_finite() {
            self.p[i] = v;
            self.model.set_params(&self.p);
            self.dirty = true;
        }
    }

    pub fn set_target(&mut self, i: usize, v: f64) {
        if i < self.n() && v.is_finite() {
            self.ctrl.q_target[i] = v;
            self.dirty = true;
        }
    }

    pub fn set_mode(&mut self, mode: u32) {
        self.mode = mode;
        self.ctrl.sync(&self.q[..self.n()]);
    }

    pub fn set_flag(&mut self, flag: Flag, v: f64) {
        let c = &mut self.ctrl;
        match flag {
            Flag::Coriolis => self.coriolis = v != 0.0,
            Flag::Umax => self.umax = v.max(0.0),
            Flag::TargetX => c.tool_target[0] = v,
            Flag::TargetY => c.tool_target[1] = v,
            Flag::Ik => {
                c.ik = v != 0.0;
                c.ik_goal = c.q_ref;
            }
            Flag::RefWn => c.ref_wn = v.max(0.1),
            Flag::CtWn => c.ct_wn = v.max(0.1),
            Flag::PdWn => (c.pd_wn, self.dirty) = (v.max(0.1), true),
            Flag::LqrQx => (c.lqr_qx, self.dirty) = (v.max(1e-6), true),
            Flag::LqrQv => (c.lqr_qv, self.dirty) = (v.max(1e-6), true),
            Flag::LqrR => (c.lqr_r, self.dirty) = (v.max(1e-6), true),
            Flag::SwingGain => c.k_e = v.max(0.0),
        }
    }

    /// Environment forces: stiff spring–damper end stops on limited prismatic joints.
    fn external(&self, q: &[f64], qd: &[f64]) -> VecN {
        let mut f = [0.0; MAXN];
        for (i, &limit) in self.model.limit.iter().enumerate() {
            if limit > 0.0 {
                let pen = q[i].abs() - limit;
                if pen > 0.0 {
                    let s = q[i].signum();
                    f[i] = -s * (STOP_K * pen + STOP_C * (s * qd[i])).max(0.0);
                }
            }
        }
        f
    }

    fn accel(&mut self, q: &VecN, qd: &VecN) -> VecN {
        let n = self.n();
        let t = Terms::eval(&mut self.model.dynamics, n, q, qd);
        let ext = self.external(q, qd);
        let tau: VecN = core::array::from_fn(|i| self.tau_u[i] + ext[i]);
        let m = 2 * self.model.nc;
        match &mut self.model.constraints {
            Some(tape) => {
                let k = Constraints::eval(tape, n, m, q, qd);
                let (qdd, lambda) = forward_constrained(n, &t, &k, qd, &tau, self.coriolis, BAUMGARTE);
                self.lambda = lambda;
                qdd
            }
            None => forward(n, &t, qd, &tau, self.coriolis),
        }
    }

    /// Advance one fixed step: evaluate the controller (zero-order hold) and integrate with RK4.
    pub fn step(&mut self) {
        let (n, nu) = (self.model.n, self.model.nu);
        if self.dirty {
            self.ctrl.design(&mut self.model, &self.p, self.dt);
            self.dirty = false;
        }
        let terms = Terms::eval(&mut self.model.dynamics, n, &self.q, &self.qd);
        let ctx = Ctx { p: &self.p, q: &self.q[..n], qd: &self.qd[..n], terms: &terms, dt: self.dt };
        let uc = self.ctrl.update(self.mode, &mut self.model, &ctx);
        for a in 0..nu {
            self.u[a] = (uc[a] + self.u_manual[a]).clamp(-self.umax, self.umax);
        }
        for i in 0..n {
            self.tau_u[i] = (0..nu).map(|a| self.model.b[i * nu + a] * self.u[a]).sum();
        }
        self.tau_ext = self.external(&self.q, &self.qd);

        let (q0, v0, h) = (self.q, self.qd, self.dt);
        let lerp = |base: &VecN, d: &VecN, s: f64| -> VecN { core::array::from_fn(|i| base[i] + s * d[i]) };
        let k1v = self.accel(&q0, &v0);
        let k1q = v0;
        let (q1, v1) = (lerp(&q0, &k1q, h / 2.0), lerp(&v0, &k1v, h / 2.0));
        let k2v = self.accel(&q1, &v1);
        let k2q = v1;
        let (q2, v2) = (lerp(&q0, &k2q, h / 2.0), lerp(&v0, &k2v, h / 2.0));
        let k3v = self.accel(&q2, &v2);
        let k3q = v2;
        let (q3, v3) = (lerp(&q0, &k3q, h), lerp(&v0, &k3v, h));
        let k4v = self.accel(&q3, &v3);
        let k4q = v3;
        for i in 0..n {
            self.q[i] = q0[i] + h / 6.0 * (k1q[i] + 2.0 * k2q[i] + 2.0 * k3q[i] + k4q[i]);
            self.qd[i] = v0[i] + h / 6.0 * (k1v[i] + 2.0 * k2v[i] + 2.0 * k3v[i] + k4v[i]);
        }
        if !self.q.iter().chain(&self.qd).all(|v| v.is_finite() && v.abs() < 1e5) {
            // a diverging controller or integration: fall back to the last good state, at rest
            self.q = q0;
            self.qd = [0.0; MAXN];
        }
        self.t += h;
        self.steps += 1;
    }

    pub fn step_n(&mut self, n: u32) {
        for _ in 0..n {
            self.step();
        }
    }

    /// `[T, V]` at the current state.
    pub fn energy(&mut self) -> [f64; 2] {
        let mut e = [0.0; 2];
        self.model.energy.eval(&self.q, &self.qd, &mut e);
        e
    }

    #[must_use]
    pub fn out_ptr(&self) -> *const f64 {
        self.out.as_ptr()
    }

    /// Evaluate every displayed quantity at the current state into the snapshot buffer.
    #[allow(clippy::too_many_lines)]
    pub fn publish(&mut self) -> &[f64] {
        if self.dirty {
            self.ctrl.design(&mut self.model, &self.p, self.dt);
            self.dirty = false;
        }
        let (n, nu) = (self.model.n, self.model.nu);
        let t = Terms::eval(&mut self.model.dynamics, n, &self.q, &self.qd);
        let tau: VecN = core::array::from_fn(|i| self.tau_u[i] + self.tau_ext[i]);
        let (q, qd) = (self.q, self.qd);
        let qdd = self.accel(&q, &qd);
        let m = 2 * self.model.nc;
        let (violation, at_lambda) = match &mut self.model.constraints {
            Some(tape) => {
                let k = Constraints::eval(tape, n, m, &self.q, &self.qd);
                let at: VecN = core::array::from_fn(|c| {
                    if c < n { (0..m).map(|r| k.a[r * n + c] * self.lambda[r]).sum() } else { 0.0 }
                });
                (k.max_violation(), at)
            }
            None => (0.0, [0.0; MAXN]),
        };
        self.model.aux.eval(&self.q, &self.qd, &mut self.aux);
        let a = &self.aux;
        let ao = self.model.aux_off;
        let mut mdot: MatN = [0.0; MAXN * MAXN];
        mdot[..n * n].copy_from_slice(&a[ao.mdot..ao.mdot + n * n]);

        // Lemma 4.2: S = Ṁ − 2C is skew-symmetric
        let mut skew: f64 = 0.0;
        for i in 0..n {
            for j in 0..n {
                let s_ij = mdot[i * n + j] - 2.0 * t.c[i * n + j];
                let s_ji = mdot[j * n + i] - 2.0 * t.c[j * n + i];
                skew = skew.max((s_ij + s_ji).abs());
            }
        }
        let eig = sym_eigenvalues(n, &t.m);
        let det: f64 = eig[..n].iter().product();
        let power: f64 = (0..n).map(|i| self.qd[i] * tau[i]).sum();
        let dissipation: f64 =
            (0..n).map(|i| self.p[self.model.beta_index[i]] * self.qd[i] * self.qd[i]).sum();
        let mqdd = mat_vec(n, &t.m, &qdd);
        let cqd = t.c_qd(n, &self.qd);

        let o = &mut self.out;
        o.fill(0.0);
        let put = |o: &mut [f64], at: usize, v: &[f64]| o[at..at + v.len()].copy_from_slice(v);
        o[off::TIME] = self.t;
        o[off::STEPS] = self.steps as f64;
        o[off::NQ] = n as f64;
        o[off::NU] = nu as f64;
        o[off::MODE] = f64::from(self.mode);
        o[off::PHASE] = f64::from(self.ctrl.phase);
        put(o, off::Q, &self.q[..n]);
        put(o, off::QD, &self.qd[..n]);
        put(o, off::QDD, &qdd[..n]);
        put(o, off::U, &self.u[..nu]);
        put(o, off::TAU, &tau[..n]);
        put(o, off::TAU_EXT, &self.tau_ext[..n]);
        put(o, off::M, &t.m[..n * n]);
        put(o, off::C, &t.c[..n * n]);
        put(o, off::N, &t.n[..n]);
        put(o, off::DV, &a[ao.dv..ao.dv + n]);
        put(o, off::MQDD, &mqdd[..n]);
        put(o, off::CQD, &cqd[..n]);
        put(o, off::MDOT, &mdot[..n * n]);
        let (ke, pe) = (a[ao.energy], a[ao.energy + 1]);
        put(o, off::ENERGY, &[ke, pe, ke + pe]);
        o[off::POWER] = power;
        o[off::DISSIPATION] = dissipation;
        o[off::SKEW] = skew;
        o[off::DET_M] = det;
        put(o, off::EIG_M, &eig[..n]);
        for (k, ang) in self.model.frame_angles.iter().enumerate().take(MAXF) {
            o[off::FRAMES + 3 * k] = a[ao.frames + 2 * k];
            o[off::FRAMES + 3 * k + 1] = a[ao.frames + 2 * k + 1];
            o[off::FRAMES + 3 * k + 2] = ang.eval(&self.q);
        }
        put(o, off::COMS, &a[ao.coms..ao.coms + 2 * self.model.desc.nb()]);
        put(o, off::TOOL_P, &a[ao.tool_p..ao.tool_p + 2]);
        put(o, off::JP, &a[ao.jp..ao.jp + 2 * n]);
        put(o, off::JS, &a[ao.js..ao.js + 3 * n]);

        let c = &self.ctrl;
        if let Some(l) = &c.lqr {
            put(o, off::LIN_A, &l.a.data);
            put(o, off::LIN_B, &l.b.data);
            put(o, off::LQR_K, &l.k.data);
            put(o, off::U_FF, &l.u_ff[..nu]);
            o[off::LQR_OK] = 1.0;
        }
        o[off::EQ_RESIDUAL] = c.eq_residual;
        o[off::E_U] = if matches!(self.mode, SWING_UP | LQR) || nu < n { c.e_u } else { 0.0 };
        o[off::E_REF] = c.e_ref;
        put(o, off::TARGET, &c.tool_target);
        o[off::IK] = f64::from(u8::from(c.ik));
        o[off::REACHABLE] = f64::from(u8::from(c.reachable));
        put(o, off::Q_TARGET, &c.q_target[..n]);
        put(o, off::Q_REF, &c.q_ref[..n]);
        put(o, off::QD_REF, &c.qd_ref[..n]);
        put(o, off::KP, &c.kp[..n]);
        put(o, off::KD, &c.kd[..n]);
        o[off::NC] = self.model.nc as f64;
        o[off::CONSTRAINT] = violation;
        put(o, off::LAMBDA, &self.lambda[..m]);
        put(o, off::AT_LAMBDA, &at_lambda[..n]);
        o[off::UMAX] = self.umax;
        o[off::CT_WN] = c.ct_wn;
        &self.out
    }
}
