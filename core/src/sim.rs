//! Fixed-step simulator for any built mechanism: control (zero-order hold) + RK4 + snapshot publication.

use crate::MAXN;
use crate::control::{Ctrl, Ctx};
use crate::dynamics::{self, MatN, Terms, VecM, VecN, mat_vec, sym_eigenvalues};
use crate::layout::{Flag, MAXF, off};
use crate::mech::MAXM;
use crate::model::Model;

/// End-stop stiffness (N/m) and damping (N s/m) of prismatic joints with limits.
const STOP_K: f64 = 2.0e4;
const STOP_C: f64 = 300.0;

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
    lambda: VecM,
    dirty: bool,
    aux: Vec<f64>,
    out: Box<[f64]>,
}

impl Sim {
    #[must_use]
    pub fn new(model: Model) -> Self {
        let mut sim = Self {
            ctrl: Ctrl::new(&model),
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
            u: [0.0; MAXN],
            tau_u: [0.0; MAXN],
            tau_ext: [0.0; MAXN],
            lambda: [0.0; MAXM],
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
        self.t = 0.0;
        self.steps = 0;
        self.settle();
    }

    /// Re-project onto the constraint manifold and restart the controller reference.
    fn settle(&mut self) {
        dynamics::project(&mut self.model, &mut self.q, &mut self.qd);
        self.ctrl.sync(&self.q[..self.model.n]);
    }

    pub fn set_state(&mut self, i: usize, q: f64, qd: f64) {
        if i < self.n() {
            (self.q[i], self.qd[i]) = (q, qd);
            self.settle();
        }
    }

    pub fn kick(&mut self, i: usize, dqd: f64) {
        if i < self.n() {
            self.qd[i] += dqd;
            self.settle();
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
        let mut redesign = |field: &mut f64, v: f64| {
            *field = v.max(1e-6);
            self.dirty = true;
        };
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
            Flag::PdWn => redesign(&mut c.pd_wn, v),
            Flag::LqrQx => redesign(&mut c.lqr_qx, v),
            Flag::LqrQv => redesign(&mut c.lqr_qv, v),
            Flag::LqrR => redesign(&mut c.lqr_r, v),
            Flag::SwingGain => c.k_e = v.max(0.0),
        }
    }

    /// Environment forces: stiff spring–damper end stops on limited prismatic joints.
    fn external(&self, q: &[f64], qd: &[f64]) -> VecN {
        core::array::from_fn(|i| {
            let limit = self.model.limit.get(i).copied().unwrap_or(0.0);
            let pen = q[i].abs() - limit;
            if limit > 0.0 && pen > 0.0 {
                let s = q[i].signum();
                -s * (STOP_K * pen + STOP_C * (s * qd[i])).max(0.0)
            } else {
                0.0
            }
        })
    }

    fn accel(&mut self, q: &VecN, qd: &VecN) -> VecN {
        let ext = self.external(q, qd);
        let tau: VecN = core::array::from_fn(|i| self.tau_u[i] + ext[i]);
        let (qdd, lambda) = dynamics::accel(&mut self.model, q, qd, &tau, self.coriolis);
        self.lambda = lambda;
        qdd
    }

    fn redesign(&mut self) {
        if self.dirty {
            self.ctrl.design(&mut self.model, &self.p, self.dt);
            self.dirty = false;
        }
    }

    /// Advance one fixed step: evaluate the controller (zero-order hold) and integrate with RK4.
    pub fn step(&mut self) {
        let (n, nu) = (self.model.n, self.model.nu);
        self.redesign();
        let terms = Terms::eval(&mut self.model.dynamics, n, &self.q, &self.qd);
        let k = dynamics::constraints(&mut self.model, &self.q, &self.qd);
        let ctx =
            Ctx { p: &self.p, q: &self.q[..n], qd: &self.qd[..n], terms: &terms, k: k.as_ref(), dt: self.dt };
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
        let k1 = (v0, self.accel(&q0, &v0));
        let (q1, v1) = (lerp(&q0, &k1.0, h / 2.0), lerp(&v0, &k1.1, h / 2.0));
        let k2 = (v1, self.accel(&q1, &v1));
        let (q2, v2) = (lerp(&q0, &k2.0, h / 2.0), lerp(&v0, &k2.1, h / 2.0));
        let k3 = (v2, self.accel(&q2, &v2));
        let (q3, v3) = (lerp(&q0, &k3.0, h), lerp(&v0, &k3.1, h));
        let k4 = (v3, self.accel(&q3, &v3));
        for i in 0..n {
            self.q[i] = q0[i] + h / 6.0 * (k1.0[i] + 2.0 * k2.0[i] + 2.0 * k3.0[i] + k4.0[i]);
            self.qd[i] = v0[i] + h / 6.0 * (k1.1[i] + 2.0 * k2.1[i] + 2.0 * k3.1[i] + k4.1[i]);
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

    /// Evaluate every displayed quantity at the current state into the snapshot buffer. Consecutive layout
    /// fields that are written together are grouped (see `layout.rs`).
    pub fn publish(&mut self) -> &[f64] {
        self.redesign();
        let (n, nu, nb) = (self.model.n, self.model.nu, self.model.desc.nb());
        let (q, qd) = (self.q, self.qd);
        let t = Terms::eval(&mut self.model.dynamics, n, &q, &qd);
        let tau: VecN = core::array::from_fn(|i| self.tau_u[i] + self.tau_ext[i]);
        let qdd = self.accel(&q, &qd);
        let k = dynamics::constraints(&mut self.model, &q, &qd);
        let at_lambda = k.as_ref().map_or([0.0; MAXN], |k| k.apply_t(&self.lambda));
        self.model.aux.eval(&q, &qd, &mut self.aux);
        let (a, ao) = (&self.aux, self.model.aux_off);
        let mut mdot: MatN = [0.0; MAXN * MAXN];
        mdot[..n * n].copy_from_slice(&a[ao.mdot..ao.mdot + n * n]);

        // Lemma 4.2: S = Ṁ − 2C is skew-symmetric
        let s = |r: usize, c: usize| mdot[r * n + c] - 2.0 * t.c[r * n + c];
        let skew = (0..n)
            .flat_map(|i| (0..n).map(move |j| (i, j)))
            .fold(0.0, |m: f64, (i, j)| m.max((s(i, j) + s(j, i)).abs()));
        let eig = sym_eigenvalues(n, &t.m);
        let power: f64 = (0..n).map(|i| qd[i] * tau[i]).sum();
        let dissipation: f64 = (0..n).map(|i| self.p[self.model.beta_index[i]] * qd[i] * qd[i]).sum();
        let mqdd = mat_vec(n, &t.m, &qdd);
        let cqd = t.c_qd(n, &qd);
        let c = &self.ctrl;
        let (ke, pe) = (a[ao.energy], a[ao.energy + 1]);
        let flag = |b: bool| f64::from(u8::from(b));

        let o = &mut self.out;
        o.fill(0.0);
        let mut put = |at: usize, v: &[f64]| o[at..at + v.len()].copy_from_slice(v);
        put(
            off::TIME,
            &[self.t, self.steps as f64, n as f64, nu as f64, f64::from(self.mode), f64::from(c.phase)],
        );
        put(off::Q, &q[..n]);
        put(off::QD, &qd[..n]);
        put(off::QDD, &qdd[..n]);
        put(off::U, &self.u[..nu]);
        put(off::TAU, &tau[..n]);
        put(off::TAU_EXT, &self.tau_ext[..n]);
        put(off::M, &t.m[..n * n]);
        put(off::C, &t.c[..n * n]);
        put(off::N, &t.n[..n]);
        put(off::DV, &a[ao.dv..ao.dv + n]);
        put(off::MQDD, &mqdd[..n]);
        put(off::CQD, &cqd[..n]);
        put(off::MDOT, &mdot[..n * n]);
        put(off::ENERGY, &[ke, pe, ke + pe, power, dissipation, skew, eig[..n].iter().product()]);
        put(off::EIG_M, &eig[..n]);
        for (kf, ang) in self.model.frame_angles.iter().enumerate().take(MAXF) {
            put(off::FRAMES + 3 * kf, &[a[ao.frames + 2 * kf], a[ao.frames + 2 * kf + 1], ang.eval(&q)]);
        }
        put(off::COMS, &a[ao.coms..ao.coms + 2 * nb]);
        put(off::TOOL_P, &a[ao.tool_p..ao.tool_p + 2]);
        put(off::JP, &a[ao.jp..ao.jp + 2 * n]);
        put(off::JS, &a[ao.js..ao.js + 3 * n]);
        if let Some(l) = &c.lqr {
            put(off::LIN_A, &l.a.data);
            put(off::LIN_B, &l.b.data);
            put(off::LQR_K, &l.k.data);
            put(off::LQR_OK, &[1.0, l.z.cols as f64]);
        }
        put(off::U_FF, &c.u_ff[..nu]);
        put(off::EQ_RESIDUAL, &[c.eq_residual, c.e_u, c.e_ref]);
        put(off::TARGET, &[c.tool_target[0], c.tool_target[1], flag(c.ik), flag(c.reachable)]);
        put(off::Q_TARGET, &c.q_star[..n]);
        put(off::Q_REF, &c.q_ref[..n]);
        put(off::QD_REF, &c.qd_ref[..n]);
        put(off::KP, &c.kp[..n]);
        put(off::KD, &c.kd[..n]);
        put(off::UMAX, &[self.umax, c.ct_wn, flag(c.ct_ok)]);
        let violation = k.as_ref().map_or(0.0, dynamics::Constraints::max_violation);
        put(off::NC, &[self.model.nc as f64, self.model.ncon as f64, violation]);
        put(off::LAMBDA, &self.lambda[..self.model.ncon]);
        put(off::AT_LAMBDA, &at_lambda[..n]);
        &self.out
    }
}
