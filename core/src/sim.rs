//! Fixed-step simulator: control (zero-order hold) + RK4 integration + snapshot publication.

use core::f64::consts::FRAC_PI_2;

use crate::control::arm::ArmCtrl;
use crate::control::cartpole::CartPoleCtrl;
use crate::control::{Controller, Ctx};
use crate::dynamics::{MatN, Terms, VecN, forward, mat_vec};
use crate::layout::{Flag, MAXF, off};
use crate::model::{Model, SystemId};
use crate::{MAXN, MAXP};

/// Rail end-stop stiffness (N/m) and damping (N s/m).
const STOP_K: f64 = 2.0e4;
const STOP_C: f64 = 300.0;

#[derive(Debug, Clone)]
pub struct Sim {
    pub model: &'static Model,
    pub p: [f64; MAXP],
    pub q: VecN,
    pub qd: VecN,
    /// Manual (user) input, added to the controller output in actuator space.
    pub u_manual: VecN,
    pub mode: u32,
    pub coriolis: bool,
    pub umax: f64,
    pub rail: f64,
    pub dt: f64,
    pub t: f64,
    pub steps: u64,
    pub ctrl: Controller,
    u: VecN,
    tau_u: VecN,
    tau_ext: VecN,
    dirty: bool,
    out: Box<[f64]>,
}

impl Sim {
    #[must_use]
    pub fn new(id: SystemId) -> Self {
        let model = id.model();
        let mut p = [0.0; MAXP];
        p[..model.defaults.len()].copy_from_slice(model.defaults);
        let ctrl = match id {
            SystemId::CartPole => Controller::CartPole(CartPoleCtrl::default()),
            SystemId::Arm2 => Controller::Arm(ArmCtrl::default()),
        };
        let mut sim = Self {
            model,
            p,
            q: [0.0; MAXN],
            qd: [0.0; MAXN],
            u_manual: [0.0; MAXN],
            mode: 0,
            coriolis: true,
            umax: match id {
                SystemId::CartPole => 40.0,
                SystemId::Arm2 => 80.0,
            },
            rail: 2.4,
            dt: 1e-3,
            t: 0.0,
            steps: 0,
            ctrl,
            u: [0.0; MAXN],
            tau_u: [0.0; MAXN],
            tau_ext: [0.0; MAXN],
            dirty: true,
            out: vec![0.0; off::LEN].into_boxed_slice(),
        };
        sim.reset();
        sim
    }

    pub fn reset(&mut self) {
        self.q = [0.0; MAXN];
        self.qd = [0.0; MAXN];
        match self.model.id {
            SystemId::CartPole => self.q[1] = 0.15,
            SystemId::Arm2 => {
                self.q[0] = -FRAC_PI_2 + 0.6;
                self.q[1] = 0.5;
            }
        }
        self.t = 0.0;
        self.steps = 0;
        self.sync_controller();
    }

    pub fn set_state(&mut self, i: usize, q: f64, qd: f64) {
        if i < self.model.nq {
            self.q[i] = q;
            self.qd[i] = qd;
            self.sync_controller();
        }
    }

    pub const fn set_param(&mut self, i: usize, v: f64) {
        if i < self.model.params.len() && v.is_finite() {
            self.p[i] = v;
            self.dirty = true;
        }
    }

    pub fn set_mode(&mut self, mode: u32) {
        self.mode = mode;
        self.sync_controller();
    }

    const fn sync_controller(&mut self) {
        if let Controller::Arm(c) = &mut self.ctrl {
            c.sync(&self.q);
        }
    }

    pub fn set_flag(&mut self, flag: Flag, v: f64) {
        match (flag, &mut self.ctrl) {
            (Flag::Coriolis, _) => self.coriolis = v != 0.0,
            (Flag::Umax, _) => self.umax = v.max(0.0),
            (Flag::Rail, _) => self.rail = v.max(0.1),
            (Flag::XRef, Controller::CartPole(c)) => c.x_ref = v,
            (Flag::LqrQx, Controller::CartPole(c)) => (c.q_x, self.dirty) = (v.max(1e-6), true),
            (Flag::LqrQth, Controller::CartPole(c)) => (c.q_th, self.dirty) = (v.max(1e-6), true),
            (Flag::LqrR, Controller::CartPole(c)) => (c.r, self.dirty) = (v.max(1e-6), true),
            (Flag::TargetX, Controller::Arm(c)) => c.target[0] = v,
            (Flag::TargetY, Controller::Arm(c)) => c.target[1] = v,
            (Flag::Elbow, Controller::Arm(c)) => c.elbow = if v < 0.0 { -1.0 } else { 1.0 },
            (Flag::RefWn, Controller::Arm(c)) => c.ref_wn = v.max(0.1),
            (Flag::CtWn, Controller::Arm(c)) => c.ct_wn = v.max(0.1),
            _ => {}
        }
    }

    /// Environment forces: stiff spring–damper end stops on the cart rail.
    fn external(&self, q: &[f64], qd: &[f64]) -> VecN {
        let mut f = [0.0; MAXN];
        if self.model.id == SystemId::CartPole {
            let pen = q[0].abs() - self.rail;
            if pen > 0.0 {
                let s = q[0].signum();
                // push back along −s, never pull the cart into the wall
                f[0] = -s * (STOP_K * pen + STOP_C * (s * qd[0])).max(0.0);
            }
        }
        f
    }

    fn accel(&self, q: &[f64], qd: &[f64]) -> VecN {
        let nq = self.model.nq;
        let t = Terms::eval(self.model, &self.p, q, qd);
        let ext = self.external(q, qd);
        let mut tau = [0.0; MAXN];
        for i in 0..nq {
            tau[i] = self.tau_u[i] + ext[i];
        }
        forward(nq, &t, qd, &tau, self.coriolis)
    }

    /// Advance one fixed step: evaluate the controller (zero-order hold) and integrate with RK4.
    pub fn step(&mut self) {
        let model = self.model;
        let (nq, nu) = (model.nq, model.nu);
        if self.dirty {
            if let Controller::CartPole(c) = &mut self.ctrl {
                c.design(model, &self.p, self.dt);
            }
            self.dirty = false;
        }

        let terms = Terms::eval(model, &self.p, &self.q, &self.qd);
        let ctx = Ctx { model, p: &self.p, q: &self.q, qd: &self.qd, terms: &terms, dt: self.dt };
        let uc = self.ctrl.update(self.mode, &ctx);
        for i in 0..nu {
            self.u[i] = (uc[i] + self.u_manual[i]).clamp(-self.umax, self.umax);
        }
        for i in 0..nq {
            self.tau_u[i] = (0..nu).map(|j| model.b[i * nu + j] * self.u[j]).sum();
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
        for i in 0..nq {
            self.q[i] = q0[i] + h / 6.0 * (k1q[i] + 2.0 * k2q[i] + 2.0 * k3q[i] + k4q[i]);
            self.qd[i] = v0[i] + h / 6.0 * (k1v[i] + 2.0 * k2v[i] + 2.0 * k3v[i] + k4v[i]);
        }
        if !self.q.iter().chain(&self.qd).all(|v| v.is_finite()) {
            // never hand NaNs to the renderer: fall back to the last good state, at rest
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

    #[must_use]
    pub fn energy(&self) -> [f64; 2] {
        (self.model.energy)(&self.p, &self.q, &self.qd)
    }

    #[must_use]
    pub fn out_ptr(&self) -> *const f64 {
        self.out.as_ptr()
    }

    /// Evaluate every displayed quantity at the current state into the snapshot buffer.
    #[allow(clippy::too_many_lines)]
    pub fn publish(&mut self) -> &[f64] {
        let model = self.model;
        let nq = model.nq;
        let (p, q, qd) = (&self.p[..], &self.q[..], &self.qd[..]);
        let t = Terms::eval(model, p, q, qd);
        let mut tau = [0.0; MAXN];
        for i in 0..nq {
            tau[i] = self.tau_u[i] + self.tau_ext[i];
        }
        let qdd = forward(nq, &t, qd, &tau, self.coriolis);
        let mut mdot: MatN = [0.0; MAXN * MAXN];
        (model.mdot)(p, q, qd, &mut mdot);
        let mut dv = [0.0; MAXN];
        (model.dvdq)(p, q, &mut dv);
        let [ke, pe] = (model.energy)(p, q, qd);

        // Lemma 4.2: S = Ṁ − 2C is skew-symmetric
        let mut skew: f64 = 0.0;
        for i in 0..nq {
            for j in 0..nq {
                let s_ij = mdot[i * nq + j] - 2.0 * t.c[i * nq + j];
                let s_ji = mdot[j * nq + i] - 2.0 * t.c[j * nq + i];
                skew = skew.max((s_ij + s_ji).abs());
            }
        }
        let (det, eig) = if nq == 2 {
            let (a, b, d) = (t.m[0], t.m[1], t.m[3]);
            #[allow(clippy::suspicious_operation_groupings)] // det of a symmetric 2×2: ad − b²
            let (tr, det) = (a + d, a * d - b * b);
            let disc = (tr * tr / 4.0 - det).max(0.0).sqrt();
            (det, [tr / 2.0 - disc, tr / 2.0 + disc])
        } else {
            (f64::NAN, [f64::NAN; 2])
        };
        let power: f64 = (0..nq).map(|i| qd[i] * tau[i]).sum();
        let dissipation: f64 = (0..nq).map(|i| p[model.damping[i]] * qd[i] * qd[i]).sum();

        let mut frames = [0.0; 3 * MAXF];
        (model.frames)(p, q, &mut frames);
        let mut coms = [0.0; 8];
        (model.coms)(p, q, &mut coms);
        let (mut tp, mut jp, mut js) = ([0.0; 2], [0.0; 2 * MAXN], [0.0; 3 * MAXN]);
        (model.tool)(p, q, &mut tp, &mut jp, &mut js);
        let mqdd = mat_vec(nq, &t.m, &qdd);
        let cqd = t.c_qd(nq, qd);

        let o = &mut self.out;
        let put = |o: &mut [f64], at: usize, v: &[f64]| o[at..at + v.len()].copy_from_slice(v);
        o[off::TIME] = self.t;
        o[off::STEPS] = self.steps as f64;
        o[off::SYSTEM] = f64::from(model.id as u32);
        o[off::NQ] = nq as f64;
        o[off::MODE] = f64::from(self.mode);
        put(o, off::Q, &self.q);
        put(o, off::QD, &self.qd);
        put(o, off::QDD, &qdd);
        put(o, off::U, &self.u);
        put(o, off::TAU, &tau);
        put(o, off::TAU_EXT, &self.tau_ext);
        put(o, off::M, &t.m);
        put(o, off::C, &t.c);
        put(o, off::N, &t.n);
        put(o, off::DV, &dv);
        put(o, off::MQDD, &mqdd);
        put(o, off::CQD, &cqd);
        put(o, off::MDOT, &mdot);
        put(o, off::ENERGY, &[ke, pe, ke + pe]);
        o[off::POWER] = power;
        o[off::DISSIPATION] = dissipation;
        o[off::SKEW] = skew;
        o[off::DET_M] = det;
        put(o, off::EIG_M, &eig);
        put(o, off::FRAMES, &frames);
        put(o, off::COMS, &coms);
        put(o, off::TOOL_P, &tp);
        put(o, off::JP, &jp);
        put(o, off::JS, &js);
        o[off::UMAX] = self.umax;
        o[off::RAIL] = self.rail;
        match &self.ctrl {
            Controller::CartPole(c) => {
                o[off::PHASE] = f64::from(c.phase);
                put(o, off::LIN_A, &core::array::from_fn::<f64, 16, _>(|k| c.a[(k / 4, k % 4)]));
                put(o, off::LIN_B, &[c.b[(0, 0)], c.b[(1, 0)], c.b[(2, 0)], c.b[(3, 0)]]);
                put(o, off::LQR_K, &[c.k[(0, 0)], c.k[(0, 1)], c.k[(0, 2)], c.k[(0, 3)]]);
                o[off::E_POLE] = c.e_pole;
                o[off::X_REF] = c.x_ref;
            }
            Controller::Arm(c) => {
                o[off::PHASE] = 0.0;
                put(o, off::TARGET, &c.target);
                o[off::REACHABLE] = if c.reachable { 1.0 } else { 0.0 };
                put(o, off::Q_REF, &c.q_ref);
                put(o, off::QD_REF, &c.qd_ref);
                put(o, off::Q_GOAL, &c.goal);
            }
        }
        &self.out
    }
}
