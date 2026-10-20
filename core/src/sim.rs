//! One simulation: a mechanism, its state, a plan and one controller. Fixed-step control (zero-order hold)
//! and RK4 integration; publishes the full snapshot (primary) or a compact slot block (any slot).

use crate::MAXN;
use crate::control::{Controller, Ctx, Kind, Telemetry};
use crate::dynamics::{self, MatN, Terms, VecM, VecN, mat_vec, sym_eigenvalues};
use crate::layout::{Flag, MAXF, off, slot};
use crate::mech::MAXM;
use crate::model::Model;
use crate::plan::{Plan, Tool, manipulability};

/// Largest force of the push on the tool (N): a hand, not a winch.
pub const PUSH_MAX: f64 = 30.0;

/// End-stop stiffness (N/m) and damping (N s/m) of prismatic joints with limits.
const STOP_K: f64 = 2.0e4;
const STOP_C: f64 = 300.0;

#[derive(Debug)]
pub struct Sim {
    pub model: Model,
    pub p: Vec<f64>,
    pub q: VecN,
    pub qd: VecN,
    /// Manual (user) input, added to the controller output in actuator space.
    pub u_manual: VecN,
    pub coriolis: bool,
    pub umax: f64,
    pub dt: f64,
    pub t: f64,
    pub steps: u64,
    pub plan: Plan,
    pub kind: Kind,
    ctrl: Box<dyn Controller>,
    u: VecN,
    tau_u: VecN,
    tau_ext: VecN,
    lambda: VecM,
    tool: Option<Tool>,
    plan_dirty: bool,
    ctrl_dirty: bool,
    aux: Vec<f64>,
}

impl Sim {
    #[must_use]
    pub fn new(model: Model, kind: Kind) -> Self {
        let mut sim = Self {
            plan: Plan::new(&model),
            p: model.defaults.clone(),
            model,
            q: [0.0; MAXN],
            qd: [0.0; MAXN],
            u_manual: [0.0; MAXN],
            coriolis: true,
            umax: 120.0,
            dt: 1e-3,
            t: 0.0,
            steps: 0,
            kind,
            ctrl: kind.create(),
            u: [0.0; MAXN],
            tau_u: [0.0; MAXN],
            tau_ext: [0.0; MAXN],
            lambda: [0.0; MAXM],
            tool: None,
            plan_dirty: true,
            ctrl_dirty: true,
            aux: Vec::new(),
        };
        sim.aux = vec![0.0; sim.model.aux_off.len];
        sim.reset();
        sim.refresh_plan();
        sim
    }

    /// An independent copy with the same model, settings and state, running controller `kind`.
    #[must_use]
    pub fn fork(&self, kind: Kind) -> Self {
        let mut s = Self::new(self.model.clone(), kind);
        s.p.clone_from(&self.p);
        s.model.set_params(&s.p);
        (s.q, s.qd, s.t, s.steps) = (self.q, self.qd, self.t, self.steps);
        (s.coriolis, s.umax, s.dt, s.u_manual) = (self.coriolis, self.umax, self.dt, self.u_manual);
        let plan = &self.plan;
        (s.plan.q_target, s.plan.tool_target, s.plan.use_tool) =
            (plan.q_target, plan.tool_target, plan.use_tool);
        (s.plan.ref_wn, s.plan.push, s.plan.push_k, s.plan.goals) =
            (plan.ref_wn, plan.push, plan.push_k, plan.goals);
        s.settle();
        s
    }

    #[must_use]
    pub fn n(&self) -> usize {
        self.model.n
    }

    pub fn set_controller(&mut self, kind: Kind) {
        self.kind = kind;
        self.ctrl = kind.create();
        self.ctrl_dirty = true;
        self.settle();
    }

    pub fn reset(&mut self) {
        self.q = [0.0; MAXN];
        self.qd = [0.0; MAXN];
        self.q[..self.model.n].copy_from_slice(&self.model.q0);
        self.t = 0.0;
        self.steps = 0;
        self.settle();
    }

    /// Re-project onto the constraint manifold and restart references and controller state.
    fn settle(&mut self) {
        dynamics::project(&mut self.model, &mut self.q, &mut self.qd);
        self.plan.sync(&mut self.model, &self.q);
        self.ctrl.reset();
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
            dynamics::project(&mut self.model, &mut self.q, &mut self.qd);
        }
    }

    pub fn set_param(&mut self, i: usize, v: f64) {
        if i < self.p.len() && v.is_finite() {
            self.p[i] = v;
            self.model.set_params(&self.p);
            self.plan_dirty = true;
        }
    }

    pub fn set_target(&mut self, i: usize, v: f64) {
        if i < self.n() && v.is_finite() {
            self.plan.q_target[i] = v;
            self.plan_dirty = true;
        }
    }

    /// The controller's design depends on parameters that changed.
    pub fn redesign_controller(&mut self) {
        self.ctrl_dirty = true;
    }

    pub fn set_flag(&mut self, flag: Flag, v: f64) {
        let pl = &mut self.plan;
        match flag {
            Flag::Coriolis => self.coriolis = v != 0.0,
            Flag::Umax => self.umax = v.max(0.0),
            Flag::TargetX => pl.tool_target[0] = v,
            Flag::TargetY => pl.tool_target[1] = v,
            Flag::UseTool => {
                pl.use_tool = v != 0.0;
                pl.ik_goal = pl.q_ref;
            }
            Flag::RefWn => pl.ref_wn = v.max(0.1),
            Flag::PushX => pl.push.get_or_insert([0.0; 2])[0] = v,
            Flag::PushY => pl.push.get_or_insert([0.0; 2])[1] = v,
            Flag::PushOn => {
                if v == 0.0 {
                    pl.push = None;
                }
            }
            Flag::PushK => pl.push_k = v.max(0.0),
            Flag::Posture => pl.goals.posture = v.max(0.0),
            Flag::Manipulability => pl.goals.manipulability = v.max(0.0),
        }
    }

    /// Environment forces: end stops of limited prismatic joints.
    fn stops(&self, q: &[f64], qd: &[f64]) -> VecN {
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

    /// External force of the push on the tool: a spring–damper from the tool to the pointer, at most
    /// [`PUSH_MAX`] newtons.
    fn push_force(&self, tool: &Tool) -> [f64; 2] {
        let Some(at) = self.plan.push else { return [0.0; 2] };
        let k = self.plan.push_k;
        let d = 2.0 * k.sqrt();
        let f: [f64; 2] = core::array::from_fn(|r| k * (at[r] - tool.p[r]) - d * tool.v[r]);
        let scale = (PUSH_MAX / f[0].hypot(f[1]).max(1e-12)).min(1.0);
        f.map(|v| v * scale)
    }

    fn accel(&mut self, q: &VecN, qd: &VecN) -> VecN {
        let stops = self.stops(q, qd);
        let tau: VecN = core::array::from_fn(|i| self.tau_u[i] + self.tau_ext[i] + stops[i]);
        let (qdd, lambda) = dynamics::accel(&mut self.model, q, qd, &tau, self.coriolis);
        self.lambda = lambda;
        qdd
    }

    /// Redesign the plan (equilibrium, linearisation, …) if parameters or targets changed.
    pub fn refresh_plan(&mut self) {
        if self.plan_dirty {
            self.plan.design(&mut self.model, &self.p);
            self.plan_dirty = false;
            self.ctrl_dirty = true;
        }
    }

    fn redesign(&mut self, params: &[f64]) {
        self.refresh_plan();
        if self.ctrl_dirty {
            self.ctrl.design(&self.plan, &mut self.model, params, self.dt);
            self.ctrl_dirty = false;
        }
    }

    /// Advance one fixed step with the controller's parameters `params`.
    pub fn step(&mut self, params: &[f64]) {
        let (n, nu) = (self.model.n, self.model.nu);
        self.redesign(params);
        self.plan.advance(&mut self.model, self.dt);
        let terms = Terms::eval(&mut self.model.dynamics, n, &self.q, &self.qd);
        let k = dynamics::constraints(&mut self.model, &self.q, &self.qd);
        self.tool = Tool::eval(&mut self.model, &self.q, &self.qd);
        // the push acts through the tool Jacobian: τ_ext = Jᵀ F_ext (held over the step)
        self.plan.f_ext = self.tool.as_ref().map_or([0.0; 2], |t| self.push_force(t));
        self.tau_ext = self.tool.as_ref().map_or([0.0; MAXN], |t| {
            core::array::from_fn(|i| {
                if i < n { (0..2).map(|r| t.jp[r * n + i] * self.plan.f_ext[r]).sum() } else { 0.0 }
            })
        });
        let ctx = Ctx {
            q: &self.q[..n],
            qd: &self.qd[..n],
            terms: &terms,
            k: k.as_ref(),
            tool: self.tool.as_ref(),
            params,
            dt: self.dt,
            umax: self.umax,
        };
        let uc = self.ctrl.update(&self.plan, &mut self.model, &ctx);
        for a in 0..nu {
            self.u[a] = (uc[a] + self.u_manual[a]).clamp(-self.umax, self.umax);
        }
        for i in 0..n {
            self.tau_u[i] = (0..nu).map(|a| self.model.b[i * nu + a] * self.u[a]).sum();
        }

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

    /// `[T, V]` at the current state.
    pub fn energy(&mut self) -> [f64; 2] {
        let mut e = [0.0; 2];
        self.model.energy.eval(&self.q, &self.qd, &mut e);
        e
    }

    /// Distances to the targets: ‖q − q*‖ (wrapped) and ‖p_tool − x_d‖.
    fn errors(&self) -> (f64, f64) {
        let n = self.n();
        let eq = (0..n)
            .map(|i| Plan::error(&self.model, i, self.q[i], self.plan.q_star[i]).powi(2))
            .sum::<f64>()
            .sqrt();
        let goal = if self.plan.use_tool { self.plan.tool_target } else { self.plan.x_star };
        let ex = self.tool.as_ref().map_or(0.0, |t| (t.p[0] - goal[0]).hypot(t.p[1] - goal[1]));
        (eq, ex)
    }

    /// The compact block of this simulation in its slot.
    pub fn publish_slot(&mut self, o: &mut [f64], params: &[f64]) {
        self.redesign(params);
        let n = self.n();
        let (eq, ex) = self.errors();
        let [ke, pe] = self.energy();
        let (q, qd) = (self.q, self.qd);
        self.model.aux.eval(&q, &qd, &mut self.aux);
        let (a, ao) = (&self.aux, self.model.aux_off);
        o.fill(0.0);
        let mut put = |at: usize, v: &[f64]| o[at..at + v.len()].copy_from_slice(v);
        let effort = (0..self.model.nu).map(|i| self.u[i] * self.u[i]).sum::<f64>().sqrt();
        put(slot::KIND, &[f64::from(self.kind as u32), f64::from(self.ctrl.phase()), eq, ex, effort]);
        put(slot::ENERGY, &[ke, pe, ke + pe]);
        put(slot::Q, &q[..n]);
        put(slot::U, &self.u[..self.model.nu]);
        for (kf, ang) in self.model.frame_angles.iter().enumerate().take(MAXF) {
            put(slot::FRAMES + 3 * kf, &[a[ao.frames + 2 * kf], a[ao.frames + 2 * kf + 1], ang.eval(&q)]);
        }
        put(slot::TOOL, &a[ao.tool_p..ao.tool_p + 2]);
        let mut tele = Telemetry::new(
            self.kind,
            &mut o[slot::TELEMETRY..slot::TELEMETRY + crate::control::TELEMETRY_LEN],
        );
        self.ctrl.telemetry(&mut tele);
    }

    /// The full snapshot of this simulation (the primary).
    pub fn publish(&mut self, o: &mut [f64], params: &[f64]) {
        self.redesign(params);
        let (n, nu, nb) = (self.model.n, self.model.nu, self.model.desc.nb());
        let (q, qd) = (self.q, self.qd);
        let t = Terms::eval(&mut self.model.dynamics, n, &q, &qd);
        let stops = self.stops(&q, &qd);
        let tau: VecN = core::array::from_fn(|i| self.tau_u[i] + self.tau_ext[i] + stops[i]);
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
        let (ke, pe) = (a[ao.energy], a[ao.energy + 1]);
        let pl = &self.plan;
        let flag = |b: bool| f64::from(u8::from(b));
        let w = manipulability(n, &a[ao.jp..ao.jp + 2 * n]);

        o[..off::SLOT].fill(0.0);
        let mut put = |at: usize, v: &[f64]| o[at..at + v.len()].copy_from_slice(v);
        let kind = f64::from(self.kind as u32);
        put(off::TIME, &[self.t, self.steps as f64, n as f64, nu as f64, kind, f64::from(self.ctrl.phase())]);
        put(off::Q, &q[..n]);
        put(off::QD, &qd[..n]);
        put(off::QDD, &qdd[..n]);
        put(off::U, &self.u[..nu]);
        put(off::TAU, &tau[..n]);
        put(off::TAU_EXT, &core::array::from_fn::<f64, MAXN, _>(|i| self.tau_ext[i] + stops[i])[..n]);
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
        if let Some(l) = &pl.lin {
            put(off::LIN_A, &l.a.data);
            put(off::LIN_B, &l.b.data);
        }
        put(off::U_FF, &pl.u_ff[..nu]);
        put(off::EQ_RESIDUAL, &[pl.eq_residual, pl.dof as f64, flag(pl.ct_ok)]);
        put(off::TARGET, &[pl.tool_target[0], pl.tool_target[1], flag(pl.use_tool), flag(pl.reachable)]);
        put(off::Q_TARGET, &pl.q_star[..n]);
        put(off::Q_REF, &pl.q_ref[..n]);
        put(off::QD_REF, &pl.qd_ref[..n]);
        put(off::X_REF, &[pl.x_ref[0], pl.x_ref[1], pl.f_ext[0], pl.f_ext[1], w, self.umax]);
        let violation = k.as_ref().map_or(0.0, dynamics::Constraints::max_violation);
        put(off::NC, &[self.model.nc as f64, self.model.ncon as f64, violation]);
        put(off::LAMBDA, &self.lambda[..self.model.ncon]);
        put(off::AT_LAMBDA, &at_lambda[..n]);
    }
}
