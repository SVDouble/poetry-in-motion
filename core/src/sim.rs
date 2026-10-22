//! One simulation: a mechanism, its state, a plan and one controller. Fixed-step control (zero-order hold)
//! and RK4 integration; publishes the full snapshot (primary) or a compact slot block (any slot).

use crate::MAXN;
use crate::control::{Controller, Ctx, Kind, Telemetry};
use crate::dynamics::{self, MatN, Terms, VecM, VecN, mat_vec, sym_eigenvalues};
use crate::layout::{Flag, MAXF, off, slot};
use crate::mech::MAXM;
use crate::model::Model;
use crate::plan::{Plan, Tool, Tuning, manipulability};

/// Default actuator limit (N·m | N) and fixed step (s).
pub const UMAX: f64 = 120.0;
pub const DT: f64 = 1e-3;
/// RK4 substeps per control step while an end stop is (about to be) in contact.
const CONTACT_SUBSTEPS: u32 = 20;
/// A step that opens a loop further by more than this (m) is redone with `LOOP_SUBSTEPS` substeps.
const LOOP_TOLERANCE: f64 = 1e-7;
const LOOP_SUBSTEPS: u32 = 10;

/// Energy budget: the work done on the mechanism since the last restart.
///
/// Integrated with the RK4 weights of the motion itself, so that `E − E₀ = W_in + W_ext + W_stop − W_friction + W_constraint` holds to RK4 accuracy.
#[derive(Clone, Copy, Debug, Default)]
pub struct Budget {
    pub e0: f64,
    /// [inputs B u, push and kicks, end stops, friction loss (positive), constraint forces]
    pub work: [f64; 5],
    stages: [[f64; 5]; 4],
    stage: usize,
}

impl Budget {
    /// E − E₀ − (net work): zero for physically consistent dynamics.
    #[must_use]
    pub fn residual(&self, e: f64) -> f64 {
        let [w_in, w_ext, w_stop, w_fric, w_con] = self.work;
        e - self.e0 - (w_in + w_ext + w_stop - w_fric + w_con)
    }
}

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
    /// Simulated time of the last move of a regulating controller's set-point.
    retargeted: f64,
    aux: Vec<f64>,
    pub budget: Budget,
}

/// Configuration that survives mechanism replacement. Targets and body indices belong to a mechanism.
#[derive(Clone, Copy, Debug)]
pub struct Settings {
    coriolis: bool,
    umax: f64,
    dt: f64,
    ref_wn: f64,
    push_k: f64,
    weights: [f64; 4],
    tuning: Tuning,
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
            umax: UMAX,
            dt: DT,
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
            retargeted: f64::NEG_INFINITY,
            aux: Vec::new(),
            budget: Budget::default(),
        };
        sim.aux = vec![0.0; sim.model.aux_off.len];
        sim.reset();
        sim.refresh_plan();
        sim
    }

    #[must_use]
    pub fn settings(&self) -> Settings {
        let p = &self.plan;
        Settings {
            coriolis: self.coriolis,
            umax: self.umax,
            dt: self.dt,
            ref_wn: p.ref_wn,
            push_k: p.push_k,
            weights: [p.goals.posture, p.goals.manipulability, p.goals.orient, p.goals.elbow],
            tuning: p.tuning,
        }
    }

    pub fn apply_settings(&mut self, s: Settings) {
        (self.coriolis, self.umax, self.dt) = (s.coriolis, s.umax, s.dt);
        let p = &mut self.plan;
        (p.ref_wn, p.push_k, p.tuning) = (s.ref_wn, s.push_k, s.tuning);
        [p.goals.posture, p.goals.manipulability, p.goals.orient, p.goals.elbow] = s.weights;
        self.plan_dirty = true;
        self.ctrl_dirty = true;
    }

    pub fn set_dt(&mut self, dt: f64) -> bool {
        if !dt.is_finite() || dt <= 0.0 {
            return false;
        }
        self.dt = dt.clamp(1e-5, 1e-2);
        self.ctrl_dirty = true;
        true
    }

    /// An independent copy with the same model, settings and state, running controller `kind`.
    #[must_use]
    pub fn fork(&self, kind: Kind) -> Self {
        let mut s = Self::new(self.model.clone(), kind);
        s.apply_settings(self.settings());
        s.p.clone_from(&self.p);
        s.model.set_params(&s.p);
        (s.q, s.qd, s.t, s.steps) = (self.q, self.qd, self.t, self.steps);
        (s.coriolis, s.umax, s.dt, s.u_manual) = (self.coriolis, self.umax, self.dt, self.u_manual);
        let plan = &self.plan;
        (s.plan.q_target, s.plan.tool_target, s.plan.use_tool) =
            (plan.q_target, plan.tool_target, plan.use_tool);
        (s.plan.ref_wn, s.plan.push, s.plan.push_k, s.plan.goals) =
            (plan.ref_wn, plan.push, plan.push_k, plan.goals);
        // The requested target also has an assembly mode. Seed the new design from the primary's
        // equilibrium, not from its possibly distant current pose, then restart references at that pose.
        s.plan.q_star = plan.q_star;
        s.plan.sync(&mut s.model, &plan.q_star);
        s.refresh_plan();
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
        // a jump of the state: a regulator on its way somewhere starts afresh from here
        self.retargeted = f64::NEG_INFINITY;
        if self.plan.setpoint.take().is_some() {
            self.plan_dirty = true;
        }
        dynamics::project(&mut self.model, &mut self.q, &mut self.qd);
        self.plan.sync(&mut self.model, &self.q);
        self.ctrl.reset();
        self.restart_budget();
    }

    /// Start a new energy budget at the current state.
    fn restart_budget(&mut self) {
        let [t, v] = self.energy();
        self.budget = Budget { e0: t + v, ..Budget::default() };
    }

    fn projected_state(&mut self, mut q: VecN, mut qd: VecN) -> Option<(VecN, VecN)> {
        if !q.iter().chain(&qd).all(|v| v.is_finite()) {
            return None;
        }
        dynamics::project(&mut self.model, &mut q, &mut qd);
        if !q.iter().chain(&qd).all(|v| v.is_finite())
            || !self.model.regular(&q)
            || !self.model.energies(&q, &qd).iter().all(|v| v.is_finite())
        {
            return None;
        }
        Some((q, qd))
    }

    pub fn set_state(&mut self, i: usize, q: f64, qd: f64) -> bool {
        if i >= self.n() {
            return false;
        }
        let (mut qs, mut vs) = (self.q, self.qd);
        (qs[i], vs[i]) = (q, qd);
        let Some((qs, vs)) = self.projected_state(qs, vs) else { return false };
        (self.q, self.qd) = (qs, vs);
        self.settle();
        true
    }

    pub fn can_kick(&mut self, i: usize, dqd: f64) -> bool {
        if i >= self.n() {
            return false;
        }
        let mut qd = self.qd;
        qd[i] += dqd;
        self.projected_state(self.q, qd).is_some()
    }

    pub fn kick(&mut self, i: usize, dqd: f64) -> bool {
        if i >= self.n() {
            return false;
        }
        let mut qd = self.qd;
        qd[i] += dqd;
        let Some((q, qd)) = self.projected_state(self.q, qd) else { return false };
        let before = self.energy();
        (self.q, self.qd) = (q, qd);
        let after = self.energy();
        self.budget.work[1] += after[0] + after[1] - before[0] - before[1];
        true
    }

    /// Preflight without changing physical state or the energy budget. Used before a world-wide edit.
    pub fn can_set_param(&mut self, i: usize, v: f64) -> bool {
        if !self.model.valid_param(i, v) {
            return false;
        }
        let old = self.p[i];
        self.p[i] = v;
        self.model.set_params(&self.p);
        let valid = self.model.regular(&self.q)
            && self.model.energies(&self.q, &self.qd).iter().all(|v| v.is_finite());
        self.p[i] = old;
        self.model.set_params(&self.p);
        valid
    }

    pub fn set_param(&mut self, i: usize, v: f64) -> bool {
        if !self.can_set_param(i, v) {
            return false;
        }
        self.p[i] = v;
        self.model.set_params(&self.p);
        self.plan_dirty = true;
        self.restart_budget();
        true
    }

    pub fn set_target(&mut self, i: usize, v: f64) -> bool {
        if i < self.n() && v.is_finite() {
            self.plan.q_target[i] = v;
            self.plan_dirty = true;
            return true;
        }
        false
    }

    /// The controller's design depends on parameters that changed.
    pub fn redesign_controller(&mut self) {
        self.ctrl_dirty = true;
    }

    pub fn set_flag(&mut self, flag: Flag, v: f64) -> bool {
        if !v.is_finite() {
            return false;
        }
        if matches!(flag, Flag::Coriolis | Flag::UseTool | Flag::PushOn)
            && v.abs() > f64::EPSILON
            && (v - 1.0).abs() > f64::EPSILON
        {
            return false;
        }
        if matches!(
            flag,
            Flag::Umax | Flag::PushK | Flag::Posture | Flag::Manipulability | Flag::Orient | Flag::Elbow
        ) && v < 0.0
        {
            return false;
        }
        if matches!(flag, Flag::RefWn) && v <= 0.0 {
            return false;
        }
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
            Flag::Orient => pl.goals.orient = v.max(0.0),
            Flag::OrientTarget => pl.goals.orient_target = v,
            Flag::Elbow => pl.goals.elbow = v.max(0.0),
            Flag::ElbowX => pl.goals.elbow_target[0] = v,
            Flag::ElbowY => pl.goals.elbow_target[1] = v,
            Flag::ElbowBody => {
                if v < 0.0 || v.fract() != 0.0 || v >= self.model.desc.nb() as f64 {
                    return false;
                }
                pl.goals.elbow_body = v as usize;
            }
            _ => {
                // tuning: only finite, positive values
                if !(v.is_finite() && v > 0.0) {
                    return false;
                }
                let tu = &mut pl.tuning;
                match flag {
                    Flag::NullDamping => tu.null_damping = v,
                    Flag::DlsEps => tu.dls[0] = v,
                    Flag::DlsMax => tu.dls[1] = v,
                    Flag::DlsLowerEps => tu.dls_lower[0] = v,
                    Flag::DlsLowerMax => tu.dls_lower[1] = v,
                    Flag::FadeLo => tu.fade[0] = v,
                    Flag::FadeHi => tu.fade[1] = v,
                    Flag::ReachMargin => tu.reach_margin = v,
                    Flag::IkStep => tu.ik_step = v,
                    Flag::IkDamping => tu.ik_damping = v,
                    Flag::PushMax => tu.push_max = v,
                    Flag::StopK => tu.stop_k = v,
                    Flag::StopC => tu.stop_c = v,
                    Flag::StopDepth => tu.stop_depth = v,
                    Flag::Baumgarte => {
                        if (tu.baumgarte - v).abs() > 0.0 {
                            self.plan_dirty = true;
                        }
                        tu.baumgarte = v;
                    }
                    Flag::MotorSpeed => tu.motor_speed = v,
                    Flag::Lead => tu.lead = v,
                    _ => {}
                }
            }
        }
        true
    }

    /// Environment forces: end stops of limited prismatic joints, a Hunt–Crossley contact `F = δ (K + D δ̇)`
    /// with penetration δ. The force grows continuously from zero (a damper that switched on at touch-down
    /// would make the force jump and cost RK4 its order); `D = C / δ₀` gives the damping `C` at the typical
    /// penetration δ₀.
    fn stops(&self, q: &[f64], qd: &[f64]) -> VecN {
        let tu = &self.plan.tuning;
        let (k, d) = (tu.stop_k, tu.stop_c / tu.stop_depth.max(1e-6));
        core::array::from_fn(|i| {
            let limit = self.model.limit.get(i).copied().unwrap_or(0.0);
            let pen = q[i].abs() - limit;
            if limit > 0.0 && pen > 0.0 {
                let s = q[i].signum();
                -s * (pen * d.mul_add(s * qd[i], k)).max(0.0)
            } else {
                0.0
            }
        })
    }

    /// External force of the push on the tool: a spring–damper from the tool to the pointer, at most
    /// `push_max` newtons.
    fn push_force(&self, tool: &Tool) -> [f64; 2] {
        let Some(at) = self.plan.push else { return [0.0; 2] };
        let k = self.plan.push_k;
        let d = 2.0 * k.sqrt();
        let f: [f64; 2] = core::array::from_fn(|r| k * (at[r] - tool.p[r]) - d * tool.v[r]);
        let scale = (self.plan.tuning.push_max / f[0].hypot(f[1]).max(1e-12)).min(1.0);
        f.map(|v| v * scale)
    }

    /// The end-stop forces and the total generalised force on the joints: inputs, pushes and stops.
    fn applied(&self, q: &[f64], qd: &[f64]) -> (VecN, VecN) {
        let stops = self.stops(q, qd);
        (stops, core::array::from_fn(|i| self.tau_u[i] + self.tau_ext[i] + stops[i]))
    }

    /// Pose (x, y, φ) of every frame (s, the bodies, the tool) from an evaluated aux tape.
    fn frame_poses<'a>(&'a self, q: &'a VecN) -> impl Iterator<Item = [f64; 3]> + 'a {
        let (a, ao) = (&self.aux, self.model.aux_off);
        self.model
            .frame_angles
            .iter()
            .take(MAXF)
            .enumerate()
            .map(move |(kf, ang)| [a[ao.frames + 2 * kf], a[ao.frames + 2 * kf + 1], ang.eval(q)])
    }

    /// Power lost to viscous joint friction, Σ βᵢ q̇ᵢ².
    fn joint_friction(&self, qd: &[f64]) -> f64 {
        (0..self.model.n).map(|i| self.p[self.model.beta_index[i]] * qd[i] * qd[i]).sum()
    }

    fn accel(&mut self, q: &VecN, qd: &VecN) -> VecN {
        let t = self.model.terms(q, qd);
        let k = dynamics::constraints(&mut self.model, q, qd);
        self.accel_with(&t, k.as_ref(), q, qd)
    }

    /// Accelerations at a state whose M, C, N and constraints are already evaluated; records the power of
    /// every force for the energy budget (the constraint forces act as −Aᵀλ).
    fn accel_with(&mut self, t: &Terms, k: Option<&dynamics::Constraints>, q: &VecN, qd: &VecN) -> VecN {
        let n = self.model.n;
        let (stops, tau) = self.applied(q, qd);
        let (qdd, lambda) =
            dynamics::accel_with(n, t, k, qd, &tau, self.coriolis, self.plan.tuning.baumgarte);
        self.lambda = lambda;
        let dot = |f: &VecN| (0..n).map(|i| qd[i] * f[i]).sum::<f64>();
        // joint friction and the dampers
        let friction = self.joint_friction(qd) + self.model.element_forces(q, qd).dissipation;
        let constraint = k.map_or(0.0, |k| -dot(&k.apply_t(&lambda)));
        let b = &mut self.budget;
        b.stages[b.stage % 4] = [dot(&self.tau_u), dot(&self.tau_ext), dot(&stops), friction, constraint];
        b.stage += 1;
        qdd
    }

    /// A controller that regulates around an equilibrium (LQR, MPC) is made for small deviations from it. On
    /// a fully actuated mechanism every pose is one it can hold, so on the way to its target (or to the
    /// inverse kinematics of a tool target) it is designed at a pose at most `lead` ahead of the state — a
    /// reference governor: it only ever corrects small deviations, and the mechanism walks to the target.
    /// Redesigned at most every `RETARGET` seconds. (Underactuated, only q* itself can be held.)
    fn retarget(&mut self) {
        const RETARGET: f64 = 0.05;
        const MOVED: f64 = 1e-6;
        if !(self.ctrl.regulates() && self.plan.ct_ok) || self.t - self.retargeted < RETARGET {
            return;
        }
        let n = self.model.n;
        let pl = &self.plan;
        let goal = if pl.follows_ik(&self.model) { pl.ik_goal } else { pl.q_star };
        let dist = |a: &VecN, b: &VecN| {
            (0..n).map(|i| Plan::error(&self.model, i, a[i], b[i]).abs()).fold(0.0, f64::max)
        };
        let lead = pl.tuning.lead;
        match pl.setpoint {
            // arrived: designed at the target itself, it stays there (a disturbance is the regulator's to
            // correct) until the target moves
            Some(at) if dist(&at, &goal) <= MOVED => return,
            // designed at the target from the start (a reset): needed only if the state is far from it
            None if dist(&self.q, &goal) <= lead => return,
            _ => {}
        }
        let to: VecN = core::array::from_fn(|i| {
            if i < n {
                self.q[i] + Plan::error(&self.model, i, self.q[i], goal[i]).clamp(-lead, lead)
            } else {
                0.0
            }
        });
        if dist(&pl.q_lin, &to) > MOVED {
            self.plan.setpoint = Some(to);
            self.plan_dirty = true;
            self.retargeted = self.t;
        }
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
        self.retarget();
        let terms = self.model.terms(&self.q, &self.qd);
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
        // the actuator limit, and the motor's torque–speed line: driving along the motion it delivers less
        // the faster it turns, braking it delivers in full
        let fade = |v: f64| self.umax * (1.0 - v.abs() / self.plan.tuning.motor_speed).max(0.0);
        for a in 0..nu {
            let v: f64 = (0..n).map(|i| self.model.b[i * nu + a] * self.qd[i]).sum();
            let (lo, hi) = if v >= 0.0 { (-self.umax, fade(v)) } else { (-fade(v), self.umax) };
            self.u[a] = (uc[a] + self.u_manual[a]).clamp(lo, hi);
        }
        for i in 0..n {
            self.tau_u[i] = (0..nu).map(|a| self.model.b[i * nu + a] * self.u[a]).sum();
        }

        let (q0, h) = (self.q, self.dt);
        // an end-stop contact switches on mid-step and costs RK4 its accuracy: resolve it with substeps
        let near = |q: &VecN, qd: &VecN| {
            (0..n).any(|i| {
                let limit = self.model.limit.get(i).copied().unwrap_or(0.0);
                limit > 0.0 && (q[i] + h * qd[i]).abs().max(q[i].abs()) > limit
            })
        };
        let sub = if near(&self.q, &self.qd) { CONTACT_SUBSTEPS } else { 1 };
        let (qd0, budget0) = (self.qd, self.budget);
        // the first stage is at the state the controller just evaluated: reuse M, C, N and A
        for s in 0..sub {
            self.rk4(h / f64::from(sub), (s == 0).then_some((&terms, k.as_ref())));
        }
        // near a singular loop configuration the constraint forces get large and 1 ms is too coarse: when
        // the step itself opened the loop measurably (beyond what Baumgarte is still closing), redo it
        if sub == 1 && self.model.closed() {
            let before = k.as_ref().map_or(0.0, dynamics::Constraints::max_violation);
            let after =
                dynamics::constraints(&mut self.model, &self.q, &self.qd).map_or(0.0, |k| k.max_violation());
            let grew = after - before;
            if grew.is_nan() || grew > LOOP_TOLERANCE {
                (self.q, self.qd, self.budget) = (q0, qd0, budget0);
                for s in 0..LOOP_SUBSTEPS {
                    self.rk4(h / f64::from(LOOP_SUBSTEPS), (s == 0).then_some((&terms, k.as_ref())));
                }
            }
        }
        if !self.q.iter().chain(&self.qd).all(|v| v.is_finite() && v.abs() < 1e5) {
            // a diverging controller or integration: fall back to the last good state, at rest
            self.q = q0;
            self.qd = [0.0; MAXN];
            self.restart_budget();
        }
        self.t += h;
        self.steps += 1;
    }

    /// One RK4 step of length `h` with the inputs held, accumulating the energy budget.
    fn rk4(&mut self, h: f64, at_start: Option<(&Terms, Option<&dynamics::Constraints>)>) {
        let n = self.model.n;
        let (q0, v0) = (self.q, self.qd);
        self.budget.stage = 0;
        let lerp = |base: &VecN, d: &VecN, s: f64| -> VecN { core::array::from_fn(|i| base[i] + s * d[i]) };
        let a0 = match at_start {
            Some((t, k)) => self.accel_with(t, k, &q0, &v0),
            None => self.accel(&q0, &v0),
        };
        let k1 = (v0, a0);
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
        let s = self.budget.stages;
        for (j, w) in self.budget.work.iter_mut().enumerate() {
            *w += h / 6.0 * (s[0][j] + 2.0 * s[1][j] + 2.0 * s[2][j] + s[3][j]);
        }
    }

    /// `[T, V]` at the current state.
    pub fn energy(&mut self) -> [f64; 2] {
        self.model.energies(&self.q, &self.qd)
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
        for (kf, pose) in self.frame_poses(&q).enumerate() {
            o[slot::FRAMES + 3 * kf..slot::FRAMES + 3 * kf + 3].copy_from_slice(&pose);
        }
        let mut put = |at: usize, v: &[f64]| o[at..at + v.len()].copy_from_slice(v);
        let effort = (0..self.model.nu).map(|i| self.u[i] * self.u[i]).sum::<f64>().sqrt();
        put(slot::KIND, &[f64::from(self.kind as u32), f64::from(self.ctrl.phase()), eq, ex, effort]);
        put(slot::ENERGY, &[ke, pe, ke + pe]);
        put(slot::Q, &q[..n]);
        put(slot::U, &self.u[..self.model.nu]);
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
        let t = self.model.terms(&q, &qd);
        let elements = self.model.element_forces(&q, &qd);
        let (stops, tau) = self.applied(&q, &qd);
        let k = dynamics::constraints(&mut self.model, &q, &qd);
        let qdd = self.accel_with(&t, k.as_ref(), &q, &qd);
        let at_lambda = k.as_ref().map_or([0.0; MAXN], |k| k.apply_t(&self.lambda));
        self.model.aux.eval(&q, &qd, &mut self.aux);
        let (a, ao) = (&self.aux, self.model.aux_off);
        let mut mdot: MatN = [0.0; MAXN * MAXN];
        mdot[..n * n].copy_from_slice(&a[ao.mdot..ao.mdot + n * n]);
        // Lemma 4.2: S = Ṁ − 2C is skew-symmetric
        let s = |r: usize, c: usize| mdot[r * n + c] - 2.0 * t.c[r * n + c];
        let skew = (0..n).flat_map(|i| (0..n).map(move |j| (i, j))).map(|(i, j)| s(i, j) + s(j, i));
        let skew = crate::mat::max_abs(skew);
        let eig = sym_eigenvalues(n, &t.m);
        let power: f64 = (0..n).map(|i| qd[i] * tau[i]).sum();
        let dissipation = self.joint_friction(&qd) + elements.dissipation;
        let mqdd = mat_vec(n, &t.m, &qdd);
        let cqd = t.c_qd(n, &qd);
        let (ke, pe) = (a[ao.energy], a[ao.energy + 1] + elements.energy);
        // ∂V/∂q with the springs (the dampers are on the friction side of N)
        let dv: VecN = core::array::from_fn(|i| if i < n { a[ao.dv + i] - elements.spring[i] } else { 0.0 });
        let pl = &self.plan;
        let flag = |b: bool| f64::from(u8::from(b));
        let w = manipulability(n, &a[ao.jp..ao.jp + 2 * n]);

        o[..off::SLOT].fill(0.0);
        for (kf, pose) in self.frame_poses(&q).enumerate() {
            o[off::FRAMES + 3 * kf..off::FRAMES + 3 * kf + 3].copy_from_slice(&pose);
        }
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
        put(off::DV, &dv[..n]);
        put(off::MQDD, &mqdd[..n]);
        put(off::CQD, &cqd[..n]);
        put(off::MDOT, &mdot[..n * n]);
        put(off::ENERGY, &[ke, pe, ke + pe, power, dissipation, skew, eig[..n].iter().product()]);
        let b = &self.budget;
        put(off::WORK, &[b.work[0], b.work[1], b.work[2], b.work[3], b.work[4], b.e0, b.residual(ke + pe)]);
        put(off::EIG_M, &eig[..n]);
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
        let g = &pl.goals;
        put(
            off::GOALS,
            &[g.orient, g.orient_target, g.elbow, g.elbow_target[0], g.elbow_target[1], g.elbow_body as f64],
        );
        let violation = k.as_ref().map_or(0.0, dynamics::Constraints::max_violation);
        put(off::NC, &[self.model.nc as f64, self.model.ncon as f64, violation]);
        put(off::LAMBDA, &self.lambda[..self.model.ncon]);
        put(off::AT_LAMBDA, &at_lambda[..n]);
        // body kinematics: v_c = J_c q̇ (tape), a_c = J_c q̈ + (∂v_c/∂q) q̇, ω = k·q̇ and α = k·q̈ (frame
        // angles are linear in q)
        let acom: Vec<f64> = (0..2 * nb)
            .map(|r| a[ao.dvcom + r] + (0..n).map(|j| a[ao.jcom + r * n + j] * qdd[j]).sum::<f64>())
            .collect();
        put(off::VCOM, &a[ao.vcom..ao.vcom + 2 * nb]);
        let mut jcom = vec![0.0; 2 * nb * MAXN];
        for r in 0..2 * nb {
            jcom[r * MAXN..r * MAXN + n].copy_from_slice(&a[ao.jcom + r * n..ao.jcom + (r + 1) * n]);
        }
        put(off::JCOM, &jcom);
        put(off::ACOM, &acom);
        let rate = |k: usize, v: &[f64]| -> f64 {
            let ang = &self.model.frame_angles[k + 1];
            (0..n).map(|j| f64::from(ang.k[j]) * v[j]).sum()
        };
        let omega: Vec<f64> = (0..nb).map(|k| rate(k, &qd)).collect();
        let alpha: Vec<f64> = (0..nb).map(|k| rate(k, &qdd)).collect();
        put(off::OMEGA, &omega);
        put(off::ALPHA, &alpha);
        put(off::ELEMENT_F, &elements.tension[..self.model.element_values.len()]);
    }
}
