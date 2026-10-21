//! What every controller of one simulation shares.
//!
//! Targets (projected onto the constraint manifold), the equilibrium feed-forward, the linearisation used by LQR and MPC, smooth joint- and task-space references,
//! inverse kinematics, the external push on the tool, and the secondary (null-space) goals.

use crate::MAXN;
use crate::dynamics::{self, Constraints, MAXK, MatN, Terms, VecN, inverse_dynamics, mat_vec};
use crate::lqr::{c2d, dlqr};
use crate::mat::{DMat, solve_dense};
use crate::model::Model;

/// Wrap an angle to (−π, π].
#[must_use]
pub fn wrap(a: f64) -> f64 {
    use core::f64::consts::{PI, TAU};
    let w = (a + PI).rem_euclid(TAU) - PI;
    if w <= -PI { w + TAU } else { w }
}

/// Secondary goals pursued in the null space of the tool task.
#[derive(Clone, Copy, Debug)]
pub struct Goals {
    /// Posture: natural frequency (rad/s) of the pull towards q* (0 = off).
    pub posture: f64,
    /// Manipulability: gain of the gradient ascent on w = √det(J Jᵀ) (0 = off).
    pub manipulability: f64,
    /// Priority 2: tool orientation φ_t → `orient_target`, natural frequency (rad/s, 0 = off).
    pub orient: f64,
    pub orient_target: f64,
    /// Priority 3: the elbow (the origin of body `elbow_body`) → `elbow_target`, natural frequency
    /// (rad/s, 0 = off). The planar counterpart of the arm angle that resolves a 7-DOF arm's redundancy.
    pub elbow: f64,
    pub elbow_target: [f64; 2],
    pub elbow_body: usize,
}

/// A secondary task for one step: its Jacobian (rows × n, row-major), J̇q̇, the desired task acceleration
/// a = ω²e − 2ω ẋ and the error e, for `rows` = 1 (orientation) or 2 (a point).
#[derive(Clone, Copy, Debug)]
pub struct SubTask {
    pub rows: usize,
    pub j: [f64; 2 * MAXN],
    pub jdqd: [f64; 2],
    pub a: [f64; 2],
    pub e: [f64; 2],
}

/// The body halfway along the chain from the ground to the tool: a natural "elbow".
fn middle_of_tool_chain(model: &Model) -> usize {
    let desc = &model.desc;
    let Some(mut b) = desc.tool else { return 0 };
    let mut chain = vec![b];
    while let Some(p) = desc.bodies[b].parent {
        chain.push(p);
        b = p;
    }
    chain[chain.len() / 2]
}

/// Linearisation in tangent coordinates x = (ξ, ξ̇) with q ≈ q* + Z ξ (Z = I for open chains).
#[derive(Clone, Debug)]
pub struct Lin {
    pub z: DMat,
    pub a: DMat,
    pub b: DMat,
}

/// Tool point kinematics at the current state.
#[derive(Clone, Copy, Debug, Default)]
pub struct Tool {
    pub p: [f64; 2],
    pub v: [f64; 2],
    /// ∂p/∂q (2 × n, stride n).
    pub jp: [f64; 2 * MAXN],
    /// J̇ q̇.
    pub jdqd: [f64; 2],
}

impl Tool {
    /// Evaluate the tool tape at q and along q̇ (J̇ q̇ by a directional finite difference).
    pub fn eval(model: &mut Model, q: &[f64], qd: &[f64]) -> Option<Self> {
        let n = model.n;
        let tape = model.tool.as_mut()?;
        let zero = [0.0; MAXN];
        let mut buf = [0.0; 2 + 2 * MAXN];
        tape.eval(q, &zero, &mut buf);
        let mut t = Self { p: [buf[0], buf[1]], ..Self::default() };
        t.jp[..2 * n].copy_from_slice(&buf[2..2 + 2 * n]);
        for r in 0..2 {
            t.v[r] = (0..n).map(|j| t.jp[r * n + j] * qd[j]).sum();
        }
        let speed = qd[..n].iter().fold(0.0_f64, |m, v| m.max(v.abs()));
        if speed > 1e-12 {
            let h = 1e-6 / speed;
            let qh: VecN = core::array::from_fn(|i| if i < n { q[i] + h * qd[i] } else { 0.0 });
            tape.eval(&qh, &zero, &mut buf);
            for r in 0..2 {
                let jh: f64 = (0..n).map(|j| buf[2 + r * n + j] * qd[j]).sum();
                t.jdqd[r] = (jh - t.v[r]) / h;
            }
        }
        Some(t)
    }
}

#[derive(Clone, Debug)]
#[allow(clippy::struct_excessive_bools)] // independent switches
pub struct Plan {
    // ---- inputs
    pub q_target: VecN,
    pub tool_target: [f64; 2],
    /// Follow `tool_target` (task controllers directly, joint controllers through IK) instead of q*.
    pub use_tool: bool,
    pub ref_wn: f64,
    /// Pointer position of a push on the tool, and the push stiffness.
    pub push: Option<[f64; 2]>,
    pub push_k: f64,
    pub goals: Goals,
    // ---- design
    pub q_star: VecN,
    /// Tool position at q* (task target when no tool target is set).
    pub x_star: [f64; 2],
    pub u_ff: VecN,
    pub eq_residual: f64,
    /// [B −Aᵀ] spans ℝⁿ: any generalised force can be produced (full actuation on the manifold).
    pub ct_ok: bool,
    /// Degrees of freedom (dimension of the tangent space).
    pub dof: usize,
    pub lin: Option<Lin>,
    /// Can linear feedback hold q*? False when an unstable mode of the linearisation is out of reach of
    /// every input (e.g. a passive bar standing on a point that the inputs only move along it).
    pub stabilisable: bool,
    /// M(q*): shapes joint-space gains (K = ω² M(q*)), so that near q* every closed-loop mode has the
    /// designed natural frequency and damping.
    pub m_star: MatN,
    pub v_star: f64,
    /// Energy scale of the passive bodies, 2 Σ m g |r| (swing-up).
    pub e_ref: f64,
    // ---- references
    pub q_ref: VecN,
    pub qd_ref: VecN,
    pub qdd_ref: VecN,
    pub ik_goal: VecN,
    pub reachable: bool,
    /// Where the IK aims the tool: the target itself when it is reachable, otherwise (just inside) the
    /// nearest point the mechanism can reach.
    pub ik_point: [f64; 2],
    /// The adaptive part of the IK's damping.
    ik_mu: f64,
    pub x_ref: [f64; 2],
    pub xd_ref: [f64; 2],
    pub xdd_ref: [f64; 2],
    /// External force on the tool this step (world frame).
    pub f_ext: [f64; 2],
    synced: bool,
}

impl Plan {
    #[must_use]
    pub fn new(model: &Model) -> Self {
        let mut q_target = [0.0; MAXN];
        q_target[..model.n].copy_from_slice(&model.q_target);
        Self {
            q_target,
            tool_target: [0.0; 2],
            use_tool: false,
            ref_wn: 4.0,
            push: None,
            push_k: 200.0,
            goals: Goals {
                posture: 4.0,
                manipulability: 0.0,
                orient: 0.0,
                orient_target: 0.0,
                elbow: 0.0,
                elbow_target: [0.0; 2],
                elbow_body: middle_of_tool_chain(model),
            },
            q_star: q_target,
            x_star: [0.0; 2],
            u_ff: [0.0; MAXN],
            eq_residual: 0.0,
            ct_ok: false,
            stabilisable: false,
            dof: model.n,
            lin: None,
            m_star: core::array::from_fn(|k| f64::from(u8::from(k % (MAXN + 1) == 0))),
            v_star: 0.0,
            e_ref: 1.0,
            q_ref: [0.0; MAXN],
            qd_ref: [0.0; MAXN],
            qdd_ref: [0.0; MAXN],
            ik_goal: q_target,
            reachable: true,
            ik_point: [0.0; 2],
            ik_mu: 0.01,
            x_ref: [0.0; 2],
            xd_ref: [0.0; 2],
            xdd_ref: [0.0; 2],
            f_ext: [0.0; 2],
            synced: false,
        }
    }

    /// Error `target − q` (wrapped for revolute coordinates).
    #[must_use]
    pub fn error(model: &Model, i: usize, q: f64, target: f64) -> f64 {
        if model.revolute[i] { wrap(target - q) } else { target - q }
    }

    /// Restart the references at the current state (after reset, mode change or a jump).
    pub fn sync(&mut self, model: &mut Model, q: &[f64]) {
        let n = model.n;
        self.q_ref[..n].copy_from_slice(&q[..n]);
        self.qd_ref = [0.0; MAXN];
        self.qdd_ref = [0.0; MAXN];
        self.ik_goal[..n].copy_from_slice(&q[..n]);
        self.ik_mu = 0.01;
        if let Some(t) = Tool::eval(model, q, &[0.0; MAXN]) {
            self.x_ref = t.p;
            self.ik_point = t.p;
        }
        self.xd_ref = [0.0; 2];
        self.xdd_ref = [0.0; 2];
        self.synced = true;
    }

    /// Recompute everything that depends on parameters (`p`) or targets.
    pub fn design(&mut self, model: &mut Model, p: &[f64]) {
        let n = model.n;
        let zero = [0.0; MAXN];
        // q* on the manifold: keep the actuated coordinates where the user put them if the loops allow it,
        // and start the passive ones from the current configuration (same assembly mode)
        let mut seed = self.q_target;
        if model.closed() && self.synced {
            for &i in &model.unactuated {
                seed[i] = self.q_ref[i];
            }
        }
        let (mut qs, mut v0) = (seed, zero);
        dynamics::project_holding(model, &mut qs, &mut v0, &model.actuated.clone());
        if dynamics::constraints(model, &qs, &zero).is_some_and(|k| k.max_violation() > 1e-9) {
            (qs, v0) = (seed, zero);
            dynamics::project(model, &mut qs, &mut v0);
        }
        // a passive joint cannot be held anywhere: it rests where its potential torque vanishes. Keep the
        // actuated coordinates at their targets and let the passive ones settle (open chains; with loops
        // the constraint forces share the load and the projection above already balances it)
        if !model.closed() && !model.unactuated.is_empty() {
            settle_passive(model, &mut qs);
        }
        self.q_star = qs;
        if let Some(t) = Tool::eval(model, &qs, &zero) {
            self.x_star = t.p;
        }
        let t = model.terms(&qs, &zero);
        let k = dynamics::constraints(model, &qs, &zero);
        let dv = model.potential_grad(&qs);
        (self.u_ff, self.eq_residual) = inverse_dynamics(model, k.as_ref(), &dv);
        self.v_star = model.energies(&qs, &zero)[1];
        // gains shaped by the full inertia: K = ω² M(q*) puts every linearised mode at ω (a diagonal choice
        // leaves strongly coupled chains with modes hundreds of times stiffer or slower than designed)
        self.m_star = [0.0; MAXN * MAXN];
        self.m_star[..n * n].copy_from_slice(&t.m[..n * n]);
        self.ct_ok = model.nu > 0
            && (0..n).all(|i| {
                let e: VecN = core::array::from_fn(|j| f64::from(u8::from(i == j)));
                inverse_dynamics(model, k.as_ref(), &e).1 < 1e-6
            });
        let z = k.map_or_else(|| DMat::identity(n), |k| k.tangent_basis());
        self.dof = z.cols;
        self.lin = if model.nu > 0 && z.cols > 0 { Some(self.linearise(model, z)) } else { None };
        self.stabilisable = self.lin.as_ref().is_some_and(stabilisable);
        let vars = model.vars;
        let passive: Vec<usize> = if model.closed() { (0..n).collect() } else { model.unactuated.clone() };
        self.e_ref = passive
            .iter()
            .map(|&j| model.coord_body[j])
            .map(|i| 2.0 * p[vars.m(i)] * p[vars.g()] * p[vars.r(i)].abs())
            .sum::<f64>()
            .max(1e-6);
    }

    /// Linearisation of the (constrained) dynamics about (q*, u*) in tangent coordinates, by central
    /// differences of the same model the simulator integrates.
    fn linearise(&self, model: &mut Model, z: DMat) -> Lin {
        let (n, nu, r) = (model.n, model.nu, z.cols);
        let (qs, u_ff) = (self.q_star, self.u_ff);
        let mut f = |xi: &[f64], xid: &[f64], du: &[f64]| -> Vec<f64> {
            let mut q: VecN = core::array::from_fn(|i| {
                if i < n { qs[i] + (0..r).map(|c| z[(i, c)] * xi[c]).sum::<f64>() } else { 0.0 }
            });
            let mut qd: VecN =
                core::array::from_fn(|i| if i < n { (0..r).map(|c| z[(i, c)] * xid[c]).sum() } else { 0.0 });
            dynamics::project(model, &mut q, &mut qd);
            let tau: VecN = core::array::from_fn(|i| {
                if i < n { (0..nu).map(|a| model.b[i * nu + a] * (u_ff[a] + du[a])).sum() } else { 0.0 }
            });
            let (qdd, _) = dynamics::accel(model, &q, &qd, &tau, true);
            (0..r).map(|c| (0..n).map(|i| z[(i, c)] * qdd[i]).sum()).collect()
        };
        let h = 1e-6;
        let mut column = |which: usize, j: usize| -> Vec<f64> {
            let mut plus = [vec![0.0; r], vec![0.0; r], vec![0.0; nu]];
            let mut minus = plus.clone();
            plus[which][j] = h;
            minus[which][j] = -h;
            let fp = f(&plus[0], &plus[1], &plus[2]);
            let fm = f(&minus[0], &minus[1], &minus[2]);
            fp.iter().zip(&fm).map(|(a, b)| (a - b) / (2.0 * h)).collect()
        };
        let fx: Vec<Vec<f64>> = (0..r).map(|j| column(0, j)).collect();
        let fv: Vec<Vec<f64>> = (0..r).map(|j| column(1, j)).collect();
        let fu: Vec<Vec<f64>> = (0..nu).map(|j| column(2, j)).collect();
        let a = DMat::from_fn(2 * r, 2 * r, |i, j| match (i < r, j < r) {
            (true, true) => 0.0,
            (true, false) => f64::from(u8::from(j - r == i)),
            (false, true) => fx[j][i - r],
            (false, false) => fv[j - r][i - r],
        });
        let b = DMat::from_fn(2 * r, nu, |i, j| if i < r { 0.0 } else { fu[j][i - r] });
        Lin { z, a, b }
    }

    /// Tangent-coordinate state x = (Zᵀ(q − q*), Zᵀ q̇).
    #[must_use]
    pub fn tangent_state(&self, model: &Model, q: &[f64], qd: &[f64]) -> Vec<f64> {
        let Some(l) = &self.lin else { return Vec::new() };
        let (n, r) = (model.n, l.z.cols);
        let dq: VecN =
            core::array::from_fn(|i| if i < n { -Self::error(model, i, q[i], self.q_star[i]) } else { 0.0 });
        let mut x = vec![0.0; 2 * r];
        for c in 0..r {
            x[c] = (0..n).map(|i| l.z[(i, c)] * dq[i]).sum();
            x[r + c] = (0..n).map(|i| l.z[(i, c)] * qd[i]).sum();
        }
        x
    }

    /// Advance the references by one step: joint reference (towards q*, or the IK goal when following the
    /// tool target) and task reference (towards the tool target — the nearest reachable point when it is out
    /// of reach — or the tool position at q*).
    pub fn advance(&mut self, model: &mut Model, dt: f64) {
        let n = model.n;
        if !self.synced {
            return;
        }
        let ik = self.use_tool && model.has_tool() && self.ct_ok;
        if ik {
            self.ik_step(model);
        }
        let wn = self.ref_wn;
        for i in 0..n {
            let goal = if ik { self.ik_goal[i] } else { self.q_star[i] };
            let goal = self.q_ref[i] + Self::error(model, i, self.q_ref[i], goal);
            self.qdd_ref[i] = wn * wn * (goal - self.q_ref[i]) - 2.0 * wn * self.qd_ref[i];
            self.qd_ref[i] += self.qdd_ref[i] * dt;
            self.q_ref[i] += self.qd_ref[i] * dt;
        }
        // a target out of reach is replaced by the nearest point the arm can reach: a task-space spring
        // stretched towards a point it can never get to pulls the arm ever harder against its own
        // singularity (an acceleration that cannot be realised bends a redundant arm back and forth)
        let goal = match (self.use_tool, ik && !self.reachable) {
            (true, true) => self.ik_point,
            (true, false) => self.tool_target,
            (false, _) => self.x_star,
        };
        for r in 0..2 {
            self.xdd_ref[r] = wn * wn * (goal[r] - self.x_ref[r]) - 2.0 * wn * self.xd_ref[r];
            self.xd_ref[r] += self.xdd_ref[r] * dt;
            self.x_ref[r] += self.xd_ref[r] * dt;
        }
    }

    /// ‖e‖² + ‖φ‖² of the tool target and the loop closures at `q`, with the tool there.
    fn ik_cost(&self, model: &mut Model, q: &[f64]) -> Option<(f64, Tool)> {
        let zero = [0.0; MAXN];
        let tool = Tool::eval(model, q, &zero)?;
        let e = [self.tool_target[0] - tool.p[0], self.tool_target[1] - tool.p[1]];
        let phi = dynamics::constraints(model, q, &zero)
            .map_or(0.0, |k| (0..k.m).map(|r| k.phi[r] * k.phi[r]).sum());
        Some((e[0].mul_add(e[0], e[1] * e[1]) + phi, tool))
    }

    /// Two Levenberg–Marquardt steps of inverse kinematics on `ik_goal`, keeping loops closed:
    /// `[A; J] Δq ≈ [−φ; e]` with λ = ½‖e‖² + μ (Sugihara, 2011). A step is kept only when it lowers the
    /// error (μ then halves), otherwise μ quadruples and the step is retried: Gauss–Newton close to a
    /// solution, a descent on ‖e‖² far from one, which ends at the nearest reachable point when the target
    /// is out of reach (a fixed λ flips the elbow back and forth across the stretched pose instead).
    fn ik_step(&mut self, model: &mut Model) {
        let n = model.n;
        let zero = [0.0; MAXN];
        for _ in 0..2 {
            let Some((cost, tool)) = self.ik_cost(model, &self.ik_goal) else { return };
            let e = [self.tool_target[0] - tool.p[0], self.tool_target[1] - tool.p[1]];
            let k = dynamics::constraints(model, &self.ik_goal, &zero);
            let m = k.as_ref().map_or(0, |k| k.m);
            let rows = m + 2;
            let mut s = [0.0; MAXK * MAXN];
            let mut rhs = [0.0; MAXK];
            if let Some(k) = &k {
                s[..m * n].copy_from_slice(&k.a[..m * n]);
                for r in 0..m {
                    rhs[r] = -k.phi[r];
                }
            }
            s[m * n..(m + 2) * n].copy_from_slice(&tool.jp[..2 * n]);
            rhs[m] = e[0];
            rhs[m + 1] = e[1];
            let lambda = 0.5 * e[0].mul_add(e[0], e[1] * e[1]) + self.ik_mu;
            let mut sst = [0.0; MAXK * MAXK];
            for i in 0..rows {
                for j in 0..rows {
                    sst[i * rows + j] = (0..n).map(|c| s[i * n + c] * s[j * n + c]).sum::<f64>()
                        + if i == j { if i < m { 1e-9 } else { lambda } } else { 0.0 };
                }
            }
            if !solve_dense(rows, &mut sst[..rows * rows], &mut rhs[..rows]) {
                return;
            }
            let mut q = self.ik_goal;
            for c in 0..n {
                let step: f64 = (0..rows).map(|i| s[i * n + c] * rhs[i]).sum();
                q[c] += step.clamp(-0.05, 0.05);
            }
            match self.ik_cost(model, &q) {
                Some((c, _)) if c < cost => {
                    self.ik_goal = q;
                    self.ik_mu = (self.ik_mu * 0.5).max(1e-3);
                }
                _ => self.ik_mu = (self.ik_mu * 4.0).min(1e3),
            }
        }
        let Some((_, tool)) = self.ik_cost(model, &self.ik_goal) else { return };
        let e = [self.tool_target[0] - tool.p[0], self.tool_target[1] - tool.p[1]];
        let dist = e[0].hypot(e[1]);
        self.reachable = dist < 2e-3;
        // out of reach the IK ends on the boundary of the workspace, where the arm is singular: aim a little
        // inside it (0.5 % of the arm's size √tr(J Jᵀ), continuous at the boundary) to keep it controllable
        let size = tool.jp[..2 * n].iter().map(|v| v * v).sum::<f64>().sqrt();
        let back = if dist > 0.0 { (0.005 * size).min(dist) / dist } else { 0.0 };
        self.ik_point = [tool.p[0] - back * e[0], tool.p[1] - back * e[1]];
    }

    /// Task-space reference at this step: (x_d, ẋ_d, ẍ_d).
    #[must_use]
    pub fn task_ref(&self) -> ([f64; 2], [f64; 2], [f64; 2]) {
        (self.x_ref, self.xd_ref, self.xdd_ref)
    }

    /// The enabled secondary tasks, highest priority first: tool orientation, then the elbow point.
    pub fn sub_tasks(&self, model: &mut Model, q: &[f64], qd: &[f64]) -> Vec<SubTask> {
        let n = model.n;
        let mut out = Vec::new();
        let g = self.goals;
        if g.orient > 0.0 && model.has_tool() {
            // the tool frame's angle is linear in q: φ = k·q + c, so J = k and J̇ = 0
            let angle = &model.frame_angles[model.desc.nb() + 1];
            let mut j = [0.0; 2 * MAXN];
            for (i, &k) in angle.k.iter().enumerate().take(n) {
                j[i] = f64::from(k);
            }
            let phi = angle.eval(q);
            let rate: f64 = (0..n).map(|i| j[i] * qd[i]).sum();
            let e = wrap(g.orient_target - phi);
            out.push(SubTask {
                rows: 1,
                j,
                jdqd: [0.0; 2],
                a: [g.orient * g.orient * e - 2.0 * g.orient * rate, 0.0],
                e: [e, 0.0],
            });
        }
        if g.elbow > 0.0 && g.elbow_body < model.desc.nb() {
            let (p, j, jdqd) = point_kinematics(model, g.elbow_body + 1, q, qd);
            let v: [f64; 2] = core::array::from_fn(|r| (0..n).map(|c| j[r * n + c] * qd[c]).sum());
            let e = [g.elbow_target[0] - p[0], g.elbow_target[1] - p[1]];
            let a = core::array::from_fn(|r| g.elbow * g.elbow * e[r] - 2.0 * g.elbow * v[r]);
            out.push(SubTask { rows: 2, j, jdqd, a, e });
        }
        out
    }

    /// Secondary-goal torque τ₀ (posture and manipulability), to be projected into the task null space.
    ///
    /// The posture acceleration is shaped by M(q*) along the tool's chain; joints beyond the tool (a
    /// gripper's fingers) get their own share only. Through the coupling of M a chain that cannot follow its
    /// posture (an arm stretched towards an unreachable target) would otherwise push light distal bodies
    /// round and round: the coupling is about (arm length / finger length) times their own inertia.
    pub fn secondary(&self, model: &mut Model, q: &[f64], qd: &[f64]) -> VecN {
        let n = model.n;
        let a = self.secondary_accel(model, q, qd);
        let chain: VecN = core::array::from_fn(|i| if i < n && !model.beyond_tool[i] { a[i] } else { 0.0 });
        let mut tau = mat_vec(n, &self.m_star, &chain);
        for i in (0..n).filter(|&i| model.beyond_tool[i]) {
            tau[i] = self.m_star[i * n + i] * a[i];
        }
        tau
    }

    /// The posture spring–damper at ω and the climb along ∇w, as joint accelerations.
    pub fn secondary_accel(&self, model: &mut Model, q: &[f64], qd: &[f64]) -> VecN {
        let n = model.n;
        let w = self.goals.posture;
        let mut a: VecN = core::array::from_fn(|i| {
            if i < n { w * w * Self::error(model, i, q[i], self.q_star[i]) - 2.0 * w * qd[i] } else { 0.0 }
        });
        if self.goals.manipulability > 0.0 {
            let grad = manipulability_gradient(model, q);
            for i in 0..n {
                a[i] += self.goals.manipulability * grad[i];
            }
        }
        a
    }
}

/// Does a reference LQR (unit weights) stabilise the linearisation? Its closed loop over 2¹⁴ steps of
/// 10 ms (≈ 164 s) must decay: a mode no input reaches grows (or, undamped, at least does not die out).
fn stabilisable(l: &Lin) -> bool {
    let (nx, nu) = (l.a.rows, l.b.cols);
    let dt = 0.01;
    let (ad, bd) = c2d(&l.a, &l.b, dt);
    let Some(k) = dlqr(&ad, &bd, &DMat::identity(nx).scale(dt), &DMat::identity(nu).scale(dt)) else {
        return false;
    };
    let mut m = &ad - &(&bd * &k);
    for _ in 0..14 {
        m = &m * &m;
        let big = m.max_abs();
        if !big.is_finite() || big > 1e12 {
            return false;
        }
    }
    m.max_abs() < 1.0
}

/// Newton's method on the passive rows of ∂V/∂q (gravity and springs), from the user's targets: the
/// passive coordinates move to the nearest static equilibrium with the actuated ones held. `q` is left as
/// it was if no equilibrium is found (the residual then tells).
fn settle_passive(model: &mut Model, q: &mut VecN) {
    let pas = model.unactuated.clone();
    let m = pas.len();
    let residual = |model: &mut Model, q: &VecN| -> (Vec<f64>, f64) {
        let g = model.potential_grad(q);
        let r: Vec<f64> = pas.iter().map(|&i| g[i]).collect();
        let norm = r.iter().map(|v| v * v).sum::<f64>().sqrt();
        (r, norm)
    };
    let mut x = *q;
    let (mut r, mut norm) = residual(model, &x);
    for _ in 0..60 {
        if norm < 1e-10 {
            break;
        }
        // ∂(∂V/∂q_p)/∂q_p by central differences of the same model
        let h = 1e-6;
        let mut jac = vec![0.0; m * m];
        for (c, &j) in pas.iter().enumerate() {
            let (mut plus, mut minus) = (x, x);
            plus[j] += h;
            minus[j] -= h;
            let (gp, gm) = (model.potential_grad(&plus), model.potential_grad(&minus));
            for (row, &i) in pas.iter().enumerate() {
                jac[row * m + c] = (gp[i] - gm[i]) / (2.0 * h);
            }
        }
        let mut dx: Vec<f64> = r.iter().map(|v| -v).collect();
        if !solve_dense(m, &mut jac, &mut dx) {
            return;
        }
        // a bounded step, halved until the residual drops
        let big = dx.iter().fold(0.0_f64, |a, v| a.max(v.abs()));
        let mut t = if big > 0.5 { 0.5 / big } else { 1.0 };
        loop {
            let mut trial = x;
            for (c, &j) in pas.iter().enumerate() {
                trial[j] += t * dx[c];
            }
            let (rt, nt) = residual(model, &trial);
            if nt < norm || t < 1e-4 {
                (x, r, norm) = (trial, rt, nt);
                break;
            }
            t *= 0.5;
        }
    }
    if norm < 1e-8 {
        for &j in &pas {
            q[j] = if model.revolute[j] { wrap(x[j]) } else { x[j] };
        }
    }
}

/// Manipulability w(q) = √det(J Jᵀ) of the tool's 2 × n position Jacobian.
#[must_use]
pub fn manipulability(n: usize, jp: &[f64]) -> f64 {
    let dot = |a: usize, b: usize| (0..n).map(|c| jp[a * n + c] * jp[b * n + c]).sum::<f64>();
    (dot(0, 0) * dot(1, 1) - dot(0, 1) * dot(0, 1)).max(0.0).sqrt()
}

/// ∇w by central differences.
pub fn manipulability_gradient(model: &mut Model, q: &[f64]) -> VecN {
    let n = model.n;
    let zero = [0.0; MAXN];
    let mut g = [0.0; MAXN];
    let h = 1e-6;
    for i in 0..n {
        let mut qp: VecN = core::array::from_fn(|k| if k < n { q[k] } else { 0.0 });
        let mut qm = qp;
        qp[i] += h;
        qm[i] -= h;
        let (Some(tp), Some(tm)) = (Tool::eval(model, &qp, &zero), Tool::eval(model, &qm, &zero)) else {
            return g;
        };
        g[i] = (manipulability(n, &tp.jp) - manipulability(n, &tm.jp)) / (2.0 * h);
    }
    g
}

/// Resolve accelerations: the q̈ closest to `pref` in the kinetic-energy metric (‖·‖_M) that satisfies the task
/// rows `S q̈ = b` and the loop constraints `A q̈ = −Ȧ q̇`:
///
/// ```text
/// q̈ = q̈₀ + M⁻¹ Sᵀ (S M⁻¹ Sᵀ)⁻¹ (b − S q̈₀)
/// ```
///
/// With the M-metric the correction is dynamically consistent: secondary goals in `pref` cannot disturb the
/// task. `s` is row-major (rows × n).
#[must_use]
pub fn resolve_accel(
    n: usize,
    t: &Terms,
    k: Option<&Constraints>,
    task: (&[f64], &[f64]),
    pref: &VecN,
) -> VecN {
    let m = k.map_or(0, |k| k.m);
    let task_rows = task.1.len();
    let rows = m + task_rows;
    let mut s = [0.0; MAXK * MAXN];
    let mut b = [0.0; MAXK];
    if let Some(k) = k {
        s[..m * n].copy_from_slice(&k.a[..m * n]);
        for r in 0..m {
            b[r] = -k.adq[r];
        }
    }
    s[m * n..rows * n].copy_from_slice(&task.0[..task_rows * n]);
    b[m..rows].copy_from_slice(task.1);
    // M⁻¹ Sᵀ, column by column
    let mut minv_st = [0.0; MAXN * MAXK];
    for r in 0..rows {
        let mut col = [0.0; MAXN];
        let rhs: VecN = core::array::from_fn(|c| if c < n { s[r * n + c] } else { 0.0 });
        if !crate::mat::cholesky_solve(n, &t.m, &rhs, &mut col) {
            return *pref;
        }
        for c in 0..n {
            minv_st[c * rows + r] = col[c];
        }
    }
    let mut g = [0.0; MAXK * MAXK];
    let mut y = [0.0; MAXK];
    for i in 0..rows {
        for j in 0..rows {
            g[i * rows + j] = (0..n).map(|c| s[i * n + c] * minv_st[c * rows + j]).sum::<f64>()
                + if i == j { 1e-9 } else { 0.0 };
        }
        y[i] = b[i] - (0..n).map(|c| s[i * n + c] * pref[c]).sum::<f64>();
    }
    if !solve_dense(rows, &mut g[..rows * rows], &mut y[..rows]) {
        return *pref;
    }
    core::array::from_fn(|c| {
        if c < n { pref[c] + (0..rows).map(|r| minv_st[c * rows + r] * y[r]).sum::<f64>() } else { 0.0 }
    })
}

/// Position, Jacobian (2 × n) and J̇q̇ of frame `k`'s origin, by central differences of the frame tape.
pub fn point_kinematics(
    model: &mut Model,
    k: usize,
    q: &[f64],
    qd: &[f64],
) -> ([f64; 2], [f64; 2 * MAXN], [f64; 2]) {
    let n = model.n;
    let len = model.points.len();
    let mut buf = vec![0.0; len];
    let zero = [0.0; MAXN];
    let mut at = |model: &mut Model, x: &VecN| -> [f64; 2] {
        model.points.eval(x, &zero, &mut buf);
        [buf[2 * k], buf[2 * k + 1]]
    };
    let base: VecN = core::array::from_fn(|i| if i < n { q[i] } else { 0.0 });
    let p = at(model, &base);
    let h = 1e-6;
    let mut j = [0.0; 2 * MAXN];
    for c in 0..n {
        let (mut a, mut b) = (base, base);
        a[c] += h;
        b[c] -= h;
        let (pa, pb) = (at(model, &a), at(model, &b));
        for r in 0..2 {
            j[r * n + c] = (pa[r] - pb[r]) / (2.0 * h);
        }
    }
    // J̇q̇ = d²p/ds² along q + s q̇ (second directional difference)
    let speed = qd[..n].iter().fold(0.0_f64, |m, v| m.max(v.abs()));
    let mut jdqd = [0.0; 2];
    if speed > 1e-12 {
        let s = 1e-4 / speed;
        let fwd: VecN = core::array::from_fn(|i| base[i] + if i < n { s * qd[i] } else { 0.0 });
        let back: VecN = core::array::from_fn(|i| base[i] - if i < n { s * qd[i] } else { 0.0 });
        let (pf, pb) = (at(model, &fwd), at(model, &back));
        for r in 0..2 {
            jdqd[r] = (pf[r] - 2.0 * p[r] + pb[r]) / (s * s);
        }
    }
    (p, j, jdqd)
}

#[cfg(test)]
mod tests {
    #[test]
    fn manipulability_of_a_square_jacobian_is_its_determinant() {
        let jp = [2.0, 1.0, 0.0, 3.0];
        assert!((super::manipulability(2, &jp) - 6.0).abs() < 1e-12);
    }
}
