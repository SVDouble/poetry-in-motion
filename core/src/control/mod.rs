//! Generic controllers for any built mechanism — open chains and closed loops alike (MLS ch. 4 §5, ch. 6).
//!
//! | mode | law | applies to |
//! |---|---|---|
//! | passive | u = 0 | all |
//! | PD | u = K_p e + K_v ė | actuated joints |
//! | PD + gravity | PD + u_g, with B u_g − Aᵀλ = ∂V/∂q | actuated joints |
//! | computed torque | B u − Aᵀλ = M q̈_d + C q̇ + N, q̈_d consistent with A q̈ + Ȧ q̇ = 0 | [B −Aᵀ] spans ℝⁿ |
//! | LQR | u = u* − K x, x in tangent coordinates of the constraint manifold | any, about an equilibrium |
//! | swing-up | energy pumping via partial feedback linearisation, LQR catch | underactuated |
//!
//! With no loops, A is empty and the laws reduce to their textbook open-chain forms.

use crate::MAXN;
use crate::dynamics::{self, Constraints, MAXK, Terms, VecN, inverse_dynamics, lstsq, mat_vec};
use crate::lqr::{c2d, dlqr};
use crate::mat::{DMat, solve_dense};
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
    /// Loop-closure constraints at this state (`None` for open chains).
    pub k: Option<&'a Constraints>,
    pub dt: f64,
}

#[derive(Clone, Debug)]
pub struct LqrDesign {
    /// Tangent basis Z (n × r) of the constraint manifold at q* (identity for open chains).
    pub z: DMat,
    /// Linearisation ẋ = A x + B u in tangent coordinates x = (ξ, ξ̇), q ≈ q* + Z ξ.
    pub a: DMat,
    pub b: DMat,
    pub k: DMat,
}

#[derive(Clone, Debug)]
#[allow(clippy::struct_excessive_bools)] // independent switches, not a hidden state machine
pub struct Ctrl {
    /// Requested target, and the nearest configuration on the constraint manifold (the one used).
    pub q_target: VecN,
    pub q_star: VecN,
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
    /// Computed torque can realise any acceleration on the manifold ([B −Aᵀ] spans ℝⁿ).
    pub ct_ok: bool,
    pub lqr: Option<LqrDesign>,
    pub lqr_qx: f64,
    pub lqr_qv: f64,
    pub lqr_r: f64,
    /// Feed-forward holding q*: B u* − Aᵀλ = ∂V/∂q(q*), and its residual (0 ⇔ q* is an equilibrium).
    pub u_ff: VecN,
    pub eq_residual: f64,
    pub phase: u32,
    /// Energy relative to the target: of the passive motion for open chains, total for loops.
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
            q_star: q_target,
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
            ct_ok: false,
            lqr: None,
            lqr_qx: 10.0,
            lqr_qv: 1.0,
            lqr_r: 1.0,
            u_ff: [0.0; MAXN],
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

    /// Recompute everything that depends on parameters or targets: q* on the manifold, feed-forward, gains,
    /// controller availability and the LQR.
    pub fn design(&mut self, model: &mut Model, p: &[f64], dt: f64) {
        let n = model.n;
        let zero = [0.0; MAXN];
        // q* on the manifold: keep the actuated coordinates where the user put them if the loops allow it, and
        // start the passive ones from the current configuration so q* stays in the same assembly mode
        let mut seed = self.q_target;
        if model.closed() && self.initialised {
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
        self.q_star = qs;
        let t = Terms::eval(&mut model.dynamics, n, &qs, &zero);
        let k = dynamics::constraints(model, &qs, &zero);
        let mut dv = [0.0; MAXN];
        model.gravity.eval(&qs, &zero, &mut dv);
        (self.u_ff, self.eq_residual) = inverse_dynamics(model, k.as_ref(), &dv);
        let mut en = [0.0; 2];
        model.energy.eval(&qs, &zero, &mut en);
        self.v_target = en[1];

        for i in 0..n {
            let mii = t.m[i * n + i].max(1e-9);
            self.kp[i] = self.pd_wn * self.pd_wn * mii;
            self.kd[i] = 2.0 * self.pd_wn * mii;
        }
        // computed torque needs B u − Aᵀλ to reach every generalised force
        self.ct_ok = model.nu > 0
            && (0..n).all(|i| {
                let e: VecN = core::array::from_fn(|j| f64::from(u8::from(i == j)));
                inverse_dynamics(model, k.as_ref(), &e).1 < 1e-6
            });

        // energy scale of the swing-up: twice the gravitational "height energy" of the passive bodies
        let vars = model.vars;
        let passive: Vec<usize> = if model.closed() { (0..n).collect() } else { model.unactuated.clone() };
        self.e_ref = passive
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

        self.lqr = self.design_lqr(model, k.as_ref(), dt);
    }

    /// LQR in tangent coordinates: q = proj(q* + Z ξ), q̇ = Z ξ̇, linearised by central differences of the
    /// constrained forward dynamics (exact structure for open chains, where Z = I).
    fn design_lqr(&self, model: &mut Model, k: Option<&Constraints>, dt: f64) -> Option<LqrDesign> {
        let (n, nu) = (model.n, model.nu);
        if nu == 0 {
            return None;
        }
        let z = k.map_or_else(|| DMat::identity(n), Constraints::tangent_basis);
        let r = z.cols;
        if r == 0 {
            return None;
        }
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
        // central differences around (ξ, ξ̇, u) = (0, 0, u*)
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
        let q = DMat::from_fn(2 * r, 2 * r, |i, j| {
            if i != j {
                0.0
            } else if i < r {
                self.lqr_qx * dt
            } else {
                self.lqr_qv * dt
            }
        });
        let rr = DMat::identity(nu).scale(self.lqr_r * dt);
        let (ad, bd) = c2d(&a, &b, dt);
        dlqr(&ad, &bd, &q, &rr).map(|k| LqrDesign { z, a, b, k })
    }

    fn error(model: &Model, i: usize, q: f64, target: f64) -> f64 {
        if model.revolute[i] { wrap(target - q) } else { target - q }
    }

    fn lqr_law(&self, model: &Model, q: &[f64], qd: &[f64], u: &mut VecN) {
        let Some(l) = &self.lqr else { return };
        let (n, r) = (model.n, l.z.cols);
        let dq: VecN =
            core::array::from_fn(|i| if i < n { -Self::error(model, i, q[i], self.q_star[i]) } else { 0.0 });
        let mut x = vec![0.0; 2 * r];
        for c in 0..r {
            x[c] = (0..n).map(|i| l.z[(i, c)] * dq[i]).sum();
            x[r + c] = (0..n).map(|i| l.z[(i, c)] * qd[i]).sum();
        }
        for a in 0..model.nu {
            u[a] = self.u_ff[a] - (0..2 * r).map(|j| l.k[(a, j)] * x[j]).sum::<f64>();
        }
    }

    /// One damped-least-squares step of inverse kinematics on `ik_goal`: move the tool towards `tool_target`
    /// while keeping the loops closed, i.e. solve `[A; J] Δq ≈ [−φ; e]`.
    fn ik_step(&mut self, model: &mut Model) {
        let n = model.n;
        let zero = [0.0; MAXN];
        for _ in 0..2 {
            let Some(tool) = &mut model.tool else { return };
            let mut buf = [0.0; 2 + 2 * MAXN];
            tool.eval(&self.ik_goal[..n], &zero, &mut buf);
            let e = [self.tool_target[0] - buf[0], self.tool_target[1] - buf[1]];
            let k = dynamics::constraints(model, &self.ik_goal, &zero);
            let m = k.as_ref().map_or(0, |k| k.m);
            let rows = m + 2;
            // S = [A; J], rhs = [−φ; e]; Δq = Sᵀ (S Sᵀ + λ² I)⁻¹ rhs
            let mut s = [0.0; MAXK * MAXN];
            let mut rhs = [0.0; MAXK];
            if let Some(k) = &k {
                s[..m * n].copy_from_slice(&k.a[..m * n]);
                for r in 0..m {
                    rhs[r] = -k.phi[r];
                }
            }
            s[m * n..(m + 2) * n].copy_from_slice(&buf[2..2 + 2 * n]);
            rhs[m] = e[0];
            rhs[m + 1] = e[1];
            let mut sst = [0.0; MAXK * MAXK];
            for i in 0..rows {
                for j in 0..rows {
                    sst[i * rows + j] = (0..n).map(|c| s[i * n + c] * s[j * n + c]).sum::<f64>()
                        + if i == j { if i < m { 1e-9 } else { 0.01 } } else { 0.0 };
                }
            }
            if !solve_dense(rows, &mut sst[..rows * rows], &mut rhs[..rows]) {
                return;
            }
            for c in 0..n {
                let step: f64 = (0..rows).map(|i| s[i * n + c] * rhs[i]).sum();
                self.ik_goal[c] += step.clamp(-0.05, 0.05);
            }
            self.reachable = e[0].hypot(e[1]) < 2e-3;
        }
    }

    fn advance_reference(&mut self, model: &mut Model, dt: f64) {
        let n = model.n;
        let use_ik = self.ik && model.has_tool() && self.ct_ok;
        if use_ik {
            self.ik_step(model);
        }
        let wn = self.ref_wn;
        for i in 0..n {
            let goal = if use_ik { self.ik_goal[i] } else { self.q_star[i] };
            // the 2π-equivalent goal closest to the reference (no needless full turns)
            let goal = self.q_ref[i] + Self::error(model, i, self.q_ref[i], goal);
            self.qdd_ref[i] = wn * wn * (goal - self.q_ref[i]) - 2.0 * wn * self.qd_ref[i];
            self.qd_ref[i] += self.qdd_ref[i] * dt;
            self.q_ref[i] += self.qd_ref[i] * dt;
        }
    }

    /// Returns u (actuator space) for the given mode.
    pub fn update(&mut self, mode: u32, model: &mut Model, ctx: &Ctx<'_>) -> VecN {
        let (n, nu) = (model.n, model.nu);
        let (q, qd, t) = (ctx.q, ctx.qd, ctx.terms);
        if !self.initialised {
            self.sync(q);
        }
        self.advance_reference(model, ctx.dt);

        // energy relative to the target (passive motion for open chains, total with loops)
        let mut en = [0.0; 2];
        model.energy.eval(q, qd, &mut en);
        let ke = if model.closed() {
            en[0]
        } else {
            let un = &model.unactuated;
            un.iter()
                .flat_map(|&i| un.iter().map(move |&j| (i, j)))
                .map(|(i, j)| 0.5 * t.m[i * n + j] * qd[i] * qd[j])
                .sum()
        };
        self.e_u = ke + en[1] - self.v_target;

        let e: VecN =
            core::array::from_fn(|i| if i < n { Self::error(model, i, q[i], self.q_ref[i]) } else { 0.0 });
        let ed: VecN = core::array::from_fn(|i| if i < n { self.qd_ref[i] - qd[i] } else { 0.0 });
        let mut u = [0.0; MAXN];
        self.phase = PHASE_IDLE;
        match mode {
            PD | PD_GRAVITY => {
                for (a, &i) in model.actuated.iter().enumerate() {
                    u[a] = self.kp[i] * e[i] + self.kd[i] * ed[i];
                }
                if mode == PD_GRAVITY {
                    let mut dv = [0.0; MAXN];
                    model.gravity.eval(q, qd, &mut dv);
                    let (ug, _) = inverse_dynamics(model, ctx.k, &dv);
                    for a in 0..nu {
                        u[a] += ug[a];
                    }
                }
            }
            COMPUTED_TORQUE if self.ct_ok => {
                // task: q̈_a = v_a on the actuated coordinates; with loops the rest follows from A q̈ + Ȧ q̇ = 0
                let (kp, kv) = (self.ct_wn * self.ct_wn, 2.0 * self.ct_wn);
                let v: VecN = core::array::from_fn(|i| self.qdd_ref[i] + kv * ed[i] + kp * e[i]);
                let m = ctx.k.map_or(0, |k| k.m);
                let mut g = [0.0; MAXK * MAXN];
                let mut b = [0.0; MAXK];
                if let Some(k) = ctx.k {
                    g[..m * n].copy_from_slice(&k.a[..m * n]);
                    for r in 0..m {
                        b[r] = -k.adq[r];
                    }
                }
                for (a, &i) in model.actuated.iter().enumerate() {
                    g[(m + a) * n + i] = 1.0;
                    b[m + a] = v[i];
                }
                let (x, _) = lstsq(m + nu, n, &g, &b);
                let qdd: VecN = core::array::from_fn(|i| x[i]);
                let mqdd = mat_vec(n, &t.m, &qdd);
                let h = t.h(n, qd);
                let rhs: VecN = core::array::from_fn(|i| mqdd[i] + h[i]);
                (u, _) = inverse_dynamics(model, ctx.k, &rhs);
            }
            LQR if self.lqr.is_some() => {
                self.lqr_law(model, q, qd, &mut u);
                self.phase = PHASE_BALANCE;
            }
            SWING_UP if nu > 0 && !self.ct_ok => self.swing_up(model, ctx, &mut u),
            _ => {}
        }
        u
    }

    fn swing_up(&mut self, model: &Model, ctx: &Ctx<'_>, u: &mut VecN) {
        let (q, qd) = (ctx.q, ctx.qd);
        let watched: Vec<usize> =
            if model.closed() { (0..model.n).collect() } else { model.unactuated.clone() };
        let err = |i: usize| Self::error(model, i, q[i], self.q_star[i]).abs();
        let near = watched.iter().all(|&i| err(i) < 0.35 && qd[i].abs() < 4.0);
        let far = watched.iter().any(|&i| err(i) > 0.7);
        if self.phase == PHASE_BALANCE && far || self.phase != PHASE_BALANCE && !near {
            self.phase = PHASE_SWING;
        } else if near && self.lqr.is_some() {
            self.phase = PHASE_BALANCE;
        }
        if self.phase == PHASE_BALANCE {
            self.lqr_law(model, q, qd, u);
        } else {
            self.pump(model, ctx, u);
        }
    }

    /// Energy pumping: choose the actuated acceleration v so the power the actuated motion feeds into the
    /// passive coordinates, −q̇_uᵀ M_ua v, drives E → E*; then realise v by collocated partial feedback
    /// linearisation, generalised to loops: solve
    /// `M q̈ + h + Aᵀλ = B u`, `A q̈ + Ȧ q̇ = 0`, `q̈_a = v` for (q̈_u, λ, u).
    fn pump(&self, model: &Model, ctx: &Ctx<'_>, u: &mut VecN) {
        let (q, qd, t) = (ctx.q, ctx.qd, ctx.terms);
        let n = model.n;
        let (act, un) = (&model.actuated, &model.unactuated);
        let at_rest = qd[..n].iter().all(|v| v.abs() < 1e-3);
        let mut v = [0.0; MAXN];
        for (a, &i) in act.iter().enumerate() {
            let w: f64 = -un.iter().map(|&k| t.m[k * n + i] * qd[k]).sum::<f64>();
            let pump = self.k_e * (-self.e_u / self.e_ref) * w / self.coupling;
            let centre = self.k_c * Self::error(model, i, q[i], self.q_star[i]) - self.k_v * qd[i];
            let mut va = pump + centre;
            if at_rest && self.e_u < -0.5 * self.e_ref {
                va += 0.5 * self.a_max; // break the symmetry when resting at the bottom
            }
            v[a] = va.clamp(-self.a_max, self.a_max);
        }
        // unknowns x = (q̈_u, λ, u); rows: n dynamics + m constraints
        let h = t.h(n, qd);
        let m = ctx.k.map_or(0, |k| k.m);
        let (nuu, nu) = (un.len(), act.len());
        let (rows, cols) = (n + m, nuu + m + nu);
        let mut g = [0.0; MAXK * (MAXN + MAXK)];
        let mut rhs = [0.0; MAXK];
        let a_of = |r: usize, j: usize| ctx.k.map_or(0.0, |k| k.a[r * n + j]);
        for i in 0..n {
            for (c, &j) in un.iter().enumerate() {
                g[i * cols + c] = t.m[i * n + j];
            }
            for r in 0..m {
                g[i * cols + nuu + r] = a_of(r, i);
            }
            for a in 0..nu {
                g[i * cols + nuu + m + a] = -model.b[i * nu + a];
            }
            rhs[i] = -h[i] - act.iter().enumerate().map(|(a, &j)| t.m[i * n + j] * v[a]).sum::<f64>();
        }
        for r in 0..m {
            for (c, &j) in un.iter().enumerate() {
                g[(n + r) * cols + c] = a_of(r, j);
            }
            rhs[n + r] = -ctx.k.map_or(0.0, |k| k.adq[r])
                - act.iter().enumerate().map(|(a, &j)| a_of(r, j) * v[a]).sum::<f64>();
        }
        let (x, _) = lstsq(rows, cols, &g, &rhs);
        for a in 0..nu {
            u[a] = x[nuu + m + a];
        }
    }
}
