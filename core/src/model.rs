//! A built mechanism: the symbolic derivation compiled into numeric tapes plus everything the simulator
//! and controllers need to know about it.

use crate::MAXN;
use crate::cas::{Angle, Expr};
use crate::doc;
use crate::dynamics::{Terms, VecN};
use crate::mech::{BodyDesc, Derived, Desc, JointKind, MAXE, Vars, derive};
use crate::tape::Tape;

/// What the springs and dampers do at one state.
#[derive(Clone, Copy, Debug)]
pub struct ElementForces {
    /// Generalised forces of the springs (−∂V/∂q) and of the dampers.
    pub spring: VecN,
    pub damper: VecN,
    /// Energy stored in the springs, ½ Σ k (ℓ − ℓ₀)², and power lost in the dampers, Σ d ℓ̇².
    pub energy: f64,
    pub dissipation: f64,
    /// Force along each element, k (ℓ − ℓ₀) + c ℓ̇.
    pub tension: [f64; MAXE],
}

#[derive(Clone, Debug)]
pub struct Model {
    pub desc: Desc,
    pub vars: Vars,
    pub n: usize,
    /// Body moved by each coordinate.
    pub coord_body: Vec<usize>,
    /// Per coordinate: initial value, target, prismatic range (0 = none) and prismatic flag.
    pub q0: Vec<f64>,
    pub q_target: Vec<f64>,
    pub limit: Vec<f64>,
    pub prismatic: Vec<bool>,
    pub nu: usize,
    pub revolute: Vec<bool>,
    /// Per coordinate: the joint lies beyond the tool (its column of the tool Jacobian is identically zero,
    /// like a gripper's fingers), so no task of the tool can use or be disturbed by it.
    pub beyond_tool: Vec<bool>,
    /// Indices of the actuated joints; `B` selects them (`τ = B u`).
    pub actuated: Vec<usize>,
    pub unactuated: Vec<usize>,
    pub b: Vec<f64>,
    /// Parameter vector `[m₁ l₁ r₁ I₁ β₁ … g]` from the description.
    pub defaults: Vec<f64>,
    pub beta_index: Vec<usize>,
    /// Orientation of the frames `s, 1 … nb, t` (linear in q).
    pub frame_angles: Vec<Angle>,
    /// M (n²), C (n²), N (n).
    pub dynamics: Tape,
    /// Ṁ (n²), ∂V/∂q (n), ∂N/∂q (n²), T, V, frame positions (2 (n + 2)), COMs (2n), tool p (2),
    /// ∂p/∂q (2n), Jˢ (3n).
    pub aux: Tape,
    pub aux_off: AuxOffsets,
    /// ∂V/∂q (n) — gravity compensation.
    pub gravity: Tape,
    /// T, V.
    pub energy: Tape,
    /// Tool position p (2) and ∂p/∂q (2n) — inverse kinematics.
    pub tool: Option<Tape>,
    /// Positions of the frames `s, 1 … nb, t` (2 each): points that secondary tasks can steer.
    pub points: Tape,
    /// Loop closures: φ (m), A (m × n), Ȧ q̇ (m); `m = 2 · closures`.
    pub constraints: Option<Tape>,
    /// Springs and dampers: ℓ² (e), then ∂ℓ²/∂q (e × n).
    pub elements: Option<Tape>,
    /// (k, ℓ₀, d) of each spring/damper.
    pub element_values: Vec<[f64; 3]>,
    /// Loop closures, and the number of scalar constraint equations they give.
    pub nc: usize,
    pub ncon: usize,
    /// LaTeX derivation as JSON (see `doc`).
    pub derivation: String,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct AuxOffsets {
    pub mdot: usize,
    pub dv: usize,
    pub dndq: usize,
    pub energy: usize,
    pub frames: usize,
    pub coms: usize,
    pub tool_p: usize,
    pub jp: usize,
    pub js: usize,
    /// Centre-of-mass velocities (2 nb), their Jacobians ∂v/∂q̇ (2 nb × n, row-major) and ∂v/∂q · q̇ (2 nb).
    pub vcom: usize,
    pub jcom: usize,
    pub dvcom: usize,
    pub len: usize,
}

/// Centre-of-mass kinematics for a = J q̈ + (∂v/∂q) q̇: the Jacobians ∂v/∂q̇ (2 nb × n, row-major) and the
/// terms (∂v/∂q) q̇ (2 nb).
fn com_kinematics(d: &Derived) -> (Vec<Expr>, Vec<Expr>) {
    let (n, vars) = (d.n(), d.vars);
    let s = vars.space();
    let vcom: Vec<&Expr> = d.bodies.iter().flat_map(|b| [&b.vcom[0], &b.vcom[1]]).collect();
    let jcom = vcom.iter().flat_map(|v| (0..n).map(|j| v.diff_var(vars.qd(j)))).collect();
    let dvcom = vcom
        .iter()
        .map(|v| (0..n).fold(s.zero(), |acc, j| acc.add(&d.dq(v, j).mul(&s.var(vars.qd(j))))))
        .collect();
    (jcom, dvcom)
}

fn push<'a>(field: &mut usize, xs: &[&'a Expr], all: &mut Vec<&'a Expr>) {
    *field = all.len();
    all.extend_from_slice(xs);
}

impl Model {
    /// Derive and compile a mechanism.
    ///
    /// # Errors
    /// If the description is invalid.
    pub fn build(flat: &[f64]) -> Result<Self, String> {
        Ok(Self::from_desc(&Desc::parse(flat)?))
    }

    #[must_use]
    pub fn from_desc(desc: &Desc) -> Self {
        let d = derive(desc);
        Self::compile(&d)
    }

    /// # Panics
    /// If the mechanism has more than [`MAXN`] joints (`Desc::parse` rejects those).
    #[must_use]
    pub fn compile(d: &Derived) -> Self {
        let n = d.n();
        assert!(n <= MAXN);
        let vars = d.vars;
        let s = vars.space();
        let zero = s.zero();

        let dyn_exprs: Vec<&Expr> = d.m.iter().chain(&d.c).chain(&d.nvec).collect();

        // frames: s, bodies, tool (tool frame falls back to the last body)
        let s_origin = [s.zero(), s.zero()];
        let tool_pose = d.tool.as_ref().map(|t| &t.pose);
        let mut frame_xy: Vec<&Expr> = vec![&s_origin[0], &s_origin[1]];
        let mut frame_angles = vec![Angle::zero(n)];
        for b in &d.bodies {
            frame_xy.extend([&b.world.p[0], &b.world.p[1]]);
            frame_angles.push(b.world.phi.clone());
        }
        let last = &d.bodies[d.bodies.len() - 1].world;
        let tp = tool_pose.unwrap_or(last);
        frame_xy.extend([&tp.p[0], &tp.p[1]]);
        frame_angles.push(tp.phi.clone());

        let coms: Vec<&Expr> = d.bodies.iter().flat_map(|b| [&b.com[0], &b.com[1]]).collect();
        let (tool_p, jp, js): (Vec<&Expr>, Vec<&Expr>, Vec<&Expr>) = d.tool.as_ref().map_or_else(
            || (vec![&zero; 2], vec![&zero; 2 * n], vec![&zero; 3 * n]),
            |t| (vec![&t.pose.p[0], &t.pose.p[1]], t.jp.iter().collect(), t.js.iter().collect()),
        );

        let mut aux_exprs: Vec<&Expr> = Vec::new();
        let mut off = AuxOffsets::default();
        let mdot: Vec<&Expr> = d.mdot.iter().collect();
        let dv: Vec<&Expr> = d.dv.iter().collect();
        let dndq: Vec<&Expr> = d.dndq.iter().collect();
        push(&mut off.mdot, &mdot, &mut aux_exprs);
        push(&mut off.dv, &dv, &mut aux_exprs);
        push(&mut off.dndq, &dndq, &mut aux_exprs);
        push(&mut off.energy, &[&d.t, &d.v], &mut aux_exprs);
        push(&mut off.frames, &frame_xy, &mut aux_exprs);
        push(&mut off.coms, &coms, &mut aux_exprs);
        push(&mut off.tool_p, &tool_p, &mut aux_exprs);
        push(&mut off.jp, &jp, &mut aux_exprs);
        push(&mut off.js, &js, &mut aux_exprs);
        let vcom: Vec<&Expr> = d.bodies.iter().flat_map(|b| [&b.vcom[0], &b.vcom[1]]).collect();
        let (jcom, dvcom) = com_kinematics(d);
        push(&mut off.vcom, &vcom, &mut aux_exprs);
        push(&mut off.jcom, &jcom.iter().collect::<Vec<_>>(), &mut aux_exprs);
        push(&mut off.dvcom, &dvcom.iter().collect::<Vec<_>>(), &mut aux_exprs);
        off.len = aux_exprs.len();

        let points = Tape::compile(vars, &frame_xy);
        let tool = d.tool.as_ref().map(|t| {
            let xs: Vec<&Expr> = [&t.pose.p[0], &t.pose.p[1]].into_iter().chain(&t.jp).collect();
            Tape::compile(vars, &xs)
        });

        let constraints = (!d.phi.is_empty()).then(|| {
            let xs: Vec<&Expr> = d.phi.iter().chain(&d.a_mat).chain(&d.adot_qd).collect();
            Tape::compile(vars, &xs)
        });
        let elements = (!d.element_l2.is_empty()).then(|| {
            let xs: Vec<&Expr> = d.element_l2.iter().chain(&d.element_dl2).collect();
            Tape::compile(vars, &xs)
        });
        let defaults: Vec<f64> =
            d.desc.bodies.iter().flat_map(|b| b.values).chain(core::iter::once(d.desc.g)).collect();
        let nu = d.actuated.len();
        let per_coord = |f: fn(&BodyDesc) -> f64| -> Vec<f64> {
            d.coord_body.iter().map(|&i| f(&d.desc.bodies[i])).collect()
        };
        let mut model = Self {
            desc: d.desc.clone(),
            vars,
            n,
            coord_body: d.coord_body.clone(),
            q0: per_coord(|b| b.q0),
            q_target: per_coord(|b| b.q_target),
            limit: per_coord(|b| if b.kind == JointKind::Prismatic { b.limit } else { 0.0 }),
            prismatic: d.coord_body.iter().map(|&i| d.desc.bodies[i].kind == JointKind::Prismatic).collect(),
            nu,
            revolute: d.revolute.clone(),
            beyond_tool: (0..n)
                .map(|j| d.tool.as_ref().is_some_and(|t| t.jp[j].is_zero() && t.jp[n + j].is_zero()))
                .collect(),
            actuated: d.actuated.clone(),
            unactuated: (0..n).filter(|i| !d.actuated.contains(i)).collect(),
            b: d.b.clone(),
            beta_index: d.coord_body.iter().map(|&i| vars.beta(i)).collect(),
            frame_angles,
            dynamics: Tape::compile(vars, &dyn_exprs),
            aux: Tape::compile(vars, &aux_exprs),
            aux_off: off,
            gravity: Tape::compile(vars, &dv),
            energy: Tape::compile(vars, &[&d.t, &d.v]),
            tool,
            points,
            constraints,
            elements,
            element_values: d.desc.elements.iter().map(|e| [e.k, e.l0, e.d]).collect(),
            nc: d.desc.closures.len(),
            ncon: d.phi.len(),
            derivation: doc::json(d),
            defaults,
        };
        model.set_params(&model.defaults.clone());
        model
    }

    pub fn set_params(&mut self, p: &[f64]) {
        self.dynamics.set_params(p);
        self.aux.set_params(p);
        self.gravity.set_params(p);
        self.energy.set_params(p);
        self.points.set_params(p);
        if let Some(t) = &mut self.tool {
            t.set_params(p);
        }
        if let Some(t) = &mut self.constraints {
            t.set_params(p);
        }
        if let Some(t) = &mut self.elements {
            t.set_params(p);
        }
    }

    /// Forces, stored energy and losses of the springs and dampers at a state.
    pub fn element_forces(&mut self, q: &[f64], qd: &[f64]) -> ElementForces {
        let n = self.n;
        let mut f = ElementForces {
            spring: [0.0; MAXN],
            damper: [0.0; MAXN],
            energy: 0.0,
            dissipation: 0.0,
            tension: [0.0; MAXE],
        };
        let Some(tape) = &mut self.elements else { return f };
        let ne = self.element_values.len();
        let mut buf = [0.0; MAXE * (MAXN + 1)];
        tape.eval(q, qd, &mut buf);
        for (e, &[k, l0, d]) in self.element_values.iter().enumerate() {
            let l = buf[e].max(0.0).sqrt();
            let grad = &buf[ne + e * n..ne + (e + 1) * n];
            // ∂ℓ/∂q = (∂ℓ²/∂q) / 2ℓ; at ℓ = 0 the direction is undefined and no force acts
            if l < 1e-9 {
                f.energy += 0.5 * k * l0 * l0;
                continue;
            }
            let dl = |j: usize| grad[j] / (2.0 * l);
            let rate: f64 = (0..n).map(|j| dl(j) * qd[j]).sum();
            let (fs, fd) = (k * (l - l0), d * rate);
            f.tension[e] = fs + fd;
            for j in 0..n {
                f.spring[j] -= fs * dl(j);
                f.damper[j] -= fd * dl(j);
            }
            f.energy += 0.5 * k * (l - l0) * (l - l0);
            f.dissipation += d * rate * rate;
        }
        f
    }

    /// M, C and N at a state, with the springs and dampers in N (`N = ∂V/∂q + β q̇ − Q_springs − Q_dampers`).
    pub fn terms(&mut self, q: &[f64], qd: &[f64]) -> Terms {
        let mut t = Terms::eval(&mut self.dynamics, self.n, q, qd);
        if self.elements.is_some() {
            let f = self.element_forces(q, qd);
            for i in 0..self.n {
                t.n[i] -= f.spring[i] + f.damper[i];
            }
        }
        t
    }

    /// ∂V/∂q: gravity and the springs (what gravity compensation cancels).
    pub fn potential_grad(&mut self, q: &[f64]) -> VecN {
        let zero = [0.0; MAXN];
        let mut dv = [0.0; MAXN];
        self.gravity.eval(q, &zero, &mut dv);
        if self.elements.is_some() {
            let f = self.element_forces(q, &zero);
            for i in 0..self.n {
                dv[i] -= f.spring[i];
            }
        }
        dv
    }

    /// `[T, V]` with the energy stored in the springs.
    pub fn energies(&mut self, q: &[f64], qd: &[f64]) -> [f64; 2] {
        let mut e = [0.0; 2];
        self.energy.eval(q, qd, &mut e);
        if self.elements.is_some() {
            e[1] += self.element_forces(q, qd).energy;
        }
        e
    }

    /// Does the mechanism contain closed kinematic loops?
    #[must_use]
    pub fn closed(&self) -> bool {
        self.nc > 0
    }

    #[must_use]
    pub fn has_tool(&self) -> bool {
        self.tool.is_some()
    }

    #[must_use]
    pub fn fully_actuated(&self) -> bool {
        self.nu == self.n
    }

    #[must_use]
    pub fn is_prismatic(&self, j: usize) -> bool {
        self.prismatic[j]
    }
}
