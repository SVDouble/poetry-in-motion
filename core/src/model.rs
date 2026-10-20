//! A built mechanism: the symbolic derivation compiled into numeric tapes plus everything the simulator
//! and controllers need to know about it.

use crate::MAXN;
use crate::cas::{Angle, Expr};
use crate::doc;
use crate::mech::{BodyDesc, Derived, Desc, JointKind, Vars, derive};
use crate::tape::Tape;

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
    pub len: usize,
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
