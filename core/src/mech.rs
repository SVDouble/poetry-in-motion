//! Generic planar multibody description and its symbolic Lagrangian derivation (MLS ch. 4).
//!
//! A mechanism is a tree of bodies. Body `i` hangs from its parent (or the ground) by joint `i`, which is
//! revolute (coordinate `θᵢ`), prismatic (`dᵢ`) or fixed (a weld, no coordinate — walls, brackets, frames).
//! Each body carries symbolic coefficients `mᵢ, lᵢ, rᵢ, Iᵢ, βᵢ` (mass, length, centre-of-mass offset along
//! the body x-axis, inertia about the COM, joint damping). A joint sits at a fraction `s` of its parent's
//! length (`s · l_parent`, kept symbolic) and is rotated by a numeric mounting angle.
//!
//! Springs and dampers join two points (on bodies or the ground). Their length ℓ is a square root, which
//! the Fourier-canonical algebra cannot hold, so ℓ² and ∂ℓ²/∂q are derived symbolically and the force
//! `−(k (ℓ − ℓ₀) + d ℓ̇) ∂ℓ/∂q` with `∂ℓ/∂q = (∂ℓ²/∂q) / 2ℓ` is formed numerically.

use crate::cas::{Angle, Expr, Space};

pub const DESC_VERSION: f64 = 4.0;
/// Maximum number of bodies (fixed ones included).
pub const MAXB: usize = 8;
/// version, bodies, g, tool, closures, elements.
pub const HEADER: usize = 6;
/// Fields per loop closure: kind, a, s_a, b, s_b, ground x, ground y, slot angle.
pub const PER_CLOSURE: usize = 8;
/// Maximum number of loop closures.
pub const MAXC: usize = 4;
/// Maximum number of scalar constraint equations (two per pin).
pub const MAXM: usize = 2 * MAXC;
/// Fields per spring/damper: a, s_a, b, s_b, ground x, ground y, k, ℓ₀, d.
pub const PER_ELEMENT: usize = 9;
/// Maximum number of springs and dampers.
pub const MAXE: usize = 8;
pub const PER_BODY: usize = 18;
/// Coefficients per body: m, l, r, I, β.
pub const PARAMS_PER_BODY: usize = 5;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum JointKind {
    Revolute,
    Prismatic,
    /// Welded to the parent: no coordinate.
    Fixed,
}

#[derive(Clone, Debug, PartialEq)]
pub struct BodyDesc {
    pub parent: Option<usize>,
    pub kind: JointKind,
    /// Position of the joint along the parent body, as a fraction of its length: `(s · l_parent, 0)`.
    pub mount_s: f64,
    /// Joint position for bodies attached to the ground (numeric).
    pub mount_xy: [f64; 2],
    /// Rotation of the joint frame relative to the parent body frame.
    pub mount_angle: f64,
    /// Prismatic sliding direction in the joint frame.
    pub axis_angle: f64,
    pub actuated: bool,
    /// Numeric values of m, l, r, I, β.
    pub values: [f64; PARAMS_PER_BODY],
    pub q0: f64,
    pub q_target: f64,
    /// Half range of a prismatic joint (end stops); 0 = unlimited.
    pub limit: f64,
    /// 0 = slender link, 1 = block (cart).
    pub shape: u8,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ClosureKind {
    /// Revolute pin: the two points coincide (two constraints).
    Pin,
    /// Pin in a slot: point b slides along the line through point a with direction `angle` in the frame
    /// of body a (or the ground) — one constraint.
    Slot { angle: f64 },
}

/// A loop closure between the point `(s_a · l_a, 0)` of body `a` (or a fixed ground point) and the point
/// `(s_b · l_b, 0)` of body `b`, giving holonomic constraints φ(q) = 0 (MLS ch. 6).
#[derive(Clone, Debug, PartialEq)]
pub struct Closure {
    pub kind: ClosureKind,
    pub a: Option<usize>,
    pub s_a: f64,
    pub b: usize,
    pub s_b: f64,
    /// Ground point when `a` is `None`.
    pub ground: [f64; 2],
}

/// A linear spring and/or viscous damper.
///
/// It joins the point `(s_a · l_a, 0)` of body `a` (or a fixed ground point) and the point `(s_b · l_b, 0)` of
/// body `b`; the force along the line between them is `k (ℓ − ℓ₀) + d ℓ̇` (k = 0: a pure damper, d = 0: a
/// pure spring).
#[derive(Clone, Debug, PartialEq)]
pub struct Element {
    pub a: Option<usize>,
    pub s_a: f64,
    pub b: usize,
    pub s_b: f64,
    /// Ground point when `a` is `None`.
    pub ground: [f64; 2],
    pub k: f64,
    pub l0: f64,
    pub d: f64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Desc {
    pub bodies: Vec<BodyDesc>,
    pub g: f64,
    pub tool: Option<usize>,
    pub closures: Vec<Closure>,
    pub elements: Vec<Element>,
}

impl Desc {
    /// Parse the flat `f64` description written by the browser (see `web/src/model/description.ts`).
    ///
    /// # Errors
    /// Describes the first inconsistency (version, sizes, parents that are not earlier bodies).
    pub fn parse(flat: &[f64]) -> Result<Self, String> {
        let hdr = flat.get(..HEADER).ok_or("description too short")?;
        if (hdr[0] - DESC_VERSION).abs() > 0.0 {
            return Err(format!("unsupported description version {}", hdr[0]));
        }
        let n = hdr[1] as usize;
        if n == 0 || n > MAXB {
            return Err(format!("between 1 and {MAXB} bodies are supported (got {n})"));
        }
        if flat.len() < HEADER + n * PER_BODY {
            return Err("description truncated".into());
        }
        let mut bodies = Vec::with_capacity(n);
        for i in 0..n {
            let b = &flat[HEADER + i * PER_BODY..HEADER + (i + 1) * PER_BODY];
            let parent = if b[0] < 0.0 { None } else { Some(b[0] as usize) };
            if parent.is_some_and(|p| p >= i) {
                return Err(format!("body {} must hang from an earlier body", i + 1));
            }
            let kind = match b[1].round() as i64 {
                1 => JointKind::Prismatic,
                2 => JointKind::Fixed,
                _ => JointKind::Revolute,
            };
            bodies.push(BodyDesc {
                parent,
                kind,
                mount_s: b[2],
                mount_xy: [b[3], b[4]],
                mount_angle: b[5],
                axis_angle: b[6],
                actuated: b[7] != 0.0 && kind != JointKind::Fixed,
                values: [b[8], b[9], b[10], b[11], b[12]],
                q0: b[13],
                q_target: b[14],
                limit: b[15],
                shape: b[16] as u8,
            });
        }
        let tool = if hdr[3] < 0.0 { None } else { Some((hdr[3] as usize).min(n - 1)) };
        let nc = hdr[4] as usize;
        if nc > MAXC {
            return Err(format!("at most {MAXC} loop closures are supported"));
        }
        let base = HEADER + n * PER_BODY;
        let flat_c = flat.get(base..base + nc * PER_CLOSURE).ok_or("description truncated (closures)")?;
        let mut closures = Vec::with_capacity(nc);
        for c in flat_c.chunks(PER_CLOSURE) {
            let kind =
                if c[0].round() as i64 == 1 { ClosureKind::Slot { angle: c[7] } } else { ClosureKind::Pin };
            let a = if c[1] < 0.0 { None } else { Some(c[1] as usize) };
            let b = c[3] as usize;
            if a.is_some_and(|a| a >= n) || b >= n || a == Some(b) {
                return Err("a loop closure must join two different existing bodies".into());
            }
            closures.push(Closure { kind, a, s_a: c[2], b, s_b: c[4], ground: [c[5], c[6]] });
        }
        let ne = hdr[5] as usize;
        if ne > MAXE {
            return Err(format!("at most {MAXE} springs and dampers are supported"));
        }
        let base = base + nc * PER_CLOSURE;
        let flat_e = flat.get(base..base + ne * PER_ELEMENT).ok_or("description truncated (springs)")?;
        let mut elements = Vec::with_capacity(ne);
        for e in flat_e.chunks(PER_ELEMENT) {
            let a = if e[0] < 0.0 { None } else { Some(e[0] as usize) };
            let b = e[2] as usize;
            if a.is_some_and(|a| a >= n) || b >= n || a == Some(b) {
                return Err(
                    "a spring or damper must join two different bodies, or a body and the ground".into()
                );
            }
            let (k, l0, d) = (e[6].max(0.0), e[7].max(0.0), e[8].max(0.0));
            elements.push(Element { a, s_a: e[1], b, s_b: e[3], ground: [e[4], e[5]], k, l0, d });
        }
        let d = Self { bodies, g: hdr[2], tool, closures, elements };
        match d.n() {
            0 => Err("the mechanism has no moving joint: everything is fixed".into()),
            k if k > crate::MAXN => {
                Err(format!("at most {} moving joints are supported (got {k})", crate::MAXN))
            }
            _ => Ok(d),
        }
    }

    /// Number of generalised coordinates (moving joints).
    #[must_use]
    pub fn n(&self) -> usize {
        self.bodies.iter().filter(|b| b.kind != JointKind::Fixed).count()
    }

    /// Number of bodies (fixed ones included).
    #[must_use]
    pub fn nb(&self) -> usize {
        self.bodies.len()
    }

    /// Body moved by each coordinate.
    #[must_use]
    pub fn coord_bodies(&self) -> Vec<usize> {
        (0..self.nb()).filter(|&i| self.bodies[i].kind != JointKind::Fixed).collect()
    }
}

/// Index map of the polynomial variables: `[m₁ l₁ r₁ I₁ β₁ … (per body) g | q | q̇ | q̈ (per coordinate)]`.
#[derive(Clone, Copy, Debug)]
pub struct Vars {
    /// Coordinates.
    pub n: usize,
    /// Bodies.
    pub nb: usize,
}

impl Vars {
    #[must_use]
    pub const fn np(self) -> usize {
        PARAMS_PER_BODY * self.nb + 1
    }
    #[must_use]
    pub const fn m(self, i: usize) -> usize {
        PARAMS_PER_BODY * i
    }
    #[must_use]
    pub const fn l(self, i: usize) -> usize {
        PARAMS_PER_BODY * i + 1
    }
    #[must_use]
    pub const fn r(self, i: usize) -> usize {
        PARAMS_PER_BODY * i + 2
    }
    #[must_use]
    pub const fn inertia(self, i: usize) -> usize {
        PARAMS_PER_BODY * i + 3
    }
    #[must_use]
    pub const fn beta(self, i: usize) -> usize {
        PARAMS_PER_BODY * i + 4
    }
    #[must_use]
    pub const fn g(self) -> usize {
        PARAMS_PER_BODY * self.nb
    }
    #[must_use]
    pub const fn q(self, j: usize) -> usize {
        self.np() + j
    }
    #[must_use]
    pub const fn qd(self, j: usize) -> usize {
        self.np() + self.n + j
    }
    #[must_use]
    pub const fn qdd(self, j: usize) -> usize {
        self.np() + 2 * self.n + j
    }
    #[must_use]
    pub const fn nvars(self) -> usize {
        self.np() + 3 * self.n
    }
    #[must_use]
    pub const fn space(self) -> Space {
        Space { nvars: self.nvars(), nang: self.n }
    }
}

/// Planar pose: position and orientation (`Angle` is linear in the revolute coordinates).
#[derive(Clone, Debug)]
pub struct Pose {
    pub p: [Expr; 2],
    pub phi: Angle,
}

#[derive(Clone, Debug)]
pub struct BodyDerived {
    /// g_{parent, i}: translation and rotation of body i relative to its parent body.
    pub local: Pose,
    /// g_{s i}
    pub world: Pose,
    pub com: [Expr; 2],
    pub vcom: [Expr; 2],
    pub omega: Expr,
    pub t: Expr,
    pub v: Expr,
}

#[derive(Clone, Debug)]
pub struct ToolDerived {
    pub body: usize,
    pub pose: Pose,
    /// ∂p_st/∂q, row-major 2 × n.
    pub jp: Vec<Expr>,
    /// Spatial manipulator Jacobian (v_x, v_y, ω), row-major 3 × n.
    pub js: Vec<Expr>,
    /// Joint twists ξᵢ = Jˢ(0), row-major 3 × n.
    pub twists: Vec<Expr>,
    /// g_st(0): position (with q = 0) and angle offset.
    pub p0: [Expr; 2],
    pub phi0: f64,
}

#[derive(Clone, Debug)]
pub struct Derived {
    pub desc: Desc,
    pub vars: Vars,
    /// Per coordinate.
    pub revolute: Vec<bool>,
    /// Body moved by each coordinate, and the coordinate of each body (`None` for fixed joints).
    pub coord_body: Vec<usize>,
    pub body_coord: Vec<Option<usize>>,
    pub bodies: Vec<BodyDerived>,
    pub t: Expr,
    pub v: Expr,
    /// Row-major n × n.
    pub m: Vec<Expr>,
    /// Christoffel symbols Γ_ijk, index `(i * n + j) * n + k`.
    pub gamma: Vec<Expr>,
    pub c: Vec<Expr>,
    pub dv: Vec<Expr>,
    pub nvec: Vec<Expr>,
    pub mdot: Vec<Expr>,
    pub dndq: Vec<Expr>,
    /// Scalar equations of motion: Σⱼ Mᵢⱼ q̈ⱼ + (C q̇)ᵢ + Nᵢ.
    pub eom: Vec<Expr>,
    /// Loop closures: the two joined points, the constraints φ(q) (pin: p_a − p_b; slot: nᵀ(p_b − p_a)),
    /// the constraint Jacobian A = ∂φ/∂q (row-major m × n) and Ȧ q̇.
    pub closure_points: Vec<[[Expr; 2]; 2]>,
    pub phi: Vec<Expr>,
    pub a_mat: Vec<Expr>,
    pub adot_qd: Vec<Expr>,
    /// Springs and dampers: the two joined points, ℓ² and ∂ℓ²/∂q (row-major e × n).
    pub element_points: Vec<[[Expr; 2]; 2]>,
    pub element_l2: Vec<Expr>,
    pub element_dl2: Vec<Expr>,
    pub tool: Option<ToolDerived>,
    /// Actuation matrix B (row-major n × nu): selection of the actuated coordinates.
    pub b: Vec<f64>,
    /// Actuated coordinates.
    pub actuated: Vec<usize>,
}

impl Derived {
    #[must_use]
    pub fn n(&self) -> usize {
        self.desc.n()
    }

    /// ∂/∂q_j (angle derivative for revolute joints, polynomial derivative for prismatic ones).
    #[must_use]
    pub fn dq(&self, e: &Expr, j: usize) -> Expr {
        partial_q(&self.revolute, self.vars, e, j)
    }
}

fn partial_q(revolute: &[bool], vars: Vars, e: &Expr, j: usize) -> Expr {
    if revolute[j] { e.diff_angle(j) } else { e.diff_var(vars.q(j)) }
}

/// d/dt of an expression that depends on q only: Σⱼ ∂e/∂qⱼ q̇ⱼ.
fn dt_q(revolute: &[bool], vars: Vars, e: &Expr) -> Expr {
    let s = e.space();
    (0..vars.n).fold(s.zero(), |acc, j| acc.add(&partial_q(revolute, vars, e, j).mul(&s.var(vars.qd(j)))))
}

/// Rotate `(x, y)` by the angle `a`.
fn rotate(s: &Space, a: &Angle, x: &Expr, y: &Expr) -> [Expr; 2] {
    let (c, sn) = (s.cos(a), s.sin(a));
    [c.mul(x).sub(&sn.mul(y)), sn.mul(x).add(&c.mul(y))]
}

/// Point `(x, y)` given in the frame `pose`, expressed in the spatial frame.
fn transform(s: &Space, pose: &Pose, x: &Expr, y: &Expr) -> [Expr; 2] {
    let [rx, ry] = rotate(s, &pose.phi, x, y);
    [pose.p[0].add(&rx), pose.p[1].add(&ry)]
}

#[must_use]
#[allow(clippy::too_many_lines)]
pub fn derive(desc: &Desc) -> Derived {
    let (n, nb) = (desc.n(), desc.nb());
    let vars = Vars { n, nb };
    let s = vars.space();
    let coord_body = desc.coord_bodies();
    let mut body_coord = vec![None; nb];
    for (j, &i) in coord_body.iter().enumerate() {
        body_coord[i] = Some(j);
    }
    let revolute: Vec<bool> =
        coord_body.iter().map(|&i| desc.bodies[i].kind == JointKind::Revolute).collect();
    let zero_angle = Angle::zero(n);

    // ---- kinematics: g_{s i} = g_{s, parent} · g_{parent, joint} · g_{joint, i}(qᵢ)
    let mut bodies: Vec<BodyDerived> = Vec::with_capacity(nb);
    for (i, b) in desc.bodies.iter().enumerate() {
        let (mount_x, mount_y) = match b.parent {
            Some(p) if b.mount_s.abs() > 1e-12 => (s.var(vars.l(p)).scale(b.mount_s), s.zero()),
            Some(_) => (s.zero(), s.zero()),
            None => (s.constant(b.mount_xy[0]), s.constant(b.mount_xy[1])),
        };
        let joint_rot = zero_angle.plus_const(b.mount_angle);
        // displacement and rotation produced by the joint itself, in the parent frame
        let (local_p, local_phi) = match (b.kind, body_coord[i]) {
            (JointKind::Revolute, Some(j)) => ([mount_x, mount_y], joint_rot.plus_joint(j)),
            (JointKind::Prismatic, Some(j)) => {
                let dir = joint_rot.plus_const(b.axis_angle);
                let d = s.var(vars.q(j));
                let off = [s.cos(&dir).mul(&d), s.sin(&dir).mul(&d)];
                ([mount_x.add(&off[0]), mount_y.add(&off[1])], joint_rot)
            }
            _ => ([mount_x, mount_y], joint_rot),
        };
        let local = Pose { p: local_p, phi: local_phi };
        let world = match b.parent {
            None => local.clone(),
            Some(p) => {
                let parent = &bodies[p].world;
                Pose { p: transform(&s, parent, &local.p[0], &local.p[1]), phi: parent.phi.add(&local.phi) }
            }
        };
        let com = transform(&s, &world, &s.var(vars.r(i)), &s.zero());
        let vcom = [dt_q(&revolute, vars, &com[0]), dt_q(&revolute, vars, &com[1])];
        let omega =
            (0..n).fold(s.zero(), |acc, j| acc.add(&s.var(vars.qd(j)).scale(f64::from(world.phi.k[j]))));
        let half = 0.5;
        let t = s
            .var(vars.m(i))
            .mul(&vcom[0].square().add(&vcom[1].square()))
            .add(&s.var(vars.inertia(i)).mul(&omega.square()))
            .scale(half);
        let v = s.var(vars.m(i)).mul(&s.var(vars.g())).mul(&com[1]);
        bodies.push(BodyDerived { local, world, com, vcom, omega, t, v });
    }

    // ---- energies and the manipulator equation
    let t = bodies.iter().fold(s.zero(), |acc, b| acc.add(&b.t));
    let v = bodies.iter().fold(s.zero(), |acc, b| acc.add(&b.v));
    let mut m = Vec::with_capacity(n * n);
    for i in 0..n {
        for j in 0..n {
            m.push(t.diff_var(vars.qd(i)).diff_var(vars.qd(j)));
        }
    }
    let dm: Vec<Vec<Expr>> =
        (0..n).map(|k| m.iter().map(|e| partial_q(&revolute, vars, e, k)).collect()).collect();
    let mut gamma = Vec::with_capacity(n * n * n);
    for i in 0..n {
        for j in 0..n {
            for k in 0..n {
                // MLS (4.22): Γᵢⱼₖ = ½ (∂Mᵢⱼ/∂qₖ + ∂Mᵢₖ/∂qⱼ − ∂Mₖⱼ/∂qᵢ)
                gamma.push(dm[k][i * n + j].add(&dm[j][i * n + k]).sub(&dm[i][k * n + j]).scale(0.5));
            }
        }
    }
    let c: Vec<Expr> = (0..n * n)
        .map(|ij| (0..n).fold(s.zero(), |acc, k| acc.add(&gamma[ij * n + k].mul(&s.var(vars.qd(k))))))
        .collect();
    let dv: Vec<Expr> = (0..n).map(|i| partial_q(&revolute, vars, &v, i)).collect();
    // viscous friction of the joint that drives coordinate i, i.e. of the body that coordinate moves
    let nvec: Vec<Expr> =
        (0..n).map(|i| dv[i].add(&s.var(vars.beta(coord_body[i])).mul(&s.var(vars.qd(i))))).collect();
    let mdot: Vec<Expr> = m.iter().map(|e| dt_q(&revolute, vars, e)).collect();
    let dndq: Vec<Expr> = (0..n * n).map(|ij| partial_q(&revolute, vars, &dv[ij / n], ij % n)).collect();
    let eom: Vec<Expr> = (0..n)
        .map(|i| {
            let mut e = nvec[i].clone();
            for j in 0..n {
                e = e.add(&m[i * n + j].mul(&s.var(vars.qdd(j)))).add(&c[i * n + j].mul(&s.var(vars.qd(j))));
            }
            e
        })
        .collect();

    // ---- tool point: tip of the chosen body
    let tool = desc.tool.map(|tb| {
        let w = &bodies[tb].world;
        let pose = Pose { p: transform(&s, w, &s.var(vars.l(tb)), &s.zero()), phi: w.phi.clone() };
        let _ = nb;
        let mut jp = Vec::with_capacity(2 * n);
        for row in 0..2 {
            for j in 0..n {
                jp.push(partial_q(&revolute, vars, &pose.p[row], j));
            }
        }
        let mut js = vec![s.zero(); 3 * n];
        for j in 0..n {
            let w_j = f64::from(pose.phi.k[j]);
            // V^s = (ṗ + ω p_y, ṗ_y − ω p_x, ω) column by column
            js[j] = jp[j].add(&pose.p[1].scale(w_j));
            js[n + j] = jp[n + j].sub(&pose.p[0].scale(w_j));
            js[2 * n + j] = s.constant(w_j);
        }
        let coords: Vec<usize> = (0..n).map(|j| vars.q(j)).collect();
        let twists = js.iter().map(|e| e.at_zero(&coords)).collect();
        let p0 = [pose.p[0].at_zero(&coords), pose.p[1].at_zero(&coords)];
        let phi0 = pose.phi.c;
        ToolDerived { body: tb, pose, jp, js, twists, p0, phi0 }
    });

    // ---- loop closures: pins φ = p_a − p_b = 0, slots φ = nᵀ(p_b − p_a) = 0
    let point = |body: usize, frac: f64| -> [Expr; 2] {
        let x = if frac.abs() > 1e-12 { s.var(vars.l(body)).scale(frac) } else { s.zero() };
        transform(&s, &bodies[body].world, &x, &s.zero())
    };
    let mut closure_points = Vec::with_capacity(desc.closures.len());
    let mut phi = Vec::with_capacity(2 * desc.closures.len());
    for c in &desc.closures {
        let pa = c.a.map_or_else(|| [s.constant(c.ground[0]), s.constant(c.ground[1])], |a| point(a, c.s_a));
        let pb = point(c.b, c.s_b);
        match c.kind {
            ClosureKind::Pin => {
                phi.push(pa[0].sub(&pb[0]));
                phi.push(pa[1].sub(&pb[1]));
            }
            ClosureKind::Slot { angle } => {
                let dir =
                    c.a.map_or_else(|| zero_angle.clone(), |a| bodies[a].world.phi.clone()).plus_const(angle);
                // normal of the slot: (−sin, cos) of its direction
                let (nx, ny) = (s.sin(&dir).scale(-1.0), s.cos(&dir));
                phi.push(nx.mul(&pb[0].sub(&pa[0])).add(&ny.mul(&pb[1].sub(&pa[1]))));
            }
        }
        closure_points.push([pa, pb]);
    }
    let a_mat: Vec<Expr> =
        phi.iter().flat_map(|f| (0..n).map(|j| partial_q(&revolute, vars, f, j))).collect();
    let adot_qd: Vec<Expr> = (0..phi.len())
        .map(|r| {
            // Ȧ q̇ = Σ_k ∂(A q̇)/∂q_k q̇_k
            let aq = (0..n).fold(s.zero(), |acc, j| acc.add(&a_mat[r * n + j].mul(&s.var(vars.qd(j)))));
            dt_q(&revolute, vars, &aq)
        })
        .collect();

    // ---- springs and dampers: ℓ² = |p_b − p_a|² and its gradient (the force is formed numerically)
    let mut element_points = Vec::with_capacity(desc.elements.len());
    let mut element_l2 = Vec::with_capacity(desc.elements.len());
    for e in &desc.elements {
        let pa = e.a.map_or_else(|| [s.constant(e.ground[0]), s.constant(e.ground[1])], |a| point(a, e.s_a));
        let pb = point(e.b, e.s_b);
        let (dx, dy) = (pb[0].sub(&pa[0]), pb[1].sub(&pa[1]));
        element_l2.push(dx.square().add(&dy.square()));
        element_points.push([pa, pb]);
    }
    let element_dl2: Vec<Expr> =
        element_l2.iter().flat_map(|l2| (0..n).map(|j| partial_q(&revolute, vars, l2, j))).collect();

    let actuated: Vec<usize> = (0..n).filter(|&j| desc.bodies[coord_body[j]].actuated).collect();
    let mut b = vec![0.0; n * actuated.len()];
    for (col, &i) in actuated.iter().enumerate() {
        b[i * actuated.len() + col] = 1.0;
    }

    Derived {
        desc: desc.clone(),
        vars,
        revolute,
        coord_body,
        body_coord,
        bodies,
        t,
        v,
        m,
        gamma,
        c,
        dv,
        nvec,
        mdot,
        dndq,
        eom,
        closure_points,
        phi,
        a_mat,
        adot_qd,
        element_points,
        element_l2,
        element_dl2,
        tool,
        b,
        actuated,
    }
}

/// Built-in descriptions (mirrored by the browser presets) used by tests and as defaults.
pub mod presets {
    use core::f64::consts::{FRAC_PI_2, PI};

    use super::{BodyDesc, Closure, ClosureKind, Desc, JointKind};

    #[must_use]
    pub fn link(parent: Option<usize>, kind: JointKind, m: f64, l: f64) -> BodyDesc {
        BodyDesc {
            parent,
            kind,
            mount_s: if parent.is_some() { 1.0 } else { 0.0 },
            mount_xy: [0.0; 2],
            mount_angle: 0.0,
            axis_angle: 0.0,
            actuated: true,
            values: [m, l, l / 2.0, m * l * l / 12.0, 0.05],
            q0: 0.0,
            q_target: 0.0,
            limit: 0.0,
            shape: 0,
        }
    }

    /// Cart on a horizontal rail with a pole hinged on top (θ = 0 upright, counter-clockwise positive).
    #[must_use]
    pub fn cartpole() -> Desc {
        let mut cart = link(None, JointKind::Prismatic, 1.0, 0.0);
        cart.values = [1.0, 0.0, 0.0, 0.0, 0.1];
        cart.limit = 2.4;
        cart.shape = 1;
        let mut pole = link(Some(0), JointKind::Revolute, 0.3, 1.0);
        pole.mount_s = 0.0;
        pole.mount_angle = FRAC_PI_2;
        pole.actuated = false;
        pole.values[4] = 0.002;
        pole.q0 = 0.15;
        Desc { bodies: vec![cart, pole], g: 9.81, tool: Some(1), closures: Vec::new(), elements: Vec::new() }
    }

    /// Two-link planar arm, θ measured from the x-axis (MLS §4.2.3).
    #[must_use]
    pub fn arm2() -> Desc {
        let mut a = link(None, JointKind::Revolute, 1.0, 1.0);
        a.q0 = -FRAC_PI_2 + 0.6;
        let mut b = link(Some(0), JointKind::Revolute, 1.0, 1.0);
        b.q0 = 0.5;
        Desc { bodies: vec![a, b], g: 9.81, tool: Some(1), closures: Vec::new(), elements: Vec::new() }
    }

    /// Passive double pendulum hanging from the ground.
    #[must_use]
    pub fn double_pendulum() -> Desc {
        let mut d = arm2();
        for b in &mut d.bodies {
            b.actuated = false;
            b.values[4] = 0.0;
        }
        d.bodies[0].mount_angle = -FRAC_PI_2;
        d.bodies[0].q0 = 2.0;
        d.bodies[1].q0 = 0.5;
        d
    }

    /// A redundant 7-joint arm (shoulder to wrist links shortening like a human arm, gripper as tool):
    /// two task coordinates leave five for orientation, an elbow and posture goals.
    #[must_use]
    pub fn arm7() -> Desc {
        let spec = [
            (2.0, 0.5, -0.9),
            (1.6, 0.45, 0.5),
            (1.2, 0.4, 0.4),
            (0.9, 0.3, 0.3),
            (0.6, 0.25, 0.3),
            (0.4, 0.2, 0.2),
            (0.3, 0.15, 0.1),
        ];
        let bodies = spec
            .iter()
            .enumerate()
            .map(|(i, &(m, l, q0))| {
                let mut b = link(i.checked_sub(1), JointKind::Revolute, m, l);
                b.q0 = q0;
                b
            })
            .collect();
        Desc { bodies, g: 9.81, tool: Some(6), closures: Vec::new(), elements: Vec::new() }
    }

    /// Four-bar linkage: crank and rocker hinged to the ground, coupler pinned to the rocker tip (a loop).
    #[must_use]
    pub fn four_bar() -> Desc {
        let mut crank = link(None, JointKind::Revolute, 0.5, 0.5);
        crank.q0 = FRAC_PI_2;
        let mut coupler = link(Some(0), JointKind::Revolute, 1.0, 1.2);
        coupler.actuated = false;
        coupler.q0 = -1.2;
        let mut rocker = link(None, JointKind::Revolute, 0.8, 1.0);
        rocker.mount_xy = [1.0, 0.0];
        rocker.actuated = false;
        rocker.q0 = 1.6;
        let closure =
            Closure { kind: ClosureKind::Pin, a: Some(1), s_a: 1.0, b: 2, s_b: 1.0, ground: [0.0; 2] };
        Desc {
            bodies: vec![crank, coupler, rocker],
            g: 9.81,
            tool: Some(1),
            closures: vec![closure],
            elements: Vec::new(),
        }
    }

    /// Slider-crank: crank on the ground, connecting rod whose tip slides in a horizontal ground slot.
    #[must_use]
    pub fn slider_crank() -> Desc {
        let mut crank = link(None, JointKind::Revolute, 0.5, 0.4);
        crank.q0 = 1.0;
        let mut rod = link(Some(0), JointKind::Revolute, 0.6, 1.2);
        rod.actuated = false;
        rod.q0 = -1.3;
        let slot = Closure {
            kind: ClosureKind::Slot { angle: 0.0 },
            a: None,
            s_a: 0.0,
            b: 1,
            s_b: 1.0,
            ground: [0.0; 2],
        };
        Desc { bodies: vec![crank, rod], g: 9.81, tool: Some(1), closures: vec![slot], elements: Vec::new() }
    }

    /// Acrobot: actuated elbow, passive shoulder; target is the upright configuration.
    #[must_use]
    pub fn acrobot() -> Desc {
        let mut d = arm2();
        d.bodies[0].mount_angle = FRAC_PI_2;
        d.bodies[0].actuated = false;
        d.bodies[0].q0 = PI - 0.1;
        d.bodies[1].q0 = 0.0;
        d
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mass_matrix_is_symmetric_and_mdot_minus_2c_is_skew() {
        for d in [presets::cartpole(), presets::arm2(), presets::acrobot()] {
            let r = derive(&d);
            let n = r.n();
            for i in 0..n {
                for j in 0..n {
                    assert_eq!(r.m[i * n + j], r.m[j * n + i]);
                    let s = r.mdot[i * n + j].sub(&r.c[i * n + j].scale(2.0));
                    let st = r.mdot[j * n + i].sub(&r.c[j * n + i].scale(2.0));
                    assert!(s.add(&st).is_zero(), "skew fails at ({i},{j})");
                }
            }
        }
    }
}
