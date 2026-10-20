"""Planar rigid-body models and their Lagrangian derivation.

Notation follows Murray, Li & Sastry, *A Mathematical Introduction to Robotic Manipulation* (MLS), ch. 4:

    M(q) q̈ + C(q, q̇) q̇ + N(q, q̇) = τ

* M   — manipulator inertia matrix, M = ∂²T/∂q̇²
* C   — Coriolis matrix built from Christoffel symbols (MLS eq. 4.23), so that Ṁ − 2C is skew-symmetric
* N   — gravity and friction: N = ∂V/∂q + diag(β) q̇
* g_ab ∈ SE(2) — rigid transforms between frames (g is *not* gravity here, as in MLS)
* ξ_i — joint twists in spatial coordinates, written as (v_x, v_y, ω) for the plane
"""

from __future__ import annotations

from dataclasses import dataclass, field

import sympy as sp

type Pose = tuple[sp.Expr, sp.Expr, sp.Expr]


@dataclass(frozen=True)
class Param:
    symbol: sp.Symbol
    ident: str  # identifier used in generated code / UI
    tex: str  # LaTeX name


@dataclass(frozen=True)
class Body:
    name: str
    mass: sp.Expr
    inertia: sp.Expr  # planar: inertia about the COM, z axis
    com: tuple[sp.Expr, sp.Expr]  # COM position in the spatial frame S
    angle: sp.Expr  # orientation in S


@dataclass(frozen=True)
class Frame:
    name: str  # subscript used in g_{ab}, e.g. "1" or "t"
    parent: str
    local: Pose  # (x, y, φ) of this frame relative to its parent


@dataclass
class System:
    ident: str
    title: str
    q: list[sp.Symbol]
    qd: list[sp.Symbol]
    q_tex: list[str]
    params: list[Param]
    damping: list[sp.Symbol]  # β_i, one per coordinate (must also appear in params)
    gravity: sp.Symbol
    bodies: list[Body]
    frames: list[Frame]
    actuation: sp.Matrix  # τ = B u
    tool: str | None = None  # frame used for the end-effector Jacobians
    trig: dict[sp.Expr, tuple[sp.Symbol, str]] = field(default_factory=dict)  # compact LaTeX


@dataclass
class Derived:
    T: sp.Expr
    V: sp.Expr
    M: sp.Matrix
    C: sp.Matrix
    dV: sp.Matrix  # ∂V/∂q
    N: sp.Matrix
    Mdot: sp.Matrix
    dNdq: sp.Matrix
    world: dict[str, Pose]  # g_{s,frame}
    tool_p: sp.Matrix | None  # tool position p_st
    Jp: sp.Matrix | None  # ∂p_st/∂q
    Js: sp.Matrix | None  # spatial manipulator Jacobian (planar, 3 x n)
    twists: sp.Matrix | None  # ξ_i at the reference configuration q = 0 (columns)


def simp(e: sp.Expr) -> sp.Expr:
    return sp.simplify(sp.trigsimp(sp.expand(e)))


def compose(parent: Pose, local: Pose) -> Pose:
    px, py, pphi = parent
    x, y, phi = local
    return (
        px + sp.cos(pphi) * x - sp.sin(pphi) * y,
        py + sp.sin(pphi) * x + sp.cos(pphi) * y,
        pphi + phi,
    )


def homogeneous(pose: Pose) -> sp.Matrix:
    x, y, phi = pose
    return sp.Matrix([[sp.cos(phi), -sp.sin(phi), x], [sp.sin(phi), sp.cos(phi), y], [0, 0, 1]])


def derive(sys: System) -> Derived:
    q = sp.Matrix(sys.q)
    qd = sp.Matrix(sys.qd)
    n = len(sys.q)

    T: sp.Expr = sp.Integer(0)
    V: sp.Expr = sp.Integer(0)
    for b in sys.bodies:
        v = sp.Matrix(b.com).jacobian(q) * qd
        w = (sp.Matrix([b.angle]).jacobian(q) * qd)[0]
        T += sp.Rational(1, 2) * b.mass * (v.T * v)[0] + sp.Rational(1, 2) * b.inertia * w**2
        V += b.mass * sys.gravity * b.com[1]
    T, V = simp(T), simp(V)

    M = sp.Matrix(n, n, lambda i, j: simp(sp.diff(T, qd[i], qd[j])))

    # MLS (4.23): C_ij = ½ Σ_k (∂M_ij/∂q_k + ∂M_ik/∂q_j − ∂M_kj/∂q_i) q̇_k
    C = sp.zeros(n, n)
    for i in range(n):
        for j in range(n):
            C[i, j] = simp(
                sum(
                    sp.Rational(1, 2)
                    * (sp.diff(M[i, j], q[k]) + sp.diff(M[i, k], q[j]) - sp.diff(M[k, j], q[i]))
                    * qd[k]
                    for k in range(n)
                )
            )

    dV = sp.Matrix([simp(sp.diff(V, qi)) for qi in sys.q])
    N = sp.Matrix([dV[i] + sys.damping[i] * qd[i] for i in range(n)])
    Mdot = sp.Matrix(n, n, lambda i, j: simp(sum(sp.diff(M[i, j], q[k]) * qd[k] for k in range(n))))
    dNdq = sp.Matrix(n, n, lambda i, j: simp(sp.diff(dV[i], q[j])))

    world: dict[str, Pose] = {"s": (sp.Integer(0), sp.Integer(0), sp.Integer(0))}
    for f in sys.frames:
        x, y, phi = compose(world[f.parent], f.local)
        world[f.name] = (simp(x), simp(y), simp(phi))

    tool_p = Jp = Js = twists = None
    if sys.tool:
        x, y, phi = world[sys.tool]
        tool_p = sp.Matrix([x, y])
        Jp = sp.Matrix(2, n, lambda i, j: simp(sp.diff(tool_p[i], q[j])))
        # Spatial velocity of g_st: ġ g⁻¹ ↦ (ṗ + ω × ... ) — planar form V^s = (ṗ_x + φ̇ p_y, ṗ_y − φ̇ p_x, φ̇)
        dphi = [sp.diff(phi, qj) for qj in sys.q]
        Js = sp.Matrix(
            3,
            n,
            lambda i, j: simp(
                [Jp[0, j] + dphi[j] * y, Jp[1, j] - dphi[j] * x, dphi[j]][i],
            ),
        )
        twists = Js.subs({qi: 0 for qi in sys.q}).applyfunc(simp)

    return Derived(T, V, M, C, dV, N, Mdot, dNdq, world, tool_p, Jp, Js, twists)
