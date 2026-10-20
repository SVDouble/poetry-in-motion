"""The demo systems. Add a new robot here and re-run the generator."""

from __future__ import annotations

import sympy as sp

from .model import Body, Frame, Param, System

ZERO = sp.Integer(0)


def cartpole() -> System:
    x, th, xd, thd = sp.symbols("x th xd thd", real=True)
    mc, mp, r, Ip, g, bx, bth = sp.symbols("mc mp r Ip g bx bth", positive=True)
    # θ is measured counter-clockwise from upright: the pole COM sits at (x − r sin θ, r cos θ).
    return System(
        ident="cartpole",
        title="Cart-pole",
        q=[x, th],
        qd=[xd, thd],
        q_tex=["x", r"\theta"],
        params=[
            Param(mc, "mc", "m_c"),
            Param(mp, "mp", "m_p"),
            Param(r, "r", "r"),
            Param(Ip, "Ip", "I_p"),
            Param(g, "g", "g"),
            Param(bx, "bx", r"\beta_x"),
            Param(bth, "bth", r"\beta_\theta"),
        ],
        damping=[bx, bth],
        gravity=g,
        bodies=[
            Body("cart", mc, ZERO, (x, ZERO), ZERO),
            Body("pole", mp, Ip, (x - r * sp.sin(th), r * sp.cos(th)), th),
        ],
        frames=[
            Frame("c", "s", (x, ZERO, ZERO)),
            Frame("p", "c", (ZERO, ZERO, th)),
        ],
        actuation=sp.Matrix([[1], [0]]),
    )


def arm2() -> System:
    t1, t2, t1d, t2d = sp.symbols("th1 th2 th1d th2d", real=True)
    m1, m2, l1, l2, r1, r2, I1, I2, g, b1, b2 = sp.symbols("m1 m2 l1 l2 r1 r2 I1 I2 g b1 b2", positive=True)
    trig: dict[sp.Expr, tuple[sp.Symbol, str]] = {
        sp.cos(t1 + t2): (sp.Symbol("c12"), "c_{12}"),
        sp.sin(t1 + t2): (sp.Symbol("s12"), "s_{12}"),
        sp.cos(t1): (sp.Symbol("c1"), "c_{1}"),
        sp.sin(t1): (sp.Symbol("s1"), "s_{1}"),
        sp.cos(t2): (sp.Symbol("c2"), "c_{2}"),
        sp.sin(t2): (sp.Symbol("s2"), "s_{2}"),
    }
    # MLS §4.2.3: two-link planar manipulator, r_i = distance from joint i to the COM of link i.
    return System(
        ident="arm2",
        title="Two-link planar arm",
        q=[t1, t2],
        qd=[t1d, t2d],
        q_tex=[r"\theta_{1}", r"\theta_{2}"],
        params=[
            Param(m1, "m1", "m_1"),
            Param(m2, "m2", "m_2"),
            Param(l1, "l1", "l_1"),
            Param(l2, "l2", "l_2"),
            Param(r1, "r1", "r_1"),
            Param(r2, "r2", "r_2"),
            Param(I1, "I1", "I_{z1}"),
            Param(I2, "I2", "I_{z2}"),
            Param(g, "g", "g"),
            Param(b1, "b1", r"\beta_1"),
            Param(b2, "b2", r"\beta_2"),
        ],
        damping=[b1, b2],
        gravity=g,
        bodies=[
            Body("link1", m1, I1, (r1 * sp.cos(t1), r1 * sp.sin(t1)), t1),
            Body(
                "link2",
                m2,
                I2,
                (l1 * sp.cos(t1) + r2 * sp.cos(t1 + t2), l1 * sp.sin(t1) + r2 * sp.sin(t1 + t2)),
                t1 + t2,
            ),
        ],
        frames=[
            Frame("1", "s", (ZERO, ZERO, t1)),
            Frame("2", "1", (l1, ZERO, t2)),
            Frame("t", "2", (l2, ZERO, ZERO)),
        ],
        actuation=sp.eye(2),
        tool="t",
        trig=trig,
    )


ALL = (cartpole, arm2)
