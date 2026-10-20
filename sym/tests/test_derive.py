from __future__ import annotations

import sympy as sp

from meowfer_sym import emit_rust
from meowfer_sym.__main__ import render
from meowfer_sym.model import derive
from meowfer_sym.systems import ALL, arm2, cartpole


def _sym(name: str, sys_params: list) -> sp.Symbol:  # type: ignore[type-arg]
    return next(p.symbol for p in sys_params if p.ident == name)


def test_arm2_matches_mls_eq_4_11() -> None:
    """MLS (4.11): M = [[α + 2βc2, δ + βc2], [δ + βc2, δ]], C = [[−βs2θ̇2, −βs2(θ̇1+θ̇2)], [βs2θ̇1, 0]]."""
    s = arm2()
    d = derive(s)
    m1, m2, l1, r1, r2, i1, i2 = (_sym(n, s.params) for n in ("m1", "m2", "l1", "r1", "r2", "I1", "I2"))
    _, t2 = s.q
    t1d, t2d = s.qd
    alpha = i1 + i2 + m1 * r1**2 + m2 * (l1**2 + r2**2)
    beta = m2 * l1 * r2
    delta = i2 + m2 * r2**2
    c2, s2 = sp.cos(t2), sp.sin(t2)
    m_ref = sp.Matrix([[alpha + 2 * beta * c2, delta + beta * c2], [delta + beta * c2, delta]])
    c_ref = sp.Matrix([[-beta * s2 * t2d, -beta * s2 * (t1d + t2d)], [beta * s2 * t1d, 0]])
    assert sp.simplify(d.M - m_ref) == sp.zeros(2, 2)
    assert sp.simplify(d.C - c_ref) == sp.zeros(2, 2)


def test_cartpole_inertia_matrix() -> None:
    s = cartpole()
    d = derive(s)
    mc, mp, r, ip = (_sym(n, s.params) for n in ("mc", "mp", "r", "Ip"))
    th = s.q[1]
    m_ref = sp.Matrix([[mc + mp, -mp * r * sp.cos(th)], [-mp * r * sp.cos(th), ip + mp * r**2]])
    assert sp.simplify(d.M - m_ref) == sp.zeros(2, 2)


def test_structural_properties() -> None:
    """MLS Lemma 4.2: M symmetric, Ṁ − 2C skew-symmetric."""
    for make in ALL:
        d = derive(make())
        assert sp.simplify(d.M - d.M.T) == sp.zeros(*d.M.shape)
        s = d.Mdot - 2 * d.C
        assert sp.simplify(s + s.T) == sp.zeros(*s.shape)


def test_spatial_jacobian_columns_are_twists() -> None:
    """Planar revolute twist about q_i: ξ = (−ω × q_i, ω) = (q_y, −q_x, 1)."""
    s = arm2()
    d = derive(s)
    assert d.twists is not None
    l1 = _sym("l1", s.params)
    assert d.twists == sp.Matrix([[0, 0], [0, -l1], [1, 1]])


def test_rust_printer_keeps_parentheses() -> None:
    a, b, c, th = sp.symbols("a b c th")
    assert emit_rust.rs(-c * (a + b) * sp.sin(th)) == "-c*(a + b)*f64::sin(th)"
    assert "2.0" in emit_rust.rs(2 * a)


def test_generated_files_are_up_to_date() -> None:
    stale = [p for p, text in render().items() if not p.exists() or p.read_text() != text]
    assert not stale, f"run `make gen`: {stale}"
