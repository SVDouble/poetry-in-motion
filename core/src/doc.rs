//! The derivation as data: LaTeX of every intermediate quantity, serialised to JSON for the browser,
//! which turns it into a step-by-step explanation.

use std::fmt::Write;

use crate::cas::latex::{Names, Printer, angle_const};
use crate::cas::{Angle, Expr};
use crate::mech::{ClosureKind, Derived, JointKind};

/// LaTeX names of every polynomial variable of a derived mechanism.
#[must_use]
pub fn names(d: &Derived) -> Names {
    let (n, nb) = (d.n(), d.vars.nb);
    let v = d.vars;
    let mut vars = vec![String::new(); v.nvars()];
    let mut is_param = vec![false; v.nvars()];
    for i in 0..nb {
        let k = i + 1;
        vars[v.m(i)] = format!("m_{{{k}}}");
        vars[v.l(i)] = format!("l_{{{k}}}");
        vars[v.r(i)] = format!("r_{{{k}}}");
        vars[v.inertia(i)] = format!("I_{{{k}}}");
        vars[v.beta(i)] = format!(r"\beta_{{{k}}}");
    }
    for j in 0..n {
        let q = coord(d, j);
        vars[v.q(j)].clone_from(&q);
        vars[v.qd(j)] = dot(&q, 1);
        vars[v.qdd(j)] = dot(&q, 2);
    }
    vars[v.g()] = "g".into();
    for x in is_param.iter_mut().take(v.np()) {
        *x = true;
    }
    // textbook order inside monomials: m, I, g, l, r, β, then coordinates, accelerations, velocities
    let mut order = Vec::with_capacity(v.nvars());
    order.extend((0..nb).map(|i| v.m(i)));
    order.extend((0..nb).map(|i| v.inertia(i)));
    order.push(v.g());
    order.extend((0..nb).map(|i| v.l(i)));
    order.extend((0..nb).map(|i| v.r(i)));
    order.extend((0..nb).map(|i| v.beta(i)));
    order.extend((0..n).map(|i| v.q(i)));
    order.extend((0..n).map(|i| v.qdd(i)));
    order.extend((0..n).map(|i| v.qd(i)));
    Names {
        vars,
        is_param,
        order,
        angles: (0..n).map(|i| coord(d, i)).collect(),
        angle_labels: (0..n).map(|j| (d.coord_body[j] + 1).to_string()).collect(),
    }
}

/// Coordinate `j`: `\theta_{i}` for a revolute joint `i`, `d_{i}` for a prismatic one (MLS); joints are
/// labelled by the body they move, so coordinates keep their names when fixed bodies are added.
#[must_use]
pub fn coord(d: &Derived, j: usize) -> String {
    let i = d.coord_body[j];
    match d.desc.bodies[i].kind {
        JointKind::Prismatic => format!("d_{{{}}}", i + 1),
        _ => format!(r"\theta_{{{}}}", i + 1),
    }
}

fn kind_name(k: JointKind) -> &'static str {
    match k {
        JointKind::Revolute => "revolute",
        JointKind::Prismatic => "prismatic",
        JointKind::Fixed => "fixed",
    }
}

fn dot(q: &str, order: usize) -> String {
    let (base, sub) = q.split_once('_').unwrap_or((q, ""));
    let accent = if order == 1 { "dot" } else { "ddot" };
    format!(r"\{accent}{{{base}}}_{sub}")
}

/// Angle `k·θ + c` as LaTeX.
fn angle(d: &Derived, a: &Angle) -> String {
    let mut s = String::new();
    for (j, &k) in a.k.iter().enumerate().filter(|(_, k)| **k != 0) {
        let sign = match (k < 0, s.is_empty()) {
            (true, true) => "-",
            (true, false) => " - ",
            (false, true) => "",
            (false, false) => " + ",
        };
        let mag = if k.abs() == 1 { String::new() } else { format!("{} ", k.abs()) };
        let _ = write!(s, "{sign}{mag}{}", coord(d, j));
    }
    if a.c.abs() > 1e-12 || s.is_empty() {
        let c = angle_const(a.c);
        if s.is_empty() {
            s = c;
        } else if let Some(rest) = c.strip_prefix('-') {
            let _ = write!(s, " - {rest}");
        } else {
            let _ = write!(s, " + {c}");
        }
    }
    s
}

struct J<'a> {
    out: String,
    p: Printer<'a>,
}

impl J<'_> {
    fn ex(&self, e: &Expr) -> String {
        esc(&self.p.expr(e))
    }

    fn row(&self, xs: &[Expr]) -> String {
        format!("[{}]", xs.iter().map(|e| format!("\"{}\"", self.ex(e))).collect::<Vec<_>>().join(","))
    }

    fn mat(&self, xs: &[Expr], cols: usize) -> String {
        format!("[{}]", xs.chunks(cols).map(|r| self.row(r)).collect::<Vec<_>>().join(","))
    }

    /// Homogeneous transform `[[cos φ, −sin φ, x], [sin φ, cos φ, y], [0, 0, 1]]`.
    fn hom(&self, d: &Derived, p: &[Expr; 2], phi: &Angle) -> String {
        let s = d.vars.space();
        let (c, sn) = (s.cos(phi), s.sin(phi));
        let m = [
            c.clone(),
            sn.scale(-1.0),
            p[0].clone(),
            sn,
            c,
            p[1].clone(),
            s.zero(),
            s.zero(),
            s.constant(1.0),
        ];
        self.mat(&m, 3)
    }
}

/// Minimal JSON string escaping.
fn esc(s: &str) -> String {
    let mut o = String::with_capacity(s.len() + 8);
    for ch in s.chars() {
        match ch {
            '\\' => o.push_str("\\\\"),
            '"' => o.push_str("\\\""),
            '\n' => o.push_str("\\n"),
            c => o.push(c),
        }
    }
    o
}

#[must_use]
#[allow(clippy::too_many_lines)]
pub fn json(d: &Derived) -> String {
    let n = d.n();
    let v = d.vars;
    let nm = names(d);
    let mut j = J { out: String::new(), p: Printer::new(&nm) };
    let o = &mut String::new();
    let _ = write!(o, "{{\"n\":{n},\"nu\":{},", d.actuated.len());

    // coordinates
    let coords: Vec<String> = (0..n)
        .map(|i| {
            let q = coord(d, i);
            let body = d.coord_body[i];
            format!(
                "{{\"tex\":\"{}\",\"dot\":\"{}\",\"ddot\":\"{}\",\"kind\":\"{}\",\"actuated\":{},\"body\":{body}}}",
                esc(&q),
                esc(&dot(&q, 1)),
                esc(&dot(&q, 2)),
                if d.revolute[i] { "revolute" } else { "prismatic" },
                d.desc.bodies[body].actuated
            )
        })
        .collect();
    let _ = write!(o, "\"coords\":[{}],", coords.join(","));

    // parameters
    let mut params = Vec::new();
    for i in 0..d.vars.nb {
        for (k, (id, idx)) in
            [("m", v.m(i)), ("l", v.l(i)), ("r", v.r(i)), ("I", v.inertia(i)), ("beta", v.beta(i))]
                .iter()
                .enumerate()
        {
            params.push(format!(
                "{{\"id\":\"{id}{}\",\"tex\":\"{}\",\"body\":{i},\"kind\":\"{id}\",\"index\":{idx},\"value\":{}}}",
                i + 1,
                esc(&nm.vars[*idx]),
                d.desc.bodies[i].values[k]
            ));
        }
    }
    params.push(format!(
        "{{\"id\":\"g\",\"tex\":\"g\",\"body\":-1,\"kind\":\"g\",\"index\":{},\"value\":{}}}",
        v.g(),
        d.desc.g
    ));
    let _ = write!(o, "\"params\":[{}],", params.join(","));

    // bodies
    let s = v.space();
    let bodies: Vec<String> = d
        .bodies
        .iter()
        .enumerate()
        .map(|(i, b)| {
            let parent = d.desc.bodies[i].parent.map_or_else(|| "s".to_string(), |p| (p + 1).to_string());
            format!(
                "{{\"label\":\"{}\",\"parent\":\"{parent}\",\"kind\":\"{}\",\"actuated\":{},\"phi\":\"{}\",\
                 \"gLocal\":{},\"gWorld\":{},\"com\":{},\"vcom\":{},\"omega\":\"{}\",\"T\":\"{}\",\"V\":\"{}\"}}",
                i + 1,
                kind_name(d.desc.bodies[i].kind),
                d.desc.bodies[i].actuated,
                esc(&angle(d, &b.world.phi)),
                j.hom(d, &b.local.p, &b.local.phi),
                j.hom(d, &b.world.p, &b.world.phi),
                j.row(&b.com),
                j.row(&b.vcom),
                j.ex(&b.omega),
                j.ex(&b.t),
                j.ex(&b.v),
            )
        })
        .collect();
    let _ = write!(o, "\"bodies\":[{}],", bodies.join(","));

    let _ = write!(o, "\"T\":\"{}\",\"V\":\"{}\",", j.ex(&d.t), j.ex(&d.v));
    let _ = write!(o, "\"M\":{},\"C\":{},", j.mat(&d.m, n), j.mat(&d.c, n));
    // non-zero Christoffel symbols with j ≤ k (Γᵢⱼₖ = Γᵢₖⱼ)
    let mut gam = Vec::new();
    for i in 0..n {
        for a in 0..n {
            for k in a..n {
                let e = &d.gamma[(i * n + a) * n + k];
                if !e.is_zero() {
                    gam.push(format!(
                        "{{\"i\":{},\"j\":{},\"k\":{},\"tex\":\"{}\"}}",
                        i + 1,
                        a + 1,
                        k + 1,
                        j.ex(e)
                    ));
                }
            }
        }
    }
    let _ = write!(o, "\"gamma\":[{}],", gam.join(","));
    let nu = d.actuated.len().max(1);
    let bmat: Vec<Expr> = (0..n * d.actuated.len()).map(|k| s.constant(d.b[k])).collect();
    let _ = write!(
        o,
        "\"dV\":{},\"N\":{},\"B\":{},\"eom\":{},",
        j.row(&d.dv),
        j.row(&d.nvec),
        if d.actuated.is_empty() { "[]".into() } else { j.mat(&bmat, nu) },
        j.row(&d.eom)
    );

    // loop closures
    let closures: Vec<String> = d
        .desc
        .closures
        .iter()
        .zip(&d.closure_points)
        .map(|(c, [pa, pb])| {
            format!(
                "{{\"kind\":\"{}\",\"angle\":{},\"a\":\"{}\",\"b\":\"{}\",\"sa\":{},\"sb\":{},\"pa\":{},\"pb\":{}}}",
                if matches!(c.kind, ClosureKind::Pin) { "pin" } else { "slot" },
                if let ClosureKind::Slot { angle } = c.kind { angle } else { 0.0 },
                c.a.map_or_else(|| "s".to_string(), |a| (a + 1).to_string()),
                c.b + 1,
                c.s_a,
                c.s_b,
                j.row(pa),
                j.row(pb)
            )
        })
        .collect();
    let _ = write!(
        o,
        "\"closures\":[{}],\"phi\":{},\"A\":{},\"adot\":{},",
        closures.join(","),
        j.row(&d.phi),
        if d.phi.is_empty() { "[]".into() } else { j.mat(&d.a_mat, n) },
        j.row(&d.adot_qd)
    );

    match &d.tool {
        Some(t) => {
            let p0 = j.hom(d, &t.p0, &Angle { k: vec![0; n], c: t.phi0 });
            let _ = write!(
                o,
                "\"tool\":{{\"body\":\"{}\",\"p\":{},\"Jp\":{},\"Js\":{},\"twists\":{},\"gst0\":{},\"phi\":\"{}\"}},",
                t.body + 1,
                j.row(&t.pose.p),
                j.mat(&t.jp, n),
                j.mat(&t.js, n),
                j.mat(&t.twists, n),
                p0,
                esc(&angle(d, &t.pose.phi)),
            );
        }
        None => o.push_str("\"tool\":null,"),
    }

    // printing is done: the legend lists every shorthand that appeared
    let legend: Vec<String> =
        j.p.legend().iter().map(|(a, b)| format!("[\"{}\",\"{}\"]", esc(a), esc(b))).collect();
    let _ = write!(o, "\"trig\":[{}]}}", legend.join(","));
    j.out.push_str(o);
    j.out
}
