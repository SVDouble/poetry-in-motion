//! LaTeX printing of [`Expr`] in textbook style.
//!
//! Terms are grouped by their state factor (coordinates, velocities, trigonometric harmonic); each group's
//! parameter polynomial is printed with its common monomial factored out, e.g.
//! `g \left(l_{1} m_{2} + m_{1} r_{1}\right) s_{1}`. Harmonics use the MLS shorthand `c_{12}` for
//! `cos(θ₁ + θ₂)` when every multiplier is 0 or 1.

use std::cell::RefCell;
use std::collections::{BTreeMap, BTreeSet};
use std::fmt::Write;

use super::expr::{Expr, Harmonic};

/// How to print the variables of a [`super::Space`].
#[derive(Debug, Clone)]
pub struct Names {
    /// LaTeX of every polynomial variable.
    pub vars: Vec<String>,
    /// Which variables are parameters (everything else is state: coordinates, velocities, accelerations).
    pub is_param: Vec<bool>,
    /// Order in which variables appear inside a printed monomial.
    pub order: Vec<usize>,
    /// LaTeX of each angle slot (e.g. `\theta_{2}`), and its label for shorthands (e.g. `2`).
    pub angles: Vec<String>,
    pub angle_labels: Vec<String>,
}

#[derive(Debug)]
pub struct Printer<'a> {
    names: &'a Names,
    used: RefCell<BTreeSet<Harmonic>>,
}

type Mono = Vec<u8>;
/// Terms sharing one state factor `(state monomial, harmonic)`, with their parameter monomials.
type Group = ((Mono, Harmonic), Vec<(Mono, f64)>);

impl<'a> Printer<'a> {
    #[must_use]
    pub const fn new(names: &'a Names) -> Self {
        Self { names, used: RefCell::new(BTreeSet::new()) }
    }

    /// `[shorthand, meaning]` for every shorthand harmonic printed so far.
    #[must_use]
    pub fn legend(&self) -> Vec<(String, String)> {
        self.used.borrow().iter().map(|h| (self.harmonic(h, true), self.harmonic(h, false))).collect()
    }

    #[must_use]
    pub fn expr(&self, e: &Expr) -> String {
        if e.is_zero() {
            return "0".into();
        }
        // group by state part
        let mut groups: BTreeMap<(Mono, Harmonic), Vec<(Mono, f64)>> = BTreeMap::new();
        for (k, c) in &e.terms {
            let (mut pm, mut sm) = (k.mono.clone(), k.mono.clone());
            for (i, &p) in self.names.is_param.iter().enumerate() {
                if p {
                    sm[i] = 0;
                } else {
                    pm[i] = 0;
                }
            }
            groups.entry((sm, k.h.clone())).or_default().push((pm, *c));
        }
        // textbook order: higher powers of earlier state variables first, plain terms before harmonics
        let order = &self.names.order;
        let state_key = |sm: &Mono| -> Vec<std::cmp::Reverse<u8>> {
            order.iter().filter(|&&v| !self.names.is_param[v]).map(|&v| std::cmp::Reverse(sm[v])).collect()
        };
        let mut groups: Vec<Group> = groups.into_iter().collect();
        groups.sort_by(|((sa, ha), _), ((sb, hb), _)| {
            state_key(sa).cmp(&state_key(sb)).then(hb.is_one().cmp(&ha.is_one())).then(ha.cmp(hb))
        });
        let mut out = String::new();
        for (i, ((sm, h), poly)) in groups.iter_mut().enumerate() {
            poly.sort_by(|(a, _), (b, _)| order.iter().map(|&v| b[v]).cmp(order.iter().map(|&v| a[v])));
            let (neg, body) = self.group(sm, h, poly);
            match (i, neg) {
                (0, true) => out.push_str("- "),
                (0, false) => {}
                (_, true) => out.push_str(" - "),
                (_, false) => out.push_str(" + "),
            }
            out.push_str(&body);
        }
        out
    }

    /// One group `poly(params) · state · harmonic`; returns (negative?, text without the sign).
    fn group(&self, sm: &Mono, h: &Harmonic, poly: &[(Mono, f64)]) -> (bool, String) {
        let mut state = self.mono(sm);
        if !h.is_one() {
            self.used.borrow_mut().insert(h.clone());
            push_factor(&mut state, &self.harmonic(h, true));
        }
        if poly.len() == 1 {
            let (pm, c) = &poly[0];
            let mut f = self.mono(pm);
            push_factor(&mut f, &state);
            return (*c < 0.0, with_coeff(c.abs(), &f));
        }
        // factor the common parameter monomial and a common |coefficient|
        let gcd: Mono =
            (0..poly[0].0.len()).map(|v| poly.iter().map(|(m, _)| m[v]).min().unwrap_or(0)).collect();
        let cf = poly[0].1.abs();
        let common_c = poly.iter().all(|(_, c)| (c.abs() - cf).abs() < 1e-12);
        let all_neg = poly.iter().all(|(_, c)| *c < 0.0);
        let mut inner = String::new();
        for (i, (m, c)) in poly.iter().enumerate() {
            let rest: Mono = m.iter().zip(&gcd).map(|(a, b)| a - b).collect();
            let mut c = if common_c { c / cf } else { *c };
            if all_neg {
                c = -c;
            }
            let f = self.mono(&rest);
            let term = with_coeff(c.abs(), &f);
            match (i, c < 0.0) {
                (0, true) => inner.push_str("- "),
                (0, false) => {}
                (_, true) => inner.push_str(" - "),
                (_, false) => inner.push_str(" + "),
            }
            inner.push_str(&term);
        }
        let outer = self.mono(&gcd);
        let coeff = if common_c { cf } else { 1.0 };
        if outer.is_empty() && state.is_empty() && number(coeff) == "1" && !all_neg {
            return (false, inner); // nothing multiplies the sum: no parentheses needed
        }
        let mut f = outer;
        push_factor(&mut f, &format!(r"\left({inner}\right)"));
        push_factor(&mut f, &state);
        (all_neg, with_coeff(coeff, &f))
    }

    fn mono(&self, m: &Mono) -> String {
        let mut s = String::new();
        for &v in &self.names.order {
            match m[v] {
                0 => {}
                1 => push_factor(&mut s, &self.names.vars[v]),
                e => push_factor(&mut s, &format!("{}^{{{e}}}", self.names.vars[v])),
            }
        }
        s
    }

    /// `c_{12}` (shorthand) or `\cos\left(\theta_{1} - 2 \theta_{2}\right)`.
    fn harmonic(&self, h: &Harmonic, short: bool) -> String {
        let f = if h.sin { "s" } else { "c" };
        let simple = h.k.iter().all(|&k| k == 0 || k == 1);
        if short && simple {
            let labels: Vec<&str> =
                h.k.iter()
                    .enumerate()
                    .filter(|(_, k)| **k == 1)
                    .map(|(j, _)| self.names.angle_labels[j].as_str())
                    .collect();
            let sep = if labels.iter().any(|l| l.len() > 1) { "," } else { "" };
            return format!("{f}_{{{}}}", labels.join(sep));
        }
        let mut arg = String::new();
        for (j, &k) in h.k.iter().enumerate().filter(|(_, k)| **k != 0) {
            let sign = match (k < 0, arg.is_empty()) {
                (true, true) => "-",
                (true, false) => " - ",
                (false, true) => "",
                (false, false) => " + ",
            };
            let mag = if k.abs() == 1 { String::new() } else { format!("{} ", k.abs()) };
            let _ = write!(arg, "{sign}{mag}{}", self.names.angles[j]);
        }
        let func = if h.sin { r"\sin" } else { r"\cos" };
        if h.k.iter().filter(|&&k| k != 0).count() == 1 && h.k.iter().all(|&k| k == 0 || k == 1) {
            format!("{func} {arg}")
        } else {
            format!(r"{func}\left({arg}\right)")
        }
    }
}

fn push_factor(s: &mut String, f: &str) {
    if f.is_empty() {
        return;
    }
    if !s.is_empty() {
        s.push(' ');
    }
    s.push_str(f);
}

/// `coefficient · factors`, omitting a unit coefficient.
fn with_coeff(c: f64, factors: &str) -> String {
    let n = number(c);
    match (n.as_str(), factors.is_empty()) {
        (_, true) => n,
        ("1", false) => factors.to_string(),
        _ => format!("{n} {factors}"),
    }
}

/// Non-negative number: integer, small rational (`\frac{1}{2}`), multiple of π-free decimal otherwise.
#[must_use]
pub fn number(c: f64) -> String {
    if (c - c.round()).abs() < 1e-9 {
        return format!("{}", c.round() as i64);
    }
    for den in 2..=24_i64 {
        let num = c * den as f64;
        if (num - num.round()).abs() < 1e-9 {
            return format!(r"\frac{{{}}}{{{den}}}", num.round() as i64);
        }
    }
    let s = format!("{c:.4}");
    s.trim_end_matches('0').to_string()
}

/// Signed number for standalone use (e.g. an angle offset): `\frac{\pi}{2}` when it is a nice multiple of π.
#[must_use]
pub fn angle_const(c: f64) -> String {
    use core::f64::consts::PI;
    if c.abs() < 1e-12 {
        return "0".into();
    }
    let sign = if c < 0.0 { "-" } else { "" };
    let x = c.abs() / PI;
    for den in 1..=12_i64 {
        let num = x * den as f64;
        if (num - num.round()).abs() < 1e-9 {
            let num = num.round() as i64;
            let top = if num == 1 { r"\pi".to_string() } else { format!(r"{num}\pi") };
            return if den == 1 { format!("{sign}{top}") } else { format!(r"{sign}\frac{{{top}}}{{{den}}}") };
        }
    }
    format!("{sign}{}", number(c.abs()))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cas::expr::{Angle, Space};

    fn names() -> Names {
        Names {
            vars: vec!["m".into(), "l".into(), "g".into()],
            is_param: vec![true, true, true],
            order: vec![2, 0, 1],
            angles: vec![r"\theta_{1}".into(), r"\theta_{2}".into()],
            angle_labels: vec!["1".into(), "2".into()],
        }
    }

    #[test]
    fn groups_and_factors() {
        let s = Space { nvars: 3, nang: 2 };
        let s1 = s.sin(&Angle { k: vec![1, 0], c: 0.0 });
        let e = s.var(2).mul(&s.var(0)).mul(&s1).add(&s.var(2).mul(&s.var(1)).mul(&s1));
        let n = names();
        let p = Printer::new(&n);
        assert_eq!(p.expr(&e), r"g \left(m + l\right) s_{1}");
        assert_eq!(p.legend(), vec![("s_{1}".to_string(), r"\sin \theta_{1}".to_string())]);
    }

    #[test]
    fn coefficients_and_signs() {
        let s = Space { nvars: 3, nang: 2 };
        let e = s.var(0).scale(-0.5).add(&s.cos(&Angle { k: vec![1, -1], c: 0.0 }).scale(2.0));
        let n = names();
        assert_eq!(
            Printer::new(&n).expr(&e),
            r"- \frac{1}{2} m + 2 \cos\left(\theta_{1} - \theta_{2}\right)"
        );
        assert_eq!(angle_const(core::f64::consts::FRAC_PI_2), r"\frac{\pi}{2}");
    }
}
