//! Exact symbolic expressions for planar rigid-body mechanics.
//!
//! Every quantity of a planar open chain — positions, velocities, energies, M, C, N, Jacobians — is a
//! finite sum of terms
//!
//! ```text
//! c · Π pᵢ^aᵢ · Π d_j^b_j · Π q̇_k^e_k · Π q̈_l^f_l · {cos | sin}(k · θ)
//! ```
//!
//! with numeric `c`, symbolic parameters `p`, prismatic coordinates `d`, velocities `q̇`, accelerations
//! `q̈` and an integer combination `k · θ` of revolute angles. Products of trigonometric factors are
//! expanded with the product-to-sum identities, so each expression has a unique *Fourier canonical form*:
//! no simplifier is needed, and identities such as `c₁c₁₂ + s₁s₁₂ = c₂` hold by construction.

use std::collections::BTreeMap;

/// Coefficients smaller than this are treated as exact zeros (they come from cos(π/2) and friends).
const EPS: f64 = 1e-12;

/// `cos(k · θ)` or `sin(k · θ)`, canonical: the first non-zero entry of `k` is positive, and the constant
/// harmonic is `cos(0) = 1`.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct Harmonic {
    pub sin: bool,
    pub k: Vec<i8>,
}

impl Harmonic {
    #[must_use]
    pub fn one(nang: usize) -> Self {
        Self { sin: false, k: vec![0; nang] }
    }

    #[must_use]
    pub fn is_one(&self) -> bool {
        !self.sin && self.k.iter().all(|&x| x == 0)
    }

    /// Canonicalise `trig(k · θ)`: returns the sign to apply, or `None` for `sin(0) = 0`.
    fn canon(sin: bool, mut k: Vec<i8>) -> Option<(f64, Self)> {
        match k.iter().find(|&&x| x != 0) {
            None if sin => None,
            Some(&first) if first < 0 => {
                for x in &mut k {
                    *x = -*x;
                }
                Some((if sin { -1.0 } else { 1.0 }, Self { sin, k }))
            }
            _ => Some((1.0, Self { sin, k })),
        }
    }

    /// Product-to-sum: `self · other = Σ cᵢ hᵢ`.
    fn mul(&self, other: &Self) -> Vec<(f64, Self)> {
        if self.is_one() {
            return vec![(1.0, other.clone())];
        }
        if other.is_one() {
            return vec![(1.0, self.clone())];
        }
        let plus: Vec<i8> = self.k.iter().zip(&other.k).map(|(a, b)| a + b).collect();
        let minus: Vec<i8> = self.k.iter().zip(&other.k).map(|(a, b)| a - b).collect();
        // (sin?, coefficient) for the (a − b) and (a + b) parts
        let ((s_m, c_m), (s_p, c_p)) = match (self.sin, other.sin) {
            (false, false) => ((false, 0.5), (false, 0.5)), // cos a cos b
            (true, true) => ((false, 0.5), (false, -0.5)),  // sin a sin b
            (true, false) => ((true, 0.5), (true, 0.5)),    // sin a cos b
            (false, true) => ((true, -0.5), (true, 0.5)),   // cos a sin b
        };
        let mut out = Vec::with_capacity(2);
        for (sin, c, k) in [(s_m, c_m, minus), (s_p, c_p, plus)] {
            if let Some((sign, h)) = Self::canon(sin, k) {
                out.push((c * sign, h));
            }
        }
        out
    }
}

/// An angle `k · θ + c` (linear in the revolute coordinates, numeric offset).
#[derive(Clone, Debug, PartialEq)]
pub struct Angle {
    pub k: Vec<i8>,
    pub c: f64,
}

impl Angle {
    #[must_use]
    pub fn zero(nang: usize) -> Self {
        Self { k: vec![0; nang], c: 0.0 }
    }

    #[must_use]
    pub fn add(&self, o: &Self) -> Self {
        Self { k: self.k.iter().zip(&o.k).map(|(a, b)| a + b).collect(), c: self.c + o.c }
    }

    #[must_use]
    pub fn plus_const(&self, c: f64) -> Self {
        Self { k: self.k.clone(), c: self.c + c }
    }

    #[must_use]
    pub fn plus_joint(&self, j: usize) -> Self {
        let mut k = self.k.clone();
        k[j] += 1;
        Self { k, c: self.c }
    }

    #[must_use]
    pub fn eval(&self, q: &[f64]) -> f64 {
        self.c + self.k.iter().zip(q).map(|(&k, &q)| f64::from(k) * q).sum::<f64>()
    }
}

/// Sizes of the variable space an expression lives in.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Space {
    /// Number of polynomial variables (parameters, coordinates, velocities, accelerations).
    pub nvars: usize,
    /// Number of angle slots (one per coordinate; only revolute ones are ever non-zero).
    pub nang: usize,
}

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct Key {
    pub mono: Vec<u8>,
    pub h: Harmonic,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Expr {
    pub terms: BTreeMap<Key, f64>,
    nvars: usize,
    nang: usize,
}

impl Space {
    #[must_use]
    pub fn zero(&self) -> Expr {
        Expr { terms: BTreeMap::new(), nvars: self.nvars, nang: self.nang }
    }

    #[must_use]
    pub fn constant(&self, c: f64) -> Expr {
        let mut e = self.zero();
        e.push(Key { mono: vec![0; self.nvars], h: Harmonic::one(self.nang) }, c);
        e
    }

    #[must_use]
    pub fn var(&self, i: usize) -> Expr {
        let mut mono = vec![0; self.nvars];
        mono[i] = 1;
        let mut e = self.zero();
        e.push(Key { mono, h: Harmonic::one(self.nang) }, 1.0);
        e
    }

    fn harmonic(&self, sin: bool, k: Vec<i8>, c: f64) -> Expr {
        let mut e = self.zero();
        if let Some((sign, h)) = Harmonic::canon(sin, k) {
            e.push(Key { mono: vec![0; self.nvars], h }, sign * c);
        }
        e
    }

    /// `cos(k·θ + c) = cos c · cos(k·θ) − sin c · sin(k·θ)`
    #[must_use]
    pub fn cos(&self, a: &Angle) -> Expr {
        let (s, c) = a.c.sin_cos();
        self.harmonic(false, a.k.clone(), c).add(&self.harmonic(true, a.k.clone(), -s))
    }

    /// `sin(k·θ + c) = cos c · sin(k·θ) + sin c · cos(k·θ)`
    #[must_use]
    pub fn sin(&self, a: &Angle) -> Expr {
        let (s, c) = a.c.sin_cos();
        self.harmonic(true, a.k.clone(), c).add(&self.harmonic(false, a.k.clone(), s))
    }
}

impl Expr {
    fn push(&mut self, k: Key, c: f64) {
        let slot = self.terms.entry(k).or_insert(0.0);
        *slot += c;
    }

    fn pruned(mut self) -> Self {
        self.terms.retain(|_, c| c.abs() > EPS);
        self
    }

    #[must_use]
    pub fn space(&self) -> Space {
        Space { nvars: self.nvars, nang: self.nang }
    }

    #[must_use]
    pub fn is_zero(&self) -> bool {
        self.terms.is_empty()
    }

    #[must_use]
    pub fn add(&self, o: &Self) -> Self {
        let mut r = self.clone();
        for (k, c) in &o.terms {
            r.push(k.clone(), *c);
        }
        r.pruned()
    }

    #[must_use]
    pub fn sub(&self, o: &Self) -> Self {
        self.add(&o.scale(-1.0))
    }

    #[must_use]
    pub fn scale(&self, s: f64) -> Self {
        let mut r = self.clone();
        for c in r.terms.values_mut() {
            *c *= s;
        }
        r.pruned()
    }

    #[must_use]
    pub fn mul(&self, o: &Self) -> Self {
        let mut r = self.space().zero();
        for (ka, ca) in &self.terms {
            for (kb, cb) in &o.terms {
                let mono: Vec<u8> = ka.mono.iter().zip(&kb.mono).map(|(a, b)| a + b).collect();
                for (c, h) in ka.h.mul(&kb.h) {
                    r.push(Key { mono: mono.clone(), h }, ca * cb * c);
                }
            }
        }
        r.pruned()
    }

    #[must_use]
    pub fn square(&self) -> Self {
        self.mul(self)
    }

    /// ∂/∂(polynomial variable `i`).
    #[must_use]
    pub fn diff_var(&self, i: usize) -> Self {
        let mut r = self.space().zero();
        for (k, c) in &self.terms {
            let e = k.mono[i];
            if e > 0 {
                let mut mono = k.mono.clone();
                mono[i] -= 1;
                r.push(Key { mono, h: k.h.clone() }, c * f64::from(e));
            }
        }
        r.pruned()
    }

    /// ∂/∂θ_j for the angle slot `j`.
    #[must_use]
    pub fn diff_angle(&self, j: usize) -> Self {
        let mut r = self.space().zero();
        for (k, c) in &self.terms {
            let kj = k.h.k[j];
            if kj != 0 {
                let h = Harmonic { sin: !k.h.sin, k: k.h.k.clone() };
                let sign = if k.h.sin { 1.0 } else { -1.0 }; // d cos = −k sin, d sin = k cos
                r.push(Key { mono: k.mono.clone(), h }, c * sign * f64::from(kj));
            }
        }
        r.pruned()
    }

    /// Replace polynomial variable `i` by the expression `by` (used for q̇ ↦ q̈ style substitutions).
    #[must_use]
    pub fn subs_var(&self, i: usize, by: &Self) -> Self {
        let mut r = self.space().zero();
        for (k, c) in &self.terms {
            let e = k.mono[i];
            let mut mono = k.mono.clone();
            mono[i] = 0;
            let mut part = self.space().zero();
            part.push(Key { mono, h: k.h.clone() }, *c);
            for _ in 0..e {
                part = part.mul(by);
            }
            r = r.add(&part);
        }
        r
    }

    /// Set every angle to zero and every listed polynomial variable to zero.
    #[must_use]
    pub fn at_zero(&self, vars: &[usize]) -> Self {
        let mut r = self.space().zero();
        for (k, c) in &self.terms {
            if k.h.sin || vars.iter().any(|&v| k.mono[v] > 0) {
                continue; // sin(0) = 0, or a vanishing coordinate
            }
            r.push(Key { mono: k.mono.clone(), h: Harmonic::one(self.nang) }, *c);
        }
        r.pruned()
    }

    /// Numeric value (tests and one-off evaluations; the simulator uses compiled tapes).
    #[must_use]
    pub fn eval(&self, vars: &[f64], angles: &[f64]) -> f64 {
        self.terms
            .iter()
            .map(|(k, c)| {
                let mono: f64 = k
                    .mono
                    .iter()
                    .enumerate()
                    .filter(|(_, e)| **e > 0)
                    .map(|(i, &e)| vars[i].powi(i32::from(e)))
                    .product();
                let a: f64 = k.h.k.iter().zip(angles).map(|(&k, &q)| f64::from(k) * q).sum();
                let t = if k.h.is_one() {
                    1.0
                } else if k.h.sin {
                    a.sin()
                } else {
                    a.cos()
                };
                c * mono * t
            })
            .sum()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sp() -> Space {
        Space { nvars: 3, nang: 2 }
    }

    fn ang(k: [i8; 2], c: f64) -> Angle {
        Angle { k: k.to_vec(), c }
    }

    #[test]
    fn pythagoras_holds_by_construction() {
        let s = sp();
        let a = ang([1, 1], 0.3);
        let one = s.cos(&a).square().add(&s.sin(&a).square());
        assert_eq!(one, s.constant(1.0));
    }

    #[test]
    fn angle_difference_identity() {
        // c₁ c₁₂ + s₁ s₁₂ = c₂
        let s = sp();
        let (a1, a12) = (ang([1, 0], 0.0), ang([1, 1], 0.0));
        let lhs = s.cos(&a1).mul(&s.cos(&a12)).add(&s.sin(&a1).mul(&s.sin(&a12)));
        assert_eq!(lhs, s.cos(&ang([0, 1], 0.0)));
    }

    #[test]
    fn phase_offsets_fold_into_coefficients() {
        let s = sp();
        let up = s.cos(&ang([0, 1], core::f64::consts::FRAC_PI_2));
        assert_eq!(up, s.sin(&ang([0, 1], 0.0)).scale(-1.0));
    }

    #[test]
    fn derivatives_match_numeric() {
        let s = sp();
        let e = s.var(0).mul(&s.sin(&ang([1, -2], 0.4))).mul(&s.var(1).square());
        let (vars, th) = ([1.3, -0.7, 0.2], [0.5, 1.1]);
        let h = 1e-6;
        let d_num = (e.eval(&vars, &[th[1 - 1] + h, th[1]]) - e.eval(&vars, &[th[0] - h, th[1]])) / (2.0 * h);
        assert!((e.diff_angle(0).eval(&vars, &th) - d_num).abs() < 1e-6);
        let d_num = (e.eval(&[vars[0], vars[1] + h, vars[2]], &th)
            - e.eval(&[vars[0], vars[1] - h, vars[2]], &th))
            / (2.0 * h);
        assert!((e.diff_var(1).eval(&vars, &th) - d_num).abs() < 1e-6);
    }

    #[test]
    fn products_match_numeric() {
        let s = sp();
        let a = s.cos(&ang([1, 0], 0.2)).add(&s.var(2).mul(&s.sin(&ang([1, 1], 0.0))));
        let b = s.sin(&ang([0, 1], -0.4)).scale(3.0).add(&s.var(0));
        let (vars, th) = ([0.9, 0.0, -1.7], [0.3, -1.2]);
        assert!((a.mul(&b).eval(&vars, &th) - a.eval(&vars, &th) * b.eval(&vars, &th)).abs() < 1e-12);
    }
}
