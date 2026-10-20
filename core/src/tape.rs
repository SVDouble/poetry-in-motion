//! Compilation of symbolic expressions into flat, allocation-free evaluation tapes.
//!
//! Each term `c · P(params) · S(q, q̇) · h(θ)` is stored as indices into three shared tables: parameter
//! monomials (folded into the coefficient whenever the parameters change), state monomials and
//! harmonics (each evaluated once per call). Evaluating the full M, C, N of a 2-DOF system is a few dozen
//! multiply-adds and a handful of `sin`/`cos`.

use std::collections::HashMap;

use crate::cas::{Expr, Harmonic};
use crate::mech::Vars;

#[derive(Clone, Copy, Debug)]
struct Term {
    c: f64,
    pm: u32,
    sm: u32,
    h: u32,
}

#[derive(Clone, Debug)]
pub struct Tape {
    /// Harmonics as sparse `(angle slot, multiplier)` lists, and whether they are sines.
    harm: Vec<(bool, Vec<(usize, f64)>)>,
    /// State monomials over the state vector `[q₀ … q_{n−1}, q̇₀ … q̇_{n−1}]`.
    smono: Vec<Vec<(usize, i32)>>,
    /// Parameter monomials over the parameter vector.
    pmono: Vec<Vec<(usize, i32)>>,
    terms: Vec<Term>,
    /// `[start, end)` of each output in `terms`.
    outputs: Vec<(u32, u32)>,
    /// Coefficients with the parameter monomial folded in (refreshed by `set_params`).
    folded: Vec<f64>,
    hv: Vec<f64>,
    sv: Vec<f64>,
    state: Vec<f64>,
    n: usize,
}

impl Tape {
    /// # Panics
    /// If an expression contains accelerations (only `q`, `q̇` and parameters can be compiled).
    #[must_use]
    pub fn compile(vars: Vars, exprs: &[&Expr]) -> Self {
        let (np, n) = (vars.np(), vars.n);
        let mut harm_ix: HashMap<Harmonic, u32> = HashMap::new();
        let mut s_ix: HashMap<Vec<(usize, i32)>, u32> = HashMap::new();
        let mut p_ix: HashMap<Vec<(usize, i32)>, u32> = HashMap::new();
        let mut tape = Self {
            harm: Vec::new(),
            smono: Vec::new(),
            pmono: Vec::new(),
            terms: Vec::new(),
            outputs: Vec::with_capacity(exprs.len()),
            folded: Vec::new(),
            hv: Vec::new(),
            sv: Vec::new(),
            state: vec![0.0; 2 * n],
            n,
        };
        for e in exprs {
            let start = tape.terms.len() as u32;
            for (k, &c) in &e.terms {
                let mut pm = Vec::new();
                let mut sm = Vec::new();
                for (v, &x) in k.mono.iter().enumerate().filter(|(_, x)| **x > 0) {
                    if v < np {
                        pm.push((v, i32::from(x)));
                    } else if v < np + 2 * n {
                        sm.push((v - np, i32::from(x)));
                    } else {
                        panic!("accelerations cannot be compiled into a tape");
                    }
                }
                let h = *harm_ix.entry(k.h.clone()).or_insert_with(|| {
                    let sparse =
                        k.h.k
                            .iter()
                            .enumerate()
                            .filter(|(_, x)| **x != 0)
                            .map(|(j, &x)| (j, f64::from(x)))
                            .collect();
                    tape.harm.push((k.h.sin, sparse));
                    (tape.harm.len() - 1) as u32
                });
                let sm = *s_ix.entry(sm.clone()).or_insert_with(|| {
                    tape.smono.push(sm);
                    (tape.smono.len() - 1) as u32
                });
                let pm = *p_ix.entry(pm.clone()).or_insert_with(|| {
                    tape.pmono.push(pm);
                    (tape.pmono.len() - 1) as u32
                });
                tape.terms.push(Term { c, pm, sm, h });
            }
            tape.outputs.push((start, tape.terms.len() as u32));
        }
        tape.folded = tape.terms.iter().map(|t| t.c).collect();
        tape.hv = vec![0.0; tape.harm.len()];
        tape.sv = vec![0.0; tape.smono.len()];
        tape
    }

    #[must_use]
    pub fn len(&self) -> usize {
        self.outputs.len()
    }

    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.outputs.is_empty()
    }

    #[must_use]
    pub fn terms(&self) -> usize {
        self.terms.len()
    }

    /// Fold the parameter values into the term coefficients.
    pub fn set_params(&mut self, p: &[f64]) {
        let pv: Vec<f64> =
            self.pmono.iter().map(|m| m.iter().map(|&(v, e)| p[v].powi(e)).product()).collect();
        for (f, t) in self.folded.iter_mut().zip(&self.terms) {
            *f = t.c * pv[t.pm as usize];
        }
    }

    /// Evaluate every output at `(q, q̇)` into `out` (length ≥ `self.len()`).
    pub fn eval(&mut self, q: &[f64], qd: &[f64], out: &mut [f64]) {
        let n = self.n;
        self.state[..n].copy_from_slice(&q[..n]);
        self.state[n..2 * n].copy_from_slice(&qd[..n]);
        for (v, (sin, ks)) in self.hv.iter_mut().zip(&self.harm) {
            *v = if ks.is_empty() {
                1.0
            } else {
                let a: f64 = ks.iter().map(|&(j, k)| k * q[j]).sum();
                if *sin { a.sin() } else { a.cos() }
            };
        }
        for (v, m) in self.sv.iter_mut().zip(&self.smono) {
            *v = m.iter().map(|&(i, e)| if e == 1 { self.state[i] } else { self.state[i].powi(e) }).product();
        }
        for (o, &(a, b)) in out.iter_mut().zip(&self.outputs) {
            let mut acc = 0.0;
            for t in a as usize..b as usize {
                let term = self.terms[t];
                acc += self.folded[t] * self.sv[term.sm as usize] * self.hv[term.h as usize];
            }
            *o = acc;
        }
    }
}
