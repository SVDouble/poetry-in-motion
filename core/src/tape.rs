//! Compilation of symbolic expressions into flat, allocation-free evaluation tapes.
//!
//! Each term `c · P(params) · S(q, q̇) · h(θ)` is stored as indices into three shared tables: parameter
//! monomials (folded into the coefficient whenever the parameters change), state monomials and
//! harmonics (each evaluated once per call, one `sin_cos` per distinct angle). After folding, terms of an
//! output that share their state monomial and harmonic differ only in the coefficient and are merged, so
//! the hot loop runs over far fewer terms than the symbolic expression has.

use std::collections::{BTreeMap, HashMap};
use std::sync::Arc;

use crate::cas::{Expr, Harmonic};
use crate::mech::Vars;
use crate::memory::{arc_bytes, vec_bytes};

#[derive(Clone, Copy, Debug)]
struct Term {
    c: f64,
    pm: u32,
    sm: u32,
    h: u32,
}

/// A merged term of the hot loop: folded coefficient × state monomial × harmonic.
#[derive(Clone, Copy, Debug)]
struct Hot {
    k: f64,
    sm: u32,
    h: u32,
}

/// Compiled symbolic tables, shared by all slot clones. No evaluation state lives here.
#[derive(Debug)]
struct Program {
    harm: Vec<(bool, Vec<(usize, f64)>)>,
    angles: Vec<Vec<(usize, f64)>>,
    harm_angle: Vec<(u32, bool)>,
    smono: Vec<Vec<(usize, i32)>>,
    pmono: Vec<Vec<(usize, i32)>>,
    terms: Vec<Term>,
    outputs: Vec<(u32, u32)>,
    n: usize,
}

/// Parameter-dependent coefficients. Shared until a slot receives different physical parameters.
#[derive(Debug)]
struct Coefficients {
    hot: Vec<Hot>,
    hot_out: Vec<(u32, u32)>,
}

impl Coefficients {
    fn new(program: &Program, params: Option<&[f64]>) -> Self {
        let pv: Vec<f64> = program
            .pmono
            .iter()
            .map(|m| params.map_or(1.0, |p| m.iter().map(|&(v, e)| p[v].powi(e)).product()))
            .collect();
        let mut result = Self { hot: Vec::new(), hot_out: Vec::new() };
        for &(a, b) in &program.outputs {
            let mut acc: BTreeMap<(u32, u32), f64> = BTreeMap::new();
            for term in &program.terms[a as usize..b as usize] {
                *acc.entry((term.sm, term.h)).or_insert(0.0) += term.c * pv[term.pm as usize];
            }
            let start = result.hot.len() as u32;
            result.hot.extend(acc.into_iter().filter(|(_, k)| *k != 0.0).map(|((sm, h), k)| Hot {
                k,
                sm,
                h,
            }));
            result.hot_out.push((start, result.hot.len() as u32));
        }
        result
    }
}

/// Per-slot workspace; cloning a tape never aliases mutable evaluation buffers.
#[derive(Clone, Debug)]
struct Workspace {
    av: Vec<(f64, f64)>,
    hv: Vec<f64>,
    sv: Vec<f64>,
    state: Vec<f64>,
}

#[derive(Clone, Debug)]
pub struct Tape {
    program: Arc<Program>,
    coefficients: Arc<Coefficients>,
    work: Workspace,
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
        let mut tape = Program {
            harm: Vec::new(),
            smono: Vec::new(),
            pmono: Vec::new(),
            terms: Vec::new(),
            outputs: Vec::with_capacity(exprs.len()),
            angles: Vec::new(),
            harm_angle: Vec::new(),
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
        // sin and cos of one angle come from one `sin_cos`
        for (sin, ks) in &tape.harm {
            let key = |a: &Vec<(usize, f64)>| a.iter().map(|&(j, k)| (j, k.to_bits())).collect::<Vec<_>>();
            let at = tape.angles.iter().position(|a| key(a) == key(ks)).unwrap_or_else(|| {
                tape.angles.push(ks.clone());
                tape.angles.len() - 1
            });
            tape.harm_angle.push((at as u32, *sin));
        }
        let work = Workspace {
            av: vec![(0.0, 1.0); tape.angles.len()],
            hv: vec![0.0; tape.harm.len()],
            sv: vec![0.0; tape.smono.len()],
            state: vec![0.0; 2 * n],
        };
        let coefficients = Arc::new(Coefficients::new(&tape, None));
        Self { program: Arc::new(tape), coefficients, work }
    }

    /// Inline tape, workspace and replaceable coefficients; immutable compiled code is excluded.
    /// Shared coefficients are charged in full, so checkpoint estimates remain conservative.
    #[must_use]
    pub fn retained_bytes(&self) -> usize {
        size_of::<Self>() + self.heap_bytes()
    }

    pub(crate) fn heap_bytes(&self) -> usize {
        vec_bytes(&self.work.av)
            + vec_bytes(&self.work.hv)
            + vec_bytes(&self.work.sv)
            + vec_bytes(&self.work.state)
            + arc_bytes(&self.coefficients)
            + vec_bytes(&self.coefficients.hot)
            + vec_bytes(&self.coefficients.hot_out)
    }

    #[must_use]
    pub fn len(&self) -> usize {
        self.program.outputs.len()
    }

    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.program.outputs.is_empty()
    }

    #[must_use]
    pub fn terms(&self) -> usize {
        self.program.terms.len()
    }

    /// Fold the parameter values into the term coefficients.
    pub fn set_params(&mut self, p: &[f64]) {
        self.coefficients = Arc::new(Coefficients::new(&self.program, Some(p)));
    }

    /// (harmonics, distinct angles, state monomials, symbolic terms, merged terms): for profiling.
    #[must_use]
    pub fn stats(&self) -> (usize, usize, usize, usize, usize) {
        (
            self.program.harm.len(),
            self.program.angles.len(),
            self.program.smono.len(),
            self.program.terms.len(),
            self.coefficients.hot.len(),
        )
    }

    /// Evaluate every output at `(q, q̇)` into `out` (length ≥ `self.len()`).
    pub fn eval(&mut self, q: &[f64], qd: &[f64], out: &mut [f64]) {
        let n = self.program.n;
        self.work.state[..n].copy_from_slice(&q[..n]);
        self.work.state[n..2 * n].copy_from_slice(&qd[..n]);
        for (v, ks) in self.work.av.iter_mut().zip(&self.program.angles) {
            *v = if ks.is_empty() {
                (0.0, 1.0)
            } else {
                ks.iter().map(|&(j, k)| k * q[j]).sum::<f64>().sin_cos()
            };
        }
        for (v, &(a, sin)) in self.work.hv.iter_mut().zip(&self.program.harm_angle) {
            let (s, c) = self.work.av[a as usize];
            *v = if sin { s } else { c };
        }
        for (v, m) in self.work.sv.iter_mut().zip(&self.program.smono) {
            *v = m
                .iter()
                .map(|&(i, e)| if e == 1 { self.work.state[i] } else { self.work.state[i].powi(e) })
                .product();
        }
        for (o, &(a, b)) in out.iter_mut().zip(&self.coefficients.hot_out) {
            let mut acc = 0.0;
            for t in &self.coefficients.hot[a as usize..b as usize] {
                acc += t.k * self.work.sv[t.sm as usize] * self.work.hv[t.h as usize];
            }
            *o = acc;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{MAXN, mech::presets, model::Model};

    #[test]
    fn clones_share_compilation_but_isolate_parameters_and_evaluation() {
        let model = Model::from_desc(&presets::arm2());
        let mut a = model.dynamics.clone();
        let mut b = a.clone();
        assert!(Arc::ptr_eq(&a.program, &b.program));
        assert!(Arc::ptr_eq(&a.coefficients, &b.coefficients));
        let q = [0.3; MAXN];
        let mut expected = vec![0.0; a.len()];
        a.eval(&q, &[0.0; MAXN], &mut expected);
        let mut altered = vec![0.0; a.len()];
        b.eval(&[1.2; MAXN], &[0.5; MAXN], &mut altered);
        let mut actual = vec![0.0; a.len()];
        a.eval(&q, &[0.0; MAXN], &mut actual);
        assert_eq!(expected, actual);
        let mut params = model.defaults;
        params[0] *= 2.0;
        b.set_params(&params);
        assert!(Arc::ptr_eq(&a.program, &b.program));
        assert!(!Arc::ptr_eq(&a.coefficients, &b.coefficients));
        b.eval(&q, &[0.0; MAXN], &mut altered);
        assert_ne!(expected, altered);
        a.eval(&q, &[0.0; MAXN], &mut actual);
        assert_eq!(expected, actual);
    }
}
