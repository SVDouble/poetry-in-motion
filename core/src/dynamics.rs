//! Evaluation of the manipulator equation and forward dynamics.

use crate::MAXN;
use crate::mat::cholesky_solve;
use crate::model::Model;

pub type VecN = [f64; MAXN];
pub type MatN = [f64; MAXN * MAXN];

/// M(q), C(q, q̇), N(q, q̇) at one state. Matrices are row-major with stride `nq`.
#[derive(Clone, Copy, Debug, Default)]
pub struct Terms {
    pub m: MatN,
    pub c: MatN,
    pub n: VecN,
}

impl Terms {
    #[must_use]
    pub fn eval(model: &Model, p: &[f64], q: &[f64], qd: &[f64]) -> Self {
        let mut t = Self::default();
        (model.mcn)(p, q, qd, &mut t.m, &mut t.c, &mut t.n);
        t
    }

    /// `C(q, q̇) q̇`
    #[must_use]
    pub fn c_qd(&self, nq: usize, qd: &[f64]) -> VecN {
        mat_vec(nq, &self.c, qd)
    }
}

#[must_use]
pub fn mat_vec(n: usize, a: &[f64], x: &[f64]) -> VecN {
    let mut y = [0.0; MAXN];
    for i in 0..n {
        y[i] = (0..n).map(|j| a[i * n + j] * x[j]).sum();
    }
    y
}

/// Forward dynamics: q̈ = M⁻¹ (τ − C q̇ − N). `coriolis = false` drops the C q̇ term (a "what-if" toggle).
#[must_use]
pub fn forward(nq: usize, t: &Terms, qd: &[f64], tau: &[f64], coriolis: bool) -> VecN {
    let cqd = if coriolis { t.c_qd(nq, qd) } else { [0.0; MAXN] };
    let mut rhs = [0.0; MAXN];
    for i in 0..nq {
        rhs[i] = tau[i] - cqd[i] - t.n[i];
    }
    let mut qdd = [0.0; MAXN];
    if !cholesky_solve(nq, &t.m, &rhs, &mut qdd) {
        qdd = [f64::NAN; MAXN];
    }
    qdd
}
