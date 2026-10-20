//! Evaluation of the manipulator equation and forward dynamics.

use crate::MAXN;
use crate::mat::{cholesky_solve, solve_dense};
use crate::tape::Tape;

pub type VecN = [f64; MAXN];
pub type MatN = [f64; MAXN * MAXN];

/// M(q), C(q, q̇), N(q, q̇) at one state. Matrices are row-major with stride `nq`.
#[derive(Clone, Copy, Debug)]
pub struct Terms {
    pub m: MatN,
    pub c: MatN,
    pub n: VecN,
}

impl Terms {
    /// Evaluate the dynamics tape (outputs M, C, N in that order).
    pub fn eval(tape: &mut Tape, n: usize, q: &[f64], qd: &[f64]) -> Self {
        let mut buf = [0.0; 2 * MAXN * MAXN + MAXN];
        tape.eval(q, qd, &mut buf);
        let mut t = Self { m: [0.0; MAXN * MAXN], c: [0.0; MAXN * MAXN], n: [0.0; MAXN] };
        t.m[..n * n].copy_from_slice(&buf[..n * n]);
        t.c[..n * n].copy_from_slice(&buf[n * n..2 * n * n]);
        t.n[..n].copy_from_slice(&buf[2 * n * n..2 * n * n + n]);
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

/// Largest KKT system: n coordinates plus two constraints per loop closure.
pub const MAXK: usize = MAXN + 2 * crate::mech::MAXC;

/// Constraint quantities at one state: φ (m), A = ∂φ/∂q (m × n, stride n), Ȧ q̇ (m).
#[derive(Clone, Copy, Debug)]
pub struct Constraints {
    pub m: usize,
    pub phi: [f64; 2 * crate::mech::MAXC],
    pub a: [f64; 2 * crate::mech::MAXC * MAXN],
    pub adq: [f64; 2 * crate::mech::MAXC],
}

impl Constraints {
    pub fn eval(tape: &mut Tape, n: usize, m: usize, q: &[f64], qd: &[f64]) -> Self {
        let mut buf = [0.0; 2 * crate::mech::MAXC * (MAXN + 2)];
        tape.eval(q, qd, &mut buf);
        let mut c = Self {
            m,
            phi: [0.0; 2 * crate::mech::MAXC],
            a: [0.0; 2 * crate::mech::MAXC * MAXN],
            adq: [0.0; 2 * crate::mech::MAXC],
        };
        c.phi[..m].copy_from_slice(&buf[..m]);
        c.a[..m * n].copy_from_slice(&buf[m..m + m * n]);
        c.adq[..m].copy_from_slice(&buf[m + m * n..2 * m + m * n]);
        c
    }

    #[must_use]
    pub fn max_violation(&self) -> f64 {
        self.phi[..self.m].iter().fold(0.0, |acc, v| acc.max(v.abs()))
    }
}

/// Constrained forward dynamics (MLS ch. 6): solve
///
/// ```text
/// [ M  Aᵀ ] [ q̈ ]   [ τ − C q̇ − N              ]
/// [ A  0  ] [ λ  ] = [ −Ȧ q̇ − 2α A q̇ − α² φ ]
/// ```
///
/// i.e. `M q̈ + C q̇ + N + Aᵀ λ = τ` with the constraint acceleration stabilised (Baumgarte, rate `α`).
/// Returns `(q̈, λ)`.
#[must_use]
pub fn forward_constrained(
    n: usize,
    t: &Terms,
    k: &Constraints,
    qd: &[f64],
    tau: &[f64],
    coriolis: bool,
    alpha: f64,
) -> (VecN, [f64; 2 * crate::mech::MAXC]) {
    let m = k.m;
    let size = n + m;
    let cqd = if coriolis { t.c_qd(n, qd) } else { [0.0; MAXN] };
    let mut kkt = [0.0; MAXK * MAXK];
    let mut rhs = [0.0; MAXK];
    for i in 0..n {
        for j in 0..n {
            kkt[i * size + j] = t.m[i * n + j];
        }
        rhs[i] = tau[i] - cqd[i] - t.n[i];
    }
    for r in 0..m {
        let aqd: f64 = (0..n).map(|j| k.a[r * n + j] * qd[j]).sum();
        for j in 0..n {
            kkt[(n + r) * size + j] = k.a[r * n + j];
            kkt[j * size + n + r] = k.a[r * n + j];
        }
        kkt[(n + r) * size + n + r] = -1e-10; // tiny regularisation for redundant constraints
        rhs[n + r] = -k.adq[r] - 2.0 * alpha * aqd - alpha * alpha * k.phi[r];
    }
    let mut qdd = [f64::NAN; MAXN];
    let mut lambda = [0.0; 2 * crate::mech::MAXC];
    if solve_dense(size, &mut kkt[..size * size], &mut rhs[..size]) {
        qdd[..n].copy_from_slice(&rhs[..n]);
        lambda[..m].copy_from_slice(&rhs[n..size]);
    }
    (qdd, lambda)
}

/// Eigenvalues of a symmetric `n × n` matrix by cyclic Jacobi rotations (ascending).
#[must_use]
pub fn sym_eigenvalues(n: usize, a: &[f64]) -> VecN {
    let mut m = [0.0; MAXN * MAXN];
    m[..n * n].copy_from_slice(&a[..n * n]);
    for _ in 0..50 {
        let off: f64 = (0..n)
            .flat_map(|i| (0..n).filter(move |&j| j != i).map(move |j| (i, j)))
            .map(|(i, j)| m[i * n + j].powi(2))
            .sum();
        if off < 1e-22 {
            break;
        }
        for p in 0..n {
            for q in p + 1..n {
                let apq = m[p * n + q];
                if apq.abs() < 1e-300 {
                    continue;
                }
                let theta = (m[q * n + q] - m[p * n + p]) / (2.0 * apq);
                let t = theta.signum() / (theta.abs() + (theta * theta + 1.0).sqrt());
                let t = if theta == 0.0 { 1.0 } else { t };
                let c = (t * t + 1.0).sqrt().recip();
                let s = t * c;
                for k in 0..n {
                    let (akp, akq) = (m[k * n + p], m[k * n + q]);
                    m[k * n + p] = c * akp - s * akq;
                    m[k * n + q] = s * akp + c * akq;
                }
                for k in 0..n {
                    let (apk, aqk) = (m[p * n + k], m[q * n + k]);
                    m[p * n + k] = c * apk - s * aqk;
                    m[q * n + k] = s * apk + c * aqk;
                }
            }
        }
    }
    let mut ev = [f64::NAN; MAXN];
    for i in 0..n {
        ev[i] = m[i * n + i];
    }
    ev[..n].sort_by(f64::total_cmp);
    ev
}

#[cfg(test)]
mod tests {
    #[test]
    fn jacobi_eigenvalues() {
        let ev = super::sym_eigenvalues(2, &[2.0, 1.0, 1.0, 2.0]);
        assert!((ev[0] - 1.0).abs() < 1e-12 && (ev[1] - 3.0).abs() < 1e-12);
    }
}
