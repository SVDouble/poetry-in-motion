//! The manipulator equation, loop-closure constraints and the solvers shared by the simulator and the
//! controllers (allocation-free unless noted).

use crate::MAXN;
use crate::mat::{DMat, cholesky_solve, solve_dense};
use crate::mech::MAXM;
use crate::model::Model;
use crate::tape::Tape;

pub type VecN = [f64; MAXN];
pub type MatN = [f64; MAXN * MAXN];
pub type VecM = [f64; MAXM];
/// Largest KKT / least-squares system: n coordinates plus m constraints.
pub const MAXK: usize = MAXN + MAXM;

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

    /// `h = C(q, q̇) q̇ + N(q, q̇)`
    #[must_use]
    pub fn h(&self, nq: usize, qd: &[f64]) -> VecN {
        let c = self.c_qd(nq, qd);
        core::array::from_fn(|i| if i < nq { c[i] + self.n[i] } else { 0.0 })
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

/// Loop-closure constraints at one state: φ (m), A = ∂φ/∂q (m × n, stride n), Ȧ q̇ (m).
#[derive(Clone, Copy, Debug)]
pub struct Constraints {
    pub n: usize,
    pub m: usize,
    pub phi: VecM,
    pub a: [f64; MAXM * MAXN],
    pub adq: VecM,
}

impl Constraints {
    pub fn eval(tape: &mut Tape, n: usize, m: usize, q: &[f64], qd: &[f64]) -> Self {
        let mut buf = [0.0; MAXM * (MAXN + 2)];
        tape.eval(q, qd, &mut buf);
        let mut c = Self { n, m, phi: [0.0; MAXM], a: [0.0; MAXM * MAXN], adq: [0.0; MAXM] };
        c.phi[..m].copy_from_slice(&buf[..m]);
        c.a[..m * n].copy_from_slice(&buf[m..m + m * n]);
        c.adq[..m].copy_from_slice(&buf[m + m * n..2 * m + m * n]);
        c
    }

    #[must_use]
    pub fn max_violation(&self) -> f64 {
        crate::mat::max_abs(self.phi[..self.m].iter().copied())
    }

    /// `A x`
    #[must_use]
    pub fn apply(&self, x: &[f64]) -> VecM {
        core::array::from_fn(|r| {
            if r < self.m { (0..self.n).map(|j| self.a[r * self.n + j] * x[j]).sum() } else { 0.0 }
        })
    }

    /// `Aᵀ y`
    #[must_use]
    pub fn apply_t(&self, y: &[f64]) -> VecN {
        core::array::from_fn(|j| {
            if j < self.n { (0..self.m).map(|r| self.a[r * self.n + j] * y[r]).sum() } else { 0.0 }
        })
    }

    /// Minimal-norm solution of `A x = r`: `x = Aᵀ (A Aᵀ + εI)⁻¹ r`.
    #[must_use]
    pub fn min_norm(&self, r: &[f64]) -> VecN {
        let m = self.m;
        let mut aat = [0.0; MAXM * MAXM];
        let mut y = [0.0; MAXM];
        for i in 0..m {
            for j in 0..m {
                aat[i * m + j] =
                    (0..self.n).map(|c| self.a[i * self.n + c] * self.a[j * self.n + c]).sum::<f64>()
                        + if i == j { 1e-12 } else { 0.0 };
            }
            y[i] = r[i];
        }
        if solve_dense(m, &mut aat[..m * m], &mut y[..m]) { self.apply_t(&y) } else { [0.0; MAXN] }
    }

    /// The acceleration closest to `v` that is consistent with the constraints: `A q̈ + Ȧ q̇ = 0`.
    #[must_use]
    pub fn consistent(&self, v: &[f64]) -> VecN {
        let av = self.apply(v);
        let r: VecM = core::array::from_fn(|i| av[i] + self.adq[i]);
        let d = self.min_norm(&r);
        core::array::from_fn(|i| if i < self.n { v[i] - d[i] } else { 0.0 })
    }

    /// Orthonormal basis Z (n × r) of the tangent space `null(A)` (design time; allocates).
    #[must_use]
    pub fn tangent_basis(&self) -> DMat {
        let n = self.n;
        let mut cols: Vec<VecN> = Vec::new();
        for e in 0..n {
            // project the unit vector onto null(A), then Gram–Schmidt against the columns found so far
            let mut v = [0.0; MAXN];
            v[e] = 1.0;
            let d = self.min_norm(&self.apply(&v));
            for i in 0..n {
                v[i] -= d[i];
            }
            for c in &cols {
                let dot: f64 = (0..n).map(|i| v[i] * c[i]).sum();
                for i in 0..n {
                    v[i] -= dot * c[i];
                }
            }
            let norm = (0..n).map(|i| v[i] * v[i]).sum::<f64>().sqrt();
            if norm > 1e-6 {
                cols.push(core::array::from_fn(|i| v[i] / norm));
            }
        }
        DMat::from_fn(n, cols.len(), |i, j| cols[j][i])
    }
}

/// Forward dynamics of an open chain: q̈ = M⁻¹ (τ − C q̇ − N). `coriolis = false` drops C q̇ ("what-if").
#[must_use]
pub fn forward(nq: usize, t: &Terms, qd: &[f64], tau: &[f64], coriolis: bool) -> VecN {
    let cqd = if coriolis { t.c_qd(nq, qd) } else { [0.0; MAXN] };
    let rhs: VecN = core::array::from_fn(|i| if i < nq { tau[i] - cqd[i] - t.n[i] } else { 0.0 });
    let mut qdd = [0.0; MAXN];
    if !cholesky_solve(nq, &t.m, &rhs, &mut qdd) {
        qdd = [f64::NAN; MAXN];
    }
    qdd
}

/// Constrained forward dynamics (MLS ch. 6): solve
///
/// ```text
/// [ M  Aᵀ ] [ q̈ ]   [ τ − C q̇ − N              ]
/// [ A  0  ] [ λ  ] = [ −Ȧ q̇ − 2α A q̇ − α² φ ]
/// ```
///
/// i.e. `M q̈ + C q̇ + N + Aᵀ λ = τ` with the constraint acceleration stabilised (Baumgarte, rate `α`).
#[must_use]
pub fn forward_constrained(
    t: &Terms,
    k: &Constraints,
    qd: &[f64],
    tau: &[f64],
    coriolis: bool,
    alpha: f64,
) -> (VecN, VecM) {
    let (n, m) = (k.n, k.m);
    let size = n + m;
    let cqd = if coriolis { t.c_qd(n, qd) } else { [0.0; MAXN] };
    let aqd = k.apply(qd);
    let mut kkt = [0.0; MAXK * MAXK];
    let mut rhs = [0.0; MAXK];
    for i in 0..n {
        for j in 0..n {
            kkt[i * size + j] = t.m[i * n + j];
        }
        rhs[i] = tau[i] - cqd[i] - t.n[i];
    }
    for r in 0..m {
        for j in 0..n {
            kkt[(n + r) * size + j] = k.a[r * n + j];
            kkt[j * size + n + r] = k.a[r * n + j];
        }
        kkt[(n + r) * size + n + r] = -1e-10; // tiny regularisation for redundant constraints
        rhs[n + r] = -k.adq[r] - 2.0 * alpha * aqd[r] - alpha * alpha * k.phi[r];
    }
    let mut qdd = [f64::NAN; MAXN];
    let mut lambda = [0.0; MAXM];
    if solve_dense(size, &mut kkt[..size * size], &mut rhs[..size]) {
        qdd[..n].copy_from_slice(&rhs[..n]);
        lambda[..m].copy_from_slice(&rhs[n..size]);
    }
    (qdd, lambda)
}

/// Baumgarte stabilisation rate of the loop-closure constraints (1/s).
pub const BAUMGARTE: f64 = 20.0;

/// Constraints of a model at a state, if it has loops.
pub fn constraints(model: &mut Model, q: &[f64], qd: &[f64]) -> Option<Constraints> {
    let (n, m) = (model.n, model.ncon);
    model.constraints.as_mut().map(|t| Constraints::eval(t, n, m, q, qd))
}

/// Forward dynamics of any model (open chain or with loops): `(q̈, λ)`.
pub fn accel(model: &mut Model, q: &[f64], qd: &[f64], tau: &[f64], coriolis: bool) -> (VecN, VecM) {
    let t = model.terms(q, qd);
    let k = constraints(model, q, qd);
    accel_with(model.n, &t, k.as_ref(), qd, tau, coriolis)
}

/// [`accel`] with M, C, N and the constraints already evaluated at the state.
#[must_use]
pub fn accel_with(
    n: usize,
    t: &Terms,
    k: Option<&Constraints>,
    qd: &[f64],
    tau: &[f64],
    coriolis: bool,
) -> (VecN, VecM) {
    k.map_or_else(
        || (forward(n, t, qd, tau, coriolis), [0.0; MAXM]),
        |k| forward_constrained(t, k, qd, tau, coriolis, BAUMGARTE),
    )
}

/// Move a state onto the constraint manifold: Gauss–Newton on φ(q) = 0 with minimal-norm steps, then remove
/// the velocity component that violates A(q) q̇ = 0. No-op for open chains.
pub fn project(model: &mut Model, q: &mut VecN, qd: &mut VecN) {
    project_holding(model, q, qd, &[]);
}

/// Like [`project`], but only the coordinates not listed in `held` move (when that can close the loops).
pub fn project_holding(model: &mut Model, q: &mut VecN, qd: &mut VecN, held: &[usize]) {
    let n = model.n;
    let zero = [0.0; MAXN];
    for _ in 0..100 {
        let Some(mut k) = constraints(model, q, &zero) else { return };
        if k.max_violation() < 1e-12 {
            break;
        }
        for r in 0..k.m {
            for &c in held {
                k.a[r * n + c] = 0.0;
            }
        }
        let d = k.min_norm(&k.phi);
        for c in 0..n {
            q[c] -= d[c].clamp(-0.5, 0.5);
        }
    }
    if let Some(k) = constraints(model, q, &zero) {
        let d = k.min_norm(&k.apply(qd));
        for c in 0..n {
            qd[c] -= d[c];
        }
    }
}

/// Least squares `min ‖G x − b‖` (minimal norm) for `G` of size `rows × cols` (row-major), by regularised
/// normal equations. Returns `(x, residual)`.
#[must_use]
pub fn lstsq(rows: usize, cols: usize, g: &[f64], b: &[f64]) -> ([f64; MAXK], f64) {
    let mut gtg = [0.0; MAXK * MAXK];
    let mut x = [0.0; MAXK];
    for i in 0..cols {
        for j in 0..cols {
            gtg[i * cols + j] = (0..rows).map(|r| g[r * cols + i] * g[r * cols + j]).sum::<f64>()
                + if i == j { 1e-10 } else { 0.0 };
        }
        x[i] = (0..rows).map(|r| g[r * cols + i] * b[r]).sum();
    }
    if !solve_dense(cols, &mut gtg[..cols * cols], &mut x[..cols]) {
        return ([0.0; MAXK], f64::INFINITY);
    }
    let res =
        crate::mat::max_abs((0..rows).map(|r| (0..cols).map(|c| g[r * cols + c] * x[c]).sum::<f64>() - b[r]));
    (x, res)
}

/// Inputs that produce the generalised force `rhs` given the constraints: solve `B u − Aᵀ λ = rhs` for `(u, λ)`
/// in the least-squares sense. Returns `(u, residual)`; the residual is 0 when `rhs` is achievable.
#[must_use]
pub fn inverse_dynamics(model: &Model, k: Option<&Constraints>, rhs: &[f64]) -> (VecN, f64) {
    let (n, nu) = (model.n, model.nu);
    let m = k.map_or(0, |k| k.m);
    let cols = nu + m;
    let mut g = [0.0; MAXN * MAXK];
    for i in 0..n {
        for a in 0..nu {
            g[i * cols + a] = model.b[i * nu + a];
        }
        if let Some(k) = k {
            for r in 0..m {
                g[i * cols + nu + r] = -k.a[r * n + i];
            }
        }
    }
    let (x, res) = lstsq(n, cols, &g, rhs);
    (core::array::from_fn(|a| if a < nu { x[a] } else { 0.0 }), res)
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

    #[test]
    fn least_squares_solves_consistent_systems() {
        // 2 × 3, rank 2: minimal-norm exact solution
        let g = [1.0, 0.0, 1.0, 0.0, 1.0, 1.0];
        let (x, res) = super::lstsq(2, 3, &g, &[2.0, 3.0]);
        assert!(res < 1e-8, "{res}");
        assert!((x[0] + x[2] - 2.0).abs() < 1e-8 && (x[1] + x[2] - 3.0).abs() < 1e-8);
    }
}
