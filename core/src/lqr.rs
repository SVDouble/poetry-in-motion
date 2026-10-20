//! Linear-quadratic regulator via the structure-preserving doubling algorithm (SDA).
//!
//! The continuous plant ẋ = A x + B u is discretised with the simulation step (zero-order hold), and the
//! discrete algebraic Riccati equation
//!
//! ```text
//! P = AᵀPA − AᵀPB (R + BᵀPB)⁻¹ BᵀPA + Q
//! ```
//!
//! is solved by doubling, which converges quadratically (≈ 20–40 iterations instead of thousands).

use crate::mat::Mat;

/// Zero-order-hold discretisation by truncated Taylor series of `exp([[A, B], [0, 0]] dt)`.
#[must_use]
pub fn c2d<const N: usize, const M: usize>(a: &Mat<N, N>, b: &Mat<N, M>, dt: f64) -> (Mat<N, N>, Mat<N, M>) {
    let ad_dt = a.scale(dt);
    let mut term = Mat::<N, N>::identity(); // (A dt)^k / k!
    let mut ad = Mat::<N, N>::identity();
    let mut gamma = Mat::<N, N>::identity().scale(dt); // Σ (A dt)^k dt / (k+1)!
    for k in 1..16 {
        term = (term * ad_dt).scale(1.0 / f64::from(k));
        ad = ad + term;
        gamma = gamma + term.scale(dt / f64::from(k + 1));
    }
    (ad, gamma * *b)
}

/// Solve the DARE; returns `P` or `None` if the iteration breaks down.
#[must_use]
pub fn dare<const N: usize, const M: usize>(
    a: &Mat<N, N>,
    b: &Mat<N, M>,
    q: &Mat<N, N>,
    r: &Mat<M, M>,
) -> Option<Mat<N, N>> {
    let id = Mat::<N, N>::identity();
    let mut ak = *a;
    let mut gk = *b * r.inverse()? * b.t();
    let mut hk = *q;
    for _ in 0..64 {
        let w = (id + gk * hk).inverse()?;
        let a1 = ak * w * ak;
        let g1 = gk + ak * w * gk * ak.t();
        let h1 = hk + ak.t() * hk * w * ak;
        let delta = (h1 - hk).max_abs();
        ak = a1;
        gk = g1;
        hk = h1;
        if delta <= 1e-12 * hk.max_abs().max(1.0) {
            return Some(hk);
        }
        if !delta.is_finite() {
            return None;
        }
    }
    None
}

/// Discrete LQR gain `K` for `u = −K x`.
#[must_use]
pub fn dlqr<const N: usize, const M: usize>(
    a: &Mat<N, N>,
    b: &Mat<N, M>,
    q: &Mat<N, N>,
    r: &Mat<M, M>,
) -> Option<Mat<M, N>> {
    let p = dare(a, b, q, r)?;
    Some((*r + b.t() * p * *b).inverse()? * b.t() * p * *a)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn double_integrator_matches_closed_form() {
        // ẍ = u, Q = I, R = 1  →  continuous K = [1, √3]
        let a = Mat([[0.0, 1.0], [0.0, 0.0]]);
        let b = Mat([[0.0], [1.0]]);
        let dt = 1e-3;
        let (ad, bd) = c2d(&a, &b, dt);
        let k = dlqr(&ad, &bd, &Mat::identity().scale(dt), &Mat([[dt]])).unwrap();
        assert!((k[(0, 0)] - 1.0).abs() < 2e-3, "{k:?}");
        assert!((k[(0, 1)] - 3f64.sqrt()).abs() < 2e-3, "{k:?}");
    }

    #[test]
    fn dare_residual_is_small() {
        let a = Mat([[1.1, 0.2], [0.0, 0.95]]);
        let b = Mat([[0.0], [0.1]]);
        let q = Mat::identity();
        let r = Mat([[0.5]]);
        let p = dare(&a, &b, &q, &r).unwrap();
        let res =
            a.t() * p * a - a.t() * p * b * (r + b.t() * p * b).inverse().unwrap() * b.t() * p * a + q - p;
        assert!(res.max_abs() < 1e-9);
    }
}
