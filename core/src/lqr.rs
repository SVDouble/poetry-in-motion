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

use crate::mat::DMat;

/// Zero-order-hold discretisation by truncated Taylor series of `exp([[A, B], [0, 0]] dt)`.
#[must_use]
pub fn c2d(a: &DMat, b: &DMat, dt: f64) -> (DMat, DMat) {
    let n = a.rows;
    let ad_dt = a.scale(dt);
    let mut term = DMat::identity(n); // (A dt)^k / k!
    let mut ad = DMat::identity(n);
    let mut gamma = DMat::identity(n).scale(dt); // Σ (A dt)^k dt / (k+1)!
    for k in 1..16 {
        term = (&term * &ad_dt).scale(1.0 / f64::from(k));
        ad = &ad + &term;
        gamma = &gamma + &term.scale(dt / f64::from(k + 1));
    }
    (ad, &gamma * b)
}

/// Solve the DARE; returns `P` or `None` if the iteration breaks down.
#[must_use]
pub fn dare(a: &DMat, b: &DMat, q: &DMat, r: &DMat) -> Option<DMat> {
    let id = DMat::identity(a.rows);
    let mut ak = a.clone();
    let mut gk = &(b * &r.inverse()?) * &b.t();
    let mut hk = q.clone();
    for _ in 0..64 {
        let w = (&id + &(&gk * &hk)).inverse()?;
        let akw = &ak * &w;
        let a1 = &akw * &ak;
        let g1 = &gk + &(&(&akw * &gk) * &ak.t());
        let h1 = &hk + &(&(&(&ak.t() * &hk) * &w) * &ak);
        let delta = (&h1 - &hk).max_abs();
        ak = a1;
        gk = g1;
        hk = h1;
        if !delta.is_finite() {
            return None;
        }
        if delta <= 1e-12 * hk.max_abs().max(1.0) {
            return Some(hk);
        }
    }
    None
}

/// Discrete LQR gain `K` for `u = −K x`.
#[must_use]
pub fn dlqr(a: &DMat, b: &DMat, q: &DMat, r: &DMat) -> Option<DMat> {
    let p = dare(a, b, q, r)?;
    let bt_p = &b.t() * &p;
    Some(&(r + &(&bt_p * b)).inverse()? * &(&bt_p * a))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn double_integrator_matches_closed_form() {
        // ẍ = u, Q = I, R = 1  →  continuous K = [1, √3]
        let a = DMat { rows: 2, cols: 2, data: vec![0.0, 1.0, 0.0, 0.0] };
        let b = DMat { rows: 2, cols: 1, data: vec![0.0, 1.0] };
        let dt = 1e-3;
        let (ad, bd) = c2d(&a, &b, dt);
        let k = dlqr(&ad, &bd, &DMat::identity(2).scale(dt), &DMat::identity(1).scale(dt)).unwrap();
        assert!((k[(0, 0)] - 1.0).abs() < 2e-3, "{k:?}");
        assert!((k[(0, 1)] - 3f64.sqrt()).abs() < 2e-3, "{k:?}");
    }

    #[test]
    fn dare_residual_is_small() {
        let a = DMat { rows: 2, cols: 2, data: vec![1.1, 0.2, 0.0, 0.95] };
        let b = DMat { rows: 2, cols: 1, data: vec![0.0, 0.1] };
        let q = DMat::identity(2);
        let r = DMat { rows: 1, cols: 1, data: vec![0.5] };
        let p = dare(&a, &b, &q, &r).unwrap();
        let k = &(&r + &(&(&b.t() * &p) * &b)).inverse().unwrap() * &(&(&b.t() * &p) * &a);
        let res = &(&(&(&a.t() * &p) * &a) - &(&(&(&a.t() * &p) * &b) * &k)) + &(&q - &p);
        assert!(res.max_abs() < 1e-9);
    }
}
