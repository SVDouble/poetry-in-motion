//! Box-constrained quadratic programming for MPC:
//!
//! ```text
//! minimise ½ xᵀ H x + gᵀ x   subject to   lo ≤ x ≤ hi
//! ```
//!
//! solved by FISTA (accelerated projected gradient) with a warm start — exact enough for real-time MPC with
//! input bounds, allocation-free per solve, and simple to read.

/// Largest eigenvalue of a symmetric positive semi-definite matrix (power iteration; design time).
#[must_use]
pub fn lipschitz(n: usize, h: &[f64]) -> f64 {
    let mut v = vec![1.0 / (n as f64).sqrt(); n];
    let mut lambda = 0.0;
    for _ in 0..200 {
        let w: Vec<f64> = (0..n).map(|i| (0..n).map(|j| h[i * n + j] * v[j]).sum()).collect();
        let norm = w.iter().map(|x| x * x).sum::<f64>().sqrt();
        if norm < 1e-300 {
            return 1.0;
        }
        let next = norm;
        for (vi, wi) in v.iter_mut().zip(&w) {
            *vi = wi / norm;
        }
        if (next - lambda).abs() <= 1e-10 * next {
            return next;
        }
        lambda = next;
    }
    lambda.max(1e-12)
}

/// Solve the box QP in place (`x` holds the warm start on entry). `scratch` needs `3 n` entries.
/// Returns the number of iterations used.
#[allow(clippy::too_many_arguments)]
pub fn solve_box(
    h: &[f64],
    g: &[f64],
    lo: &[f64],
    hi: &[f64],
    lip: f64,
    x: &mut [f64],
    scratch: &mut [f64],
    max_iter: usize,
) -> usize {
    let n = x.len();
    let (y, rest) = scratch.split_at_mut(n);
    let (x_prev, grad) = rest.split_at_mut(n);
    y.copy_from_slice(x);
    x_prev.copy_from_slice(x);
    let step = 1.0 / lip;
    let mut t = 1.0_f64;
    for it in 0..max_iter {
        for i in 0..n {
            grad[i] = g[i] + (0..n).map(|j| h[i * n + j] * y[j]).sum::<f64>();
        }
        let mut change = 0.0_f64;
        for i in 0..n {
            let xi = (y[i] - step * grad[i]).clamp(lo[i], hi[i]);
            change = crate::mat::max_abs([change, xi - x[i]]);
            x_prev[i] = x[i];
            x[i] = xi;
        }
        let t_next = f64::midpoint(1.0, 4.0f64.mul_add(t * t, 1.0).sqrt());
        let beta = (t - 1.0) / t_next;
        for i in 0..n {
            y[i] = x[i] + beta * (x[i] - x_prev[i]);
        }
        t = t_next;
        // NaN (bad input) must not pass for convergence: stop and let the caller see it
        if change < 1e-9 || change.is_nan() {
            return it + 1;
        }
    }
    max_iter
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn nan_input_is_not_mistaken_for_convergence() {
        let h = [2.0, 0.0, 0.0, 2.0];
        let mut x = [0.0; 2];
        let mut scratch = [0.0; 6];
        solve_box(&h, &[f64::NAN, 1.0], &[-1.0; 2], &[1.0; 2], 2.0, &mut x, &mut scratch, 50);
        assert!(x[0].is_nan(), "NaN must surface, not be swallowed by max()");
    }

    #[test]
    fn unconstrained_matches_the_linear_solve() {
        // H = [[2, 1], [1, 3]], g = [−1, −2]  →  x = H⁻¹(−g) = [0.2, 0.6]
        let h = [2.0, 1.0, 1.0, 3.0];
        let mut x = [0.0; 2];
        let mut s = [0.0; 6];
        solve_box(&h, &[-1.0, -2.0], &[-10.0; 2], &[10.0; 2], lipschitz(2, &h), &mut x, &mut s, 500);
        assert!((x[0] - 0.2).abs() < 1e-6 && (x[1] - 0.6).abs() < 1e-6, "{x:?}");
    }

    #[test]
    fn active_bounds_are_respected() {
        let h = [1.0, 0.0, 0.0, 1.0];
        let mut x = [0.0; 2];
        let mut s = [0.0; 6];
        solve_box(&h, &[-5.0, 5.0], &[-1.0; 2], &[1.0; 2], lipschitz(2, &h), &mut x, &mut s, 500);
        assert!((x[0] - 1.0).abs() < 1e-9 && (x[1] + 1.0).abs() < 1e-9, "{x:?}");
    }
}
