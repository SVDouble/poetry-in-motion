//! Tiny const-generic dense matrices: stack-allocated, fully unrolled by the compiler.

use core::ops::{Add, Index, IndexMut, Mul, Neg, Sub};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Mat<const R: usize, const C: usize>(pub [[f64; C]; R]);

pub type Vector<const N: usize> = Mat<N, 1>;

impl<const R: usize, const C: usize> Default for Mat<R, C> {
    fn default() -> Self {
        Self::ZERO
    }
}

impl<const R: usize, const C: usize> Mat<R, C> {
    pub const ZERO: Self = Self([[0.0; C]; R]);

    #[must_use]
    pub fn from_fn(f: impl Fn(usize, usize) -> f64) -> Self {
        Self(core::array::from_fn(|i| core::array::from_fn(|j| f(i, j))))
    }

    #[must_use]
    pub fn t(&self) -> Mat<C, R> {
        Mat::from_fn(|i, j| self.0[j][i])
    }

    #[must_use]
    pub fn scale(&self, s: f64) -> Self {
        Self::from_fn(|i, j| self.0[i][j] * s)
    }

    #[must_use]
    pub fn max_abs(&self) -> f64 {
        self.0.iter().flatten().fold(0.0, |m, v| m.max(v.abs()))
    }

    /// Row-major iterator over all entries.
    pub fn iter(&self) -> impl Iterator<Item = f64> + '_ {
        self.0.iter().flatten().copied()
    }
}

impl<const N: usize> Mat<N, N> {
    #[must_use]
    pub fn identity() -> Self {
        Self::from_fn(|i, j| if i == j { 1.0 } else { 0.0 })
    }

    /// Gauss–Jordan inverse with partial pivoting; `None` if (numerically) singular.
    #[must_use]
    pub fn inverse(&self) -> Option<Self> {
        let mut a = self.0;
        let mut inv = Self::identity().0;
        for col in 0..N {
            let piv = (col..N).max_by(|&i, &j| a[i][col].abs().total_cmp(&a[j][col].abs()))?;
            if a[piv][col].abs() < 1e-300 {
                return None;
            }
            a.swap(col, piv);
            inv.swap(col, piv);
            let d = a[col][col].recip();
            for j in 0..N {
                a[col][j] *= d;
                inv[col][j] *= d;
            }
            for i in (0..N).filter(|&i| i != col) {
                let f = a[i][col];
                if f != 0.0 {
                    for j in 0..N {
                        a[i][j] -= f * a[col][j];
                        inv[i][j] -= f * inv[col][j];
                    }
                }
            }
        }
        Some(Self(inv))
    }
}

impl<const R: usize, const C: usize> Index<(usize, usize)> for Mat<R, C> {
    type Output = f64;
    fn index(&self, (i, j): (usize, usize)) -> &f64 {
        &self.0[i][j]
    }
}

impl<const R: usize, const C: usize> IndexMut<(usize, usize)> for Mat<R, C> {
    fn index_mut(&mut self, (i, j): (usize, usize)) -> &mut f64 {
        &mut self.0[i][j]
    }
}

impl<const R: usize, const C: usize> Add for Mat<R, C> {
    type Output = Self;
    fn add(self, o: Self) -> Self {
        Self::from_fn(|i, j| self.0[i][j] + o.0[i][j])
    }
}

impl<const R: usize, const C: usize> Sub for Mat<R, C> {
    type Output = Self;
    fn sub(self, o: Self) -> Self {
        Self::from_fn(|i, j| self.0[i][j] - o.0[i][j])
    }
}

impl<const R: usize, const C: usize> Neg for Mat<R, C> {
    type Output = Self;
    fn neg(self) -> Self {
        self.scale(-1.0)
    }
}

impl<const R: usize, const K: usize, const C: usize> Mul<Mat<K, C>> for Mat<R, K> {
    type Output = Mat<R, C>;
    fn mul(self, o: Mat<K, C>) -> Mat<R, C> {
        Mat::from_fn(|i, j| (0..K).map(|k| self.0[i][k] * o.0[k][j]).sum())
    }
}

/// Solve `A x = b` for a symmetric positive-definite `n × n` matrix stored row-major with stride `n`.
/// Returns `false` if `A` is not positive definite. Allocation free; `n ≤ MAXN`.
#[must_use]
pub fn cholesky_solve(n: usize, a: &[f64], b: &[f64], x: &mut [f64]) -> bool {
    let mut l = [0.0; crate::MAXN * crate::MAXN];
    for i in 0..n {
        for j in 0..=i {
            let s: f64 = a[i * n + j] - (0..j).map(|k| l[i * n + k] * l[j * n + k]).sum::<f64>();
            if i == j {
                if s <= 0.0 {
                    return false;
                }
                l[i * n + i] = s.sqrt();
            } else {
                l[i * n + j] = s / l[j * n + j];
            }
        }
    }
    // forward: L y = b, then backward: Lᵀ x = y
    for i in 0..n {
        x[i] = (b[i] - (0..i).map(|k| l[i * n + k] * x[k]).sum::<f64>()) / l[i * n + i];
    }
    for i in (0..n).rev() {
        x[i] = (x[i] - (i + 1..n).map(|k| l[k * n + i] * x[k]).sum::<f64>()) / l[i * n + i];
    }
    true
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn inverse_roundtrip() {
        let a = Mat([[4.0, 1.0, 2.0], [0.5, 3.0, 1.0], [2.0, -1.0, 5.0]]);
        let i = a * a.inverse().unwrap();
        assert!((i - Mat::identity()).max_abs() < 1e-12);
    }

    #[test]
    fn cholesky_matches_inverse() {
        let a = [4.0, 1.0, 1.0, 3.0];
        let b = [1.0, 2.0];
        let mut x = [0.0; 2];
        assert!(cholesky_solve(2, &a, &b, &mut x));
        assert!((4.0 * x[0] + x[1] - 1.0).abs() < 1e-12);
        assert!((x[0] + 3.0 * x[1] - 2.0).abs() < 1e-12);
    }
}
