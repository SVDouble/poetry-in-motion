//! Small dense linear algebra: a dynamic matrix for (rare) controller design and an allocation-free
//! Cholesky solve for the (hot) forward dynamics.

use core::ops::{Add, Index, IndexMut, Mul, Sub};

use crate::MAXN;

/// Largest |x| of a sequence, propagating NaN: `f64::max` silently drops NaN, which would let a diverged
/// iteration look converged and a broken state look fine.
#[must_use]
pub fn max_abs(xs: impl IntoIterator<Item = f64>) -> f64 {
    xs.into_iter().fold(0.0, |m: f64, v| if m.is_nan() || v.is_nan() { f64::NAN } else { m.max(v.abs()) })
}

/// Row-major dynamic matrix.
#[derive(Clone, Debug, PartialEq)]
pub struct DMat {
    pub rows: usize,
    pub cols: usize,
    pub data: Vec<f64>,
}

impl DMat {
    #[must_use]
    pub fn zeros(rows: usize, cols: usize) -> Self {
        Self { rows, cols, data: vec![0.0; rows * cols] }
    }

    #[must_use]
    pub fn identity(n: usize) -> Self {
        let mut m = Self::zeros(n, n);
        for i in 0..n {
            m[(i, i)] = 1.0;
        }
        m
    }

    #[must_use]
    pub fn from_fn(rows: usize, cols: usize, f: impl Fn(usize, usize) -> f64) -> Self {
        let mut m = Self::zeros(rows, cols);
        for i in 0..rows {
            for j in 0..cols {
                m[(i, j)] = f(i, j);
            }
        }
        m
    }

    #[must_use]
    pub fn t(&self) -> Self {
        Self::from_fn(self.cols, self.rows, |i, j| self[(j, i)])
    }

    #[must_use]
    pub fn scale(&self, s: f64) -> Self {
        Self { rows: self.rows, cols: self.cols, data: self.data.iter().map(|x| x * s).collect() }
    }

    /// Largest |entry|; NaN if any entry is NaN (so that convergence tests cannot pass on garbage).
    #[must_use]
    pub fn max_abs(&self) -> f64 {
        max_abs(self.data.iter().copied())
    }

    /// Gauss–Jordan inverse with partial pivoting; `None` if (numerically) singular.
    #[must_use]
    pub fn inverse(&self) -> Option<Self> {
        let n = self.rows;
        let mut a = self.clone();
        let mut inv = Self::identity(n);
        for col in 0..n {
            let piv = (col..n).max_by(|&i, &j| a[(i, col)].abs().total_cmp(&a[(j, col)].abs()))?;
            if a[(piv, col)].abs() < 1e-300 {
                return None;
            }
            for j in 0..n {
                a.data.swap(col * n + j, piv * n + j);
                inv.data.swap(col * n + j, piv * n + j);
            }
            let d = a[(col, col)].recip();
            for j in 0..n {
                a[(col, j)] *= d;
                inv[(col, j)] *= d;
            }
            for i in (0..n).filter(|&i| i != col) {
                let f = a[(i, col)];
                if f != 0.0 {
                    for j in 0..n {
                        a[(i, j)] -= f * a[(col, j)];
                        inv[(i, j)] -= f * inv[(col, j)];
                    }
                }
            }
        }
        Some(inv)
    }
}

impl Index<(usize, usize)> for DMat {
    type Output = f64;
    fn index(&self, (i, j): (usize, usize)) -> &f64 {
        &self.data[i * self.cols + j]
    }
}

impl IndexMut<(usize, usize)> for DMat {
    fn index_mut(&mut self, (i, j): (usize, usize)) -> &mut f64 {
        &mut self.data[i * self.cols + j]
    }
}

impl Add for &DMat {
    type Output = DMat;
    fn add(self, o: &DMat) -> DMat {
        DMat {
            rows: self.rows,
            cols: self.cols,
            data: self.data.iter().zip(&o.data).map(|(a, b)| a + b).collect(),
        }
    }
}

impl Sub for &DMat {
    type Output = DMat;
    fn sub(self, o: &DMat) -> DMat {
        DMat {
            rows: self.rows,
            cols: self.cols,
            data: self.data.iter().zip(&o.data).map(|(a, b)| a - b).collect(),
        }
    }
}

impl Mul for &DMat {
    type Output = DMat;
    fn mul(self, o: &DMat) -> DMat {
        debug_assert_eq!(self.cols, o.rows);
        let mut r = DMat::zeros(self.rows, o.cols);
        for i in 0..self.rows {
            for k in 0..self.cols {
                let a = self[(i, k)];
                if a != 0.0 {
                    for j in 0..o.cols {
                        r[(i, j)] += a * o[(k, j)];
                    }
                }
            }
        }
        r
    }
}

/// Solve `A x = b` for a symmetric positive-definite `n × n` matrix stored row-major with stride `n`.
/// Returns `false` if `A` is not positive definite. Allocation free; `n ≤ MAXN`.
#[must_use]
pub fn cholesky_solve(n: usize, a: &[f64], b: &[f64], x: &mut [f64]) -> bool {
    let mut l = [0.0; MAXN * MAXN];
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
    for i in 0..n {
        x[i] = (b[i] - (0..i).map(|k| l[i * n + k] * x[k]).sum::<f64>()) / l[i * n + i];
    }
    for i in (0..n).rev() {
        x[i] = (x[i] - (i + 1..n).map(|k| l[k * n + i] * x[k]).sum::<f64>()) / l[i * n + i];
    }
    true
}

/// Solve the square system `A x = b` in place by Gaussian elimination with partial pivoting
/// (`a` row-major with stride `n`, `b` overwritten by `x`). Returns `false` if `A` is singular.
#[must_use]
pub fn solve_dense(n: usize, a: &mut [f64], b: &mut [f64]) -> bool {
    for col in 0..n {
        let Some(piv) = (col..n).max_by(|&i, &j| a[i * n + col].abs().total_cmp(&a[j * n + col].abs()))
        else {
            return false;
        };
        if a[piv * n + col].abs() < 1e-14 {
            return false;
        }
        if piv != col {
            for j in 0..n {
                a.swap(col * n + j, piv * n + j);
            }
            b.swap(col, piv);
        }
        for i in col + 1..n {
            let f = a[i * n + col] / a[col * n + col];
            if f != 0.0 {
                for j in col..n {
                    a[i * n + j] -= f * a[col * n + j];
                }
                b[i] -= f * b[col];
            }
        }
    }
    for i in (0..n).rev() {
        let s: f64 = (i + 1..n).map(|j| a[i * n + j] * b[j]).sum();
        b[i] = (b[i] - s) / a[i * n + i];
    }
    true
}

#[cfg(test)]
mod tests {
    #[test]
    fn max_abs_propagates_nan() {
        assert!(max_abs([1.0, f64::NAN, 3.0]).is_nan());
        assert!(max_abs([f64::NAN, 1.0]).is_nan());
        assert_eq!(max_abs([-4.0, 3.0]), 4.0);
    }

    #[test]
    fn dense_solve() {
        let mut a = [0.0, 2.0, 1.0, 1.0, 1.0, 0.0, 3.0, 0.0, 1.0];
        let mut b = [5.0, 3.0, 6.0];
        assert!(solve_dense(3, &mut a, &mut b));
        // verify against the original system
        let a0 = [0.0, 2.0, 1.0, 1.0, 1.0, 0.0, 3.0, 0.0, 1.0];
        for i in 0..3 {
            let r: f64 = (0..3).map(|j| a0[i * 3 + j] * b[j]).sum();
            assert!((r - [5.0, 3.0, 6.0][i]).abs() < 1e-12);
        }
    }

    use super::*;

    #[test]
    fn inverse_roundtrip() {
        let a = DMat { rows: 3, cols: 3, data: vec![4.0, 1.0, 2.0, 0.5, 3.0, 1.0, 2.0, -1.0, 5.0] };
        let i = &a * &a.inverse().unwrap();
        assert!((&i - &DMat::identity(3)).max_abs() < 1e-12);
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
