//! The in-browser CAS must reproduce the independent SymPy derivation (`sym/`, generated into `oracle/`).

#[rustfmt::skip]
#[path = "oracle/arm2.rs"]
mod arm2_oracle;
#[rustfmt::skip]
#[path = "oracle/cartpole.rs"]
mod cartpole_oracle;

use motion_core::dynamics::Terms;
use motion_core::mech::{Desc, Vars, presets};
use motion_core::model::Model;

/// Deterministic pseudo-random numbers in [−1, 1).
struct Lcg(u64);
impl Lcg {
    fn next(&mut self) -> f64 {
        self.0 = self.0.wrapping_mul(6_364_136_223_846_793_005).wrapping_add(1_442_695_040_888_963_407);
        ((self.0 >> 11) as f64 / (1u64 << 53) as f64) * 2.0 - 1.0
    }
}

type OracleMcn = fn(&[f64], &[f64], &[f64], &mut [f64], &mut [f64], &mut [f64]);
type OracleEnergy = fn(&[f64], &[f64], &[f64]) -> [f64; 2];

fn compare(desc: &Desc, oracle_p: impl Fn(&[f64]) -> Vec<f64>, mcn: OracleMcn, energy: OracleEnergy) {
    let n = desc.n();
    let mut model = Model::from_desc(desc);
    let mut rng = Lcg(7);
    for _ in 0..200 {
        // random (positive) coefficients, random state
        let p: Vec<f64> = model.defaults.iter().map(|_| 0.2 + 2.0 * rng.next().abs()).collect();
        model.set_params(&p);
        let q: Vec<f64> = (0..n).map(|_| 3.0 * rng.next()).collect();
        let qd: Vec<f64> = (0..n).map(|_| 3.0 * rng.next()).collect();
        let t = Terms::eval(&mut model.dynamics, n, &q, &qd);
        let po = oracle_p(&p);
        let (mut m, mut c, mut nv) = (vec![0.0; n * n], vec![0.0; n * n], vec![0.0; n]);
        mcn(&po, &q, &qd, &mut m, &mut c, &mut nv);
        for k in 0..n * n {
            assert!((t.m[k] - m[k]).abs() < 1e-10, "M[{k}]: {} vs {}", t.m[k], m[k]);
            assert!((t.c[k] - c[k]).abs() < 1e-10, "C[{k}]: {} vs {}", t.c[k], c[k]);
        }
        for k in 0..n {
            assert!((t.n[k] - nv[k]).abs() < 1e-10, "N[{k}]: {} vs {}", t.n[k], nv[k]);
        }
        let mut e = [0.0; 2];
        model.energy.eval(&q, &qd, &mut e);
        let eo = energy(&po, &q, &qd);
        assert!((e[0] - eo[0]).abs() < 1e-10 && (e[1] - eo[1]).abs() < 1e-10, "{e:?} vs {eo:?}");
    }
}

#[test]
fn cartpole_matches_sympy() {
    let v = Vars { n: 2, nb: 2 };
    // oracle order: mc, mp, r, Ip, g, βx, βθ
    compare(
        &presets::cartpole(),
        |p| vec![p[v.m(0)], p[v.m(1)], p[v.r(1)], p[v.inertia(1)], p[v.g()], p[v.beta(0)], p[v.beta(1)]],
        cartpole_oracle::mcn,
        cartpole_oracle::energy,
    );
}

#[test]
fn arm2_matches_sympy() {
    let v = Vars { n: 2, nb: 2 };
    // oracle order: m1, m2, l1, l2, r1, r2, I1, I2, g, β1, β2
    compare(
        &presets::arm2(),
        |p| {
            vec![
                p[v.m(0)],
                p[v.m(1)],
                p[v.l(0)],
                p[v.l(1)],
                p[v.r(0)],
                p[v.r(1)],
                p[v.inertia(0)],
                p[v.inertia(1)],
                p[v.g()],
                p[v.beta(0)],
                p[v.beta(1)],
            ]
        },
        arm2_oracle::mcn,
        arm2_oracle::energy,
    );
}
