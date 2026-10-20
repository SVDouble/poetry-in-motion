//! Derivation time and simulation throughput: `cargo run --release --example bench`.

use std::time::Instant;

use motion_core::control;
use motion_core::mech::{Desc, JointKind, presets};
use motion_core::model::Model;
use motion_core::sim::Sim;

fn chain(n: usize) -> Desc {
    let bodies = (0..n).map(|i| presets::link(i.checked_sub(1), JointKind::Revolute, 1.0, 0.6)).collect();
    Desc { bodies, g: 9.81, tool: Some(n - 1), closures: Vec::new() }
}

fn main() {
    let cases = [
        ("cart-pole, swing-up", presets::cartpole(), control::SWING_UP),
        ("RR arm, computed torque", presets::arm2(), control::COMPUTED_TORQUE),
        ("4-link chain, PD + gravity", chain(4), control::PD_GRAVITY),
        ("6-link chain, PD + gravity", chain(6), control::PD_GRAVITY),
    ];
    for (name, desc, mode) in cases {
        let t0 = Instant::now();
        let model = Model::from_desc(&desc);
        let derive_ms = t0.elapsed().as_secs_f64() * 1e3;
        let terms = model.dynamics.terms();
        let mut s = Sim::new(model);
        s.set_mode(mode);
        let n = 200_000;
        let t = Instant::now();
        s.step_n(n);
        let dt = t.elapsed().as_secs_f64();
        println!(
            "{name:28} derive {derive_ms:7.2} ms | M,C,N tape {terms:5} terms | {:7.0} ns/step ({:.0}× real time)",
            dt / f64::from(n) * 1e9,
            f64::from(n) / dt / 1000.0
        );
    }
}
