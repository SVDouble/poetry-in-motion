//! Derivation time and simulation throughput: `cargo run --release --example bench`.

use std::time::Instant;

use motion_core::control::Kind;
use motion_core::mech::{Desc, JointKind, presets};
use motion_core::model::Model;
use motion_core::world::World;

fn chain(n: usize) -> Desc {
    let bodies = (0..n).map(|i| presets::link(i.checked_sub(1), JointKind::Revolute, 1.0, 0.6)).collect();
    Desc { bodies, g: 9.81, tool: Some(n - 1), closures: Vec::new() }
}

fn main() {
    let cases = [
        ("cart-pole, swing-up", presets::cartpole(), Kind::SwingUp),
        ("RR arm, computed torque", presets::arm2(), Kind::ComputedTorque),
        ("4-link chain, PD + gravity", chain(4), Kind::PdGravity),
        ("6-link chain, PD + gravity", chain(6), Kind::PdGravity),
        ("RR arm, impedance", presets::arm2(), Kind::Impedance),
        ("cart-pole, MPC", presets::cartpole(), Kind::Mpc),
    ];
    for (name, desc, mode) in cases {
        let t0 = Instant::now();
        let model = Model::from_desc(&desc);
        let derive_ms = t0.elapsed().as_secs_f64() * 1e3;
        let terms = model.dynamics.terms();
        let mut w = World::new(model);
        w.set_controller(0, Some(mode));
        let n = 200_000;
        let t = Instant::now();
        w.step(n);
        let dt = t.elapsed().as_secs_f64();
        println!(
            "{name:28} derive {derive_ms:7.2} ms | M,C,N tape {terms:5} terms | {:7.0} ns/step ({:.0}× real time)",
            dt / f64::from(n) * 1e9,
            f64::from(n) / dt / 1000.0
        );
    }
}
