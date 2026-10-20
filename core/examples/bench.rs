//! Throughput of the full step (controller + 4 dynamics evaluations of RK4): `cargo run --release --example bench`.

use std::time::Instant;

use motion_core::control::{arm, cartpole};
use motion_core::model::SystemId;
use motion_core::sim::Sim;

fn main() {
    for (id, mode) in
        [(SystemId::CartPole, cartpole::MODE_SWING_UP), (SystemId::Arm2, arm::MODE_COMPUTED_TORQUE)]
    {
        let mut s = Sim::new(id);
        s.set_mode(mode);
        let n = 2_000_000;
        let t = Instant::now();
        s.step_n(n);
        let dt = t.elapsed().as_secs_f64();
        println!(
            "{id:?}: {:.0} ns/step, {:.1}× real time at 1 kHz",
            dt / f64::from(n) * 1e9,
            f64::from(n) / dt / 1000.0
        );
    }
}
