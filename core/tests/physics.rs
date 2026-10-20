//! End-to-end checks of the simulator against physical laws and the controllers against their specs.

use core::f64::consts::PI;

use meowfer_core::control::{arm, cartpole, wrap};
use meowfer_core::layout::{Flag, off};
use meowfer_core::model::SystemId;
use meowfer_core::sim::Sim;

fn frictionless(id: SystemId) -> Sim {
    let mut s = Sim::new(id);
    for &i in s.model.damping {
        s.set_param(i, 0.0);
    }
    s
}

fn total_energy(s: &Sim) -> f64 {
    s.energy().iter().sum()
}

#[test]
fn passive_motion_conserves_energy() {
    for id in [SystemId::CartPole, SystemId::Arm2] {
        let mut s = frictionless(id);
        s.set_state(1, 2.0, 3.0);
        let e0 = total_energy(&s);
        s.step_n(10_000); // 10 s
        let drift = (total_energy(&s) - e0).abs() / e0.abs().max(1.0);
        assert!(drift < 1e-6, "{id:?}: relative energy drift {drift:e}");
    }
}

#[test]
fn friction_only_dissipates() {
    let mut s = Sim::new(SystemId::Arm2);
    s.set_state(0, 0.0, 4.0);
    let mut e = total_energy(&s);
    for _ in 0..2000 {
        s.step();
        let e1 = total_energy(&s);
        assert!(e1 <= e + 1e-9);
        e = e1;
    }
}

#[test]
fn published_snapshot_satisfies_lemma_4_2_and_the_equation_of_motion() {
    for id in [SystemId::CartPole, SystemId::Arm2] {
        let mut s = Sim::new(id);
        s.set_state(0, 0.3, 1.7);
        s.set_state(1, -0.8, -2.1);
        s.step_n(10);
        let o = s.publish().to_vec();
        assert!(o[off::SKEW] < 1e-12, "{id:?} skew {}", o[off::SKEW]);
        assert!(o[off::DET_M] > 0.0 && o[off::EIG_M] > 0.0, "M must be positive definite");
        // M q̈ + C q̇ + N = τ
        for i in 0..2 {
            let lhs = o[off::MQDD + i] + o[off::CQD + i] + o[off::N + i];
            assert!((lhs - o[off::TAU + i]).abs() < 1e-9, "{id:?} row {i}: {lhs} vs {}", o[off::TAU + i]);
        }
    }
}

#[test]
fn lqr_balances_the_cart_pole() {
    let mut s = Sim::new(SystemId::CartPole);
    s.set_mode(cartpole::MODE_LQR);
    s.set_state(1, 0.25, 0.0);
    s.step_n(8_000);
    assert!(wrap(s.q[1]).abs() < 1e-3, "θ = {}", s.q[1]);
    assert!(s.q[0].abs() < 1e-2, "x = {}", s.q[0]);
}

#[test]
fn lqr_tracks_a_new_cart_set_point() {
    let mut s = Sim::new(SystemId::CartPole);
    s.set_mode(cartpole::MODE_LQR);
    s.set_state(1, 0.0, 0.0);
    s.set_flag(Flag::XRef, 1.0);
    s.step_n(10_000);
    assert!((s.q[0] - 1.0).abs() < 1e-2, "x = {}", s.q[0]);
}

#[test]
fn energy_pumping_swings_the_pole_up_and_catches_it() {
    let mut s = Sim::new(SystemId::CartPole);
    s.set_mode(cartpole::MODE_SWING_UP);
    s.set_state(0, 0.0, 0.0);
    s.set_state(1, PI, 0.0);
    let mut max_x: f64 = 0.0;
    for _ in 0..25_000 {
        s.step();
        max_x = max_x.max(s.q[0].abs());
    }
    assert!(wrap(s.q[1]).abs() < 1e-2, "θ = {}", wrap(s.q[1]));
    assert!(max_x < s.rail, "cart hit the end stop (|x|max = {max_x})");
}

#[test]
fn computed_torque_reaches_the_target() {
    let mut s = Sim::new(SystemId::Arm2);
    s.set_mode(arm::MODE_COMPUTED_TORQUE);
    s.set_flag(Flag::TargetX, -0.5);
    s.set_flag(Flag::TargetY, 1.2);
    s.step_n(5_000);
    let o = s.publish().to_vec();
    let (x, y) = (o[off::TOOL_P], o[off::TOOL_P + 1]);
    assert!((x + 0.5).hypot(y - 1.2) < 1e-3, "tool at ({x}, {y})");
}

#[test]
fn pd_with_gravity_compensation_has_no_steady_state_error() {
    let mut s = Sim::new(SystemId::Arm2);
    s.set_mode(arm::MODE_PD_GRAVITY);
    s.set_flag(Flag::TargetX, 1.4);
    s.set_flag(Flag::TargetY, 0.2);
    s.step_n(10_000);
    let o = s.publish().to_vec();
    let (x, y) = (o[off::TOOL_P], o[off::TOOL_P + 1]);
    assert!((x - 1.4).hypot(y - 0.2) < 1e-3, "tool at ({x}, {y})");
}

#[test]
fn inverse_kinematics_round_trips() {
    for elbow in [1.0, -1.0] {
        let (t, ok) = arm::inverse_kinematics(1.0, 0.8, 0.9, 0.7, elbow);
        assert!(ok);
        let x = t[0].cos() + 0.8 * (t[0] + t[1]).cos();
        let y = t[0].sin() + 0.8 * (t[0] + t[1]).sin();
        assert!((x - 0.9).abs() < 1e-12 && (y - 0.7).abs() < 1e-12);
    }
}

#[test]
fn rail_end_stops_keep_the_cart_on_the_rail() {
    let mut s = Sim::new(SystemId::CartPole);
    s.set_state(0, 0.0, 6.0);
    s.step_n(3_000);
    assert!(s.q[0].abs() < s.rail + 0.05, "x = {}", s.q[0]);
}
