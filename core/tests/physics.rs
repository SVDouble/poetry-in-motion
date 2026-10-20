//! End-to-end checks of generic mechanisms against physical laws and of the generic controllers.

use core::f64::consts::{FRAC_PI_2, PI};

use motion_core::control::{self, wrap};
use motion_core::layout::{Flag, off};
use motion_core::mech::{Desc, JointKind, presets};
use motion_core::model::Model;
use motion_core::sim::Sim;

fn sim(desc: &Desc) -> Sim {
    Sim::new(Model::from_desc(desc))
}

fn frictionless(mut d: Desc) -> Desc {
    for b in &mut d.bodies {
        b.values[4] = 0.0;
    }
    d
}

fn chain(n: usize) -> Desc {
    let bodies = (0..n)
        .map(|i| {
            let mut b = presets::link(i.checked_sub(1), JointKind::Revolute, 1.0, 0.6);
            b.q0 = 0.3 * i as f64;
            b
        })
        .collect();
    Desc { bodies, g: 9.81, tool: Some(n - 1), closures: Vec::new() }
}

fn total_energy(s: &mut Sim) -> f64 {
    s.energy().iter().sum()
}

#[test]
fn passive_motion_conserves_energy() {
    let mut cart_double = presets::cartpole();
    let mut top = presets::link(Some(1), JointKind::Revolute, 0.2, 0.7);
    top.q0 = 1.0;
    cart_double.bodies.push(top);
    for d in [presets::cartpole(), presets::arm2(), presets::double_pendulum(), cart_double, chain(4)] {
        let mut s = sim(&frictionless(d));
        s.set_state(1, 2.0, 3.0);
        let e0 = total_energy(&mut s);
        s.step_n(5_000);
        let drift = (total_energy(&mut s) - e0).abs() / e0.abs().max(1.0);
        // RK4 at 1 ms: ~1e-7 for regular motion, a little more for chaotic multi-link chains
        assert!(drift < 1e-5, "n = {}: relative energy drift {drift:e}", s.n());
    }
}

#[test]
fn snapshot_satisfies_lemma_4_2_and_the_equation_of_motion() {
    for d in [presets::cartpole(), presets::arm2(), presets::acrobot(), chain(5)] {
        let mut s = sim(&d);
        s.set_mode(control::PD_GRAVITY);
        for i in 0..s.n() {
            s.set_state(i, 0.4 * i as f64 - 0.3, 1.1 - 0.5 * i as f64);
        }
        s.step_n(10);
        let n = s.n();
        let o = s.publish().to_vec();
        assert!(o[off::SKEW] < 1e-10, "skew {}", o[off::SKEW]);
        assert!(o[off::DET_M] > 0.0 && o[off::EIG_M] > 0.0, "M must be positive definite");
        for i in 0..n {
            let lhs = o[off::MQDD + i] + o[off::CQD + i] + o[off::N + i];
            assert!((lhs - o[off::TAU + i]).abs() < 1e-8, "row {i}: {lhs} vs {}", o[off::TAU + i]);
        }
    }
}

#[test]
fn lqr_balances_the_cart_pole_and_tracks_a_set_point() {
    let mut s = sim(&presets::cartpole());
    s.set_mode(control::LQR);
    s.set_state(1, 0.25, 0.0);
    s.step_n(8_000);
    assert!(wrap(s.q[1]).abs() < 1e-3, "θ = {}", s.q[1]);
    s.set_target(0, 1.0);
    s.step_n(10_000);
    assert!((s.q[0] - 1.0).abs() < 1e-2, "x = {}", s.q[0]);
}

#[test]
fn energy_pumping_swings_the_cart_pole_up_and_catches_it() {
    let mut s = sim(&presets::cartpole());
    s.set_mode(control::SWING_UP);
    s.set_state(0, 0.0, 0.0);
    s.set_state(1, PI, 0.0);
    let mut max_x: f64 = 0.0;
    for _ in 0..25_000 {
        s.step();
        max_x = max_x.max(s.q[0].abs());
    }
    assert!(wrap(s.q[1]).abs() < 1e-2, "θ = {}", wrap(s.q[1]));
    assert!(max_x < 2.4, "cart hit the end stop (|x|max = {max_x})");
}

#[test]
fn lqr_balances_the_acrobot() {
    let mut s = sim(&presets::acrobot());
    s.set_mode(control::LQR);
    // the acrobot's basin of attraction is small: a 0.02 rad shoulder perturbation
    s.set_state(0, 0.02, 0.0);
    s.step_n(8_000);
    assert!(wrap(s.q[0]).abs() < 1e-3 && wrap(s.q[1]).abs() < 1e-3, "q = {:?}", &s.q[..2]);
}

#[test]
fn computed_torque_with_inverse_kinematics_reaches_the_tool_target() {
    for d in [presets::arm2(), chain(3)] {
        let mut s = sim(&d);
        s.set_mode(control::COMPUTED_TORQUE);
        s.set_flag(Flag::TargetX, -0.5);
        s.set_flag(Flag::TargetY, 1.0);
        s.set_flag(Flag::Ik, 1.0);
        s.step_n(6_000);
        let o = s.publish().to_vec();
        let (x, y) = (o[off::TOOL_P], o[off::TOOL_P + 1]);
        assert!((x + 0.5).hypot(y - 1.0) < 2e-3, "n = {}: tool at ({x}, {y})", s.n());
    }
}

#[test]
fn pd_with_gravity_compensation_has_no_steady_state_error() {
    let mut s = sim(&presets::arm2());
    s.set_mode(control::PD_GRAVITY);
    s.set_target(0, 0.3);
    s.set_target(1, 0.8);
    s.step_n(10_000);
    assert!((s.q[0] - 0.3).abs() < 1e-3 && (s.q[1] - 0.8).abs() < 1e-3, "q = {:?}", &s.q[..2]);
}

#[test]
fn plain_pd_sags_under_gravity() {
    let mut s = sim(&presets::arm2());
    s.set_mode(control::PD);
    s.set_target(0, 0.0);
    s.set_target(1, 0.0);
    s.step_n(10_000);
    assert!(s.q[0] < -0.05, "a horizontal arm under PD must sag: θ₁ = {}", s.q[0]);
}

#[test]
fn rail_end_stops_keep_the_cart_on_the_rail() {
    let mut s = sim(&presets::cartpole());
    s.set_state(0, 0.0, 6.0);
    s.step_n(3_000);
    assert!(s.q[0].abs() < 2.45, "x = {}", s.q[0]);
}

#[test]
fn mounting_geometry_is_respected() {
    // a pole mounted at +π/2 on the cart points up when θ = 0
    let mut s = sim(&presets::cartpole());
    s.set_state(1, 0.0, 0.0);
    let o = s.publish().to_vec();
    let com_y = o[off::COMS + 3];
    assert!((com_y - 0.5).abs() < 1e-12, "pole COM height {com_y}");
    assert!((o[off::FRAMES + 3 * 2 + 2] - FRAC_PI_2).abs() < 1e-12);
}

#[test]
fn a_six_link_chain_derives_and_runs() {
    let mut s = sim(&chain(6));
    s.step_n(100);
    assert!(s.q.iter().all(|q| q.is_finite()));
    assert!(s.model.derivation.len() > 1000);
}

#[test]
fn lqr_balances_a_cart_double_pendulum() {
    let mut d = presets::cartpole();
    d.bodies[1].values = [0.3, 0.6, 0.3, 0.3 * 0.36 / 12.0, 0.002];
    let mut top = presets::link(Some(1), JointKind::Revolute, 0.2, 0.6);
    top.actuated = false;
    top.values[4] = 0.002;
    d.bodies.push(top);
    let mut s = sim(&d);
    s.set_mode(control::LQR);
    s.set_state(1, 0.05, 0.0);
    s.set_state(2, -0.05, 0.0);
    s.step_n(10_000);
    assert!((0..3).all(|i| wrap(s.q[i]).abs() < 1e-3), "q = {:?}", &s.q[..3]);
}

#[test]
fn four_bar_conserves_energy_and_stays_closed() {
    let mut s = sim(&frictionless(presets::four_bar()));
    let o = s.publish().to_vec();
    assert!(o[off::CONSTRAINT] < 1e-9, "initial pose not projected: {}", o[off::CONSTRAINT]);
    s.kick(0, 3.0);
    let e0 = total_energy(&mut s);
    let mut worst: f64 = 0.0;
    for _ in 0..5_000 {
        s.step();
        worst = worst.max(s.publish()[off::CONSTRAINT]);
    }
    let drift = (total_energy(&mut s) - e0).abs() / e0.abs().max(1.0);
    assert!(drift < 1e-4, "energy drift {drift:e}");
    assert!(worst < 1e-6, "loop opened by {worst:e} m");
}

#[test]
fn four_bar_eom_balances_with_constraint_forces() {
    let mut s = sim(&presets::four_bar());
    s.set_mode(control::PD_GRAVITY);
    s.step_n(50);
    let o = s.publish().to_vec();
    for i in 0..3 {
        let lhs = o[off::MQDD + i] + o[off::CQD + i] + o[off::N + i] + o[off::AT_LAMBDA + i];
        assert!((lhs - o[off::TAU + i]).abs() < 1e-6, "row {i}: {lhs} vs {}", o[off::TAU + i]);
    }
}

#[test]
fn a_pendulum_on_a_fixed_bracket_matches_a_pendulum_mounted_there() {
    // bracket welded to the ground at 30°, pendulum hinged at its middle
    let mut bracket = presets::link(None, JointKind::Fixed, 2.0, 1.0);
    bracket.mount_angle = 0.5;
    let mut pend = presets::link(Some(0), JointKind::Revolute, 1.0, 0.8);
    pend.mount_s = 0.5;
    pend.actuated = false;
    pend.mount_angle = -0.5 - FRAC_PI_2;
    pend.q0 = 0.7;
    let with_bracket =
        Desc { bodies: vec![bracket, pend.clone()], g: 9.81, tool: None, closures: Vec::new() };
    let mut direct = pend;
    direct.parent = None;
    direct.mount_xy = [0.5 * 0.5f64.cos(), 0.5 * 0.5f64.sin()];
    direct.mount_angle = -FRAC_PI_2;
    let alone = Desc { bodies: vec![direct], g: 9.81, tool: None, closures: Vec::new() };
    let (mut a, mut b) = (sim(&with_bracket), sim(&alone));
    assert_eq!(a.n(), 1);
    a.step_n(3_000);
    b.step_n(3_000);
    assert!((a.q[0] - b.q[0]).abs() < 1e-9, "{} vs {}", a.q[0], b.q[0]);
}

#[test]
fn mount_ports_stay_symbolic() {
    let mut d = presets::arm2();
    d.bodies[1].mount_s = 0.5;
    let m = Model::from_desc(&d);
    assert!(m.derivation.contains(r"\\frac{1}{2} l_{1}"), "{}", &m.derivation[..400]);
}

fn crank_target(s: &mut Sim, v: f64) {
    s.set_target(0, v);
    s.step_n(1);
}

#[test]
fn computed_torque_drives_a_four_bar_through_its_crank() {
    let mut s = sim(&presets::four_bar());
    s.set_mode(control::COMPUTED_TORQUE);
    crank_target(&mut s, 1.2);
    assert!(s.ctrl.ct_ok, "one actuator must suffice for a 1-DOF loop");
    s.step_n(5_000);
    assert!((s.q[0] - 1.2).abs() < 1e-3, "crank at {}", s.q[0]);
    assert!(s.publish()[off::CONSTRAINT] < 1e-6);
}

#[test]
fn pd_with_gravity_compensation_holds_a_four_bar_exactly() {
    let mut s = sim(&presets::four_bar());
    s.set_mode(control::PD_GRAVITY);
    crank_target(&mut s, 1.9);
    s.step_n(10_000);
    assert!((s.q[0] - 1.9).abs() < 1e-3, "crank at {}", s.q[0]);
}

#[test]
fn lqr_on_the_constraint_manifold_stabilises_a_four_bar() {
    let mut s = sim(&presets::four_bar());
    s.set_mode(control::LQR);
    crank_target(&mut s, 1.4);
    s.kick(0, 1.0);
    s.step_n(8_000);
    assert!((s.q[0] - 1.4).abs() < 1e-3, "crank at {}", s.q[0]);
    assert_eq!(s.publish()[off::LQR_DIM], 1.0, "a four-bar has one tangent direction");
}

#[test]
fn inverse_kinematics_moves_the_coupler_of_a_four_bar() {
    let mut s = sim(&presets::four_bar());
    s.set_mode(control::COMPUTED_TORQUE);
    s.step_n(1);
    let o = s.publish().to_vec();
    // a nearby point on the coupler curve: move the crank a little and read the tool position
    let (x0, y0) = (o[off::TOOL_P], o[off::TOOL_P + 1]);
    s.set_flag(Flag::TargetX, x0 + 0.05);
    s.set_flag(Flag::TargetY, y0 - 0.02);
    s.set_flag(Flag::Ik, 1.0);
    s.step_n(5_000);
    let o = s.publish().to_vec();
    let moved = (o[off::TOOL_P] - x0).hypot(o[off::TOOL_P + 1] - y0);
    assert!(moved > 0.02, "tool did not move ({moved})");
    assert!(o[off::CONSTRAINT] < 1e-6);
}

#[test]
fn slider_crank_slot_conserves_energy_and_stays_in_the_slot() {
    let mut s = sim(&frictionless(presets::slider_crank()));
    assert_eq!(s.model.ncon, 1, "a slot is one constraint");
    s.kick(0, 4.0);
    let e0 = total_energy(&mut s);
    let mut worst: f64 = 0.0;
    for _ in 0..5_000 {
        s.step();
        worst = worst.max(s.publish()[off::CONSTRAINT]);
    }
    let drift = (total_energy(&mut s) - e0).abs() / e0.abs().max(1.0);
    assert!(drift < 1e-4, "energy drift {drift:e}");
    assert!(worst < 1e-6, "left the slot by {worst:e} m");
    // the rod tip (tool) stays on y = 0
    assert!(s.publish()[off::TOOL_P + 1].abs() < 1e-6);
}
