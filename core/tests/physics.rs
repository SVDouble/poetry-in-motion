//! End-to-end checks of generic mechanisms against physical laws and of the generic controllers.

use core::f64::consts::{FRAC_PI_2, PI};

use motion_core::control::Kind;
use motion_core::layout::{Flag, off, slot};
use motion_core::mech::{Desc, Element, JointKind, presets};
use motion_core::model::Model;
use motion_core::plan::wrap;
use motion_core::world::World;

fn sim(desc: &Desc) -> World {
    World::new(Model::from_desc(desc))
}

fn with(desc: &Desc, kind: Kind) -> World {
    let mut w = sim(desc);
    w.set_controller(0, Some(kind));
    assert_eq!(w.primary().kind, kind, "{kind:?} not applicable");
    w
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
    Desc { bodies, g: 9.81, tool: Some(n - 1), closures: Vec::new(), elements: Vec::new() }
}

fn total_energy(s: &mut World) -> f64 {
    s.primary_mut().energy().iter().sum()
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
        s.step(5_000);
        let drift = (total_energy(&mut s) - e0).abs() / e0.abs().max(1.0);
        // RK4 at 1 ms: ~1e-7 for regular motion, a little more for chaotic multi-link chains
        assert!(drift < 1e-5, "n = {}: relative energy drift {drift:e}", s.primary().n());
    }
}

#[test]
fn snapshot_satisfies_lemma_4_2_and_the_equation_of_motion() {
    for d in [presets::cartpole(), presets::arm2(), presets::acrobot(), chain(5)] {
        let mut s = with(&d, Kind::PdGravity);
        for i in 0..s.primary().n() {
            s.set_state(i, 0.4 * i as f64 - 0.3, 1.1 - 0.5 * i as f64);
        }
        s.step(10);
        let n = s.primary().n();
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
    let mut s = with(&presets::cartpole(), Kind::Lqr);
    s.set_state(1, 0.25, 0.0);
    s.step(8_000);
    assert!(wrap(s.primary().q[1]).abs() < 1e-3, "θ = {}", s.primary().q[1]);
    s.set_target(0, 1.0);
    s.step(10_000);
    assert!((s.primary().q[0] - 1.0).abs() < 1e-2, "x = {}", s.primary().q[0]);
}

#[test]
fn energy_pumping_swings_the_cart_pole_up_and_catches_it() {
    let mut s = with(&presets::cartpole(), Kind::SwingUp);
    s.set_state(0, 0.0, 0.0);
    s.set_state(1, PI, 0.0);
    let mut max_x: f64 = 0.0;
    for _ in 0..25_000 {
        s.step(1);
        max_x = max_x.max(s.primary().q[0].abs());
    }
    assert!(wrap(s.primary().q[1]).abs() < 1e-2, "θ = {}", wrap(s.primary().q[1]));
    assert!(max_x < 2.4, "cart hit the end stop (|x|max = {max_x})");
}

#[test]
fn lqr_balances_the_acrobot() {
    let mut s = with(&presets::acrobot(), Kind::Lqr);
    // the acrobot's basin of attraction is small: a 0.02 rad shoulder perturbation
    s.set_state(0, 0.02, 0.0);
    s.step(8_000);
    assert!(
        wrap(s.primary().q[0]).abs() < 1e-3 && wrap(s.primary().q[1]).abs() < 1e-3,
        "q = {:?}",
        &s.primary().q[..2]
    );
}

#[test]
fn computed_torque_with_inverse_kinematics_reaches_the_tool_target() {
    for d in [presets::arm2(), chain(3)] {
        let mut s = with(&d, Kind::ComputedTorque);
        s.set_flag(Flag::TargetX, -0.5);
        s.set_flag(Flag::TargetY, 1.0);
        s.set_flag(Flag::UseTool, 1.0);
        s.step(6_000);
        let o = s.publish().to_vec();
        let (x, y) = (o[off::TOOL_P], o[off::TOOL_P + 1]);
        assert!((x + 0.5).hypot(y - 1.0) < 2e-3, "n = {}: tool at ({x}, {y})", s.primary().n());
    }
}

#[test]
fn pd_with_gravity_compensation_has_no_steady_state_error() {
    let mut s = with(&presets::arm2(), Kind::PdGravity);
    s.set_target(0, 0.3);
    s.set_target(1, 0.8);
    s.step(10_000);
    assert!(
        (s.primary().q[0] - 0.3).abs() < 1e-3 && (s.primary().q[1] - 0.8).abs() < 1e-3,
        "q = {:?}",
        &s.primary().q[..2]
    );
}

#[test]
fn plain_pd_sags_under_gravity() {
    let mut s = with(&presets::arm2(), Kind::Pd);
    s.set_target(0, 0.0);
    s.set_target(1, 0.0);
    s.step(10_000);
    assert!(s.primary().q[0] < -0.05, "a horizontal arm under PD must sag: θ₁ = {}", s.primary().q[0]);
}

#[test]
fn rail_end_stops_keep_the_cart_on_the_rail() {
    let mut s = sim(&presets::cartpole());
    s.set_state(0, 0.0, 6.0);
    s.step(3_000);
    assert!(s.primary().q[0].abs() < 2.45, "x = {}", s.primary().q[0]);
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
    s.step(100);
    assert!(s.primary().q.iter().all(|q| q.is_finite()));
    assert!(s.primary().model.derivation.len() > 1000);
}

#[test]
fn lqr_balances_a_cart_double_pendulum() {
    let mut d = presets::cartpole();
    d.bodies[1].values = [0.3, 0.6, 0.3, 0.3 * 0.36 / 12.0, 0.002];
    let mut top = presets::link(Some(1), JointKind::Revolute, 0.2, 0.6);
    top.actuated = false;
    top.values[4] = 0.002;
    d.bodies.push(top);
    let mut s = with(&d, Kind::Lqr);
    s.set_state(1, 0.05, 0.0);
    s.set_state(2, -0.05, 0.0);
    s.step(10_000);
    assert!((0..3).all(|i| wrap(s.primary().q[i]).abs() < 1e-3), "q = {:?}", &s.primary().q[..3]);
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
        s.step(1);
        worst = worst.max(s.publish()[off::CONSTRAINT]);
    }
    let drift = (total_energy(&mut s) - e0).abs() / e0.abs().max(1.0);
    assert!(drift < 1e-4, "energy drift {drift:e}");
    assert!(worst < 1e-6, "loop opened by {worst:e} m");
}

#[test]
fn four_bar_eom_balances_with_constraint_forces() {
    let mut s = with(&presets::four_bar(), Kind::PdGravity);
    s.step(50);
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
    bracket.values[4] = 0.7; // a welded body has no joint: its friction coefficient must not matter
    let mut pend = presets::link(Some(0), JointKind::Revolute, 1.0, 0.8);
    pend.mount_s = 0.5;
    pend.actuated = false;
    pend.mount_angle = -0.5 - FRAC_PI_2;
    pend.q0 = 0.7;
    let with_bracket = Desc {
        bodies: vec![bracket, pend.clone()],
        g: 9.81,
        tool: None,
        closures: Vec::new(),
        elements: Vec::new(),
    };
    let mut direct = pend;
    direct.parent = None;
    direct.mount_xy = [0.5 * 0.5f64.cos(), 0.5 * 0.5f64.sin()];
    direct.mount_angle = -FRAC_PI_2;
    let alone =
        Desc { bodies: vec![direct], g: 9.81, tool: None, closures: Vec::new(), elements: Vec::new() };
    let (mut a, mut b) = (sim(&with_bracket), sim(&alone));
    assert_eq!(a.primary().n(), 1);
    a.step(3_000);
    b.step(3_000);
    assert!(
        (a.primary().q[0] - b.primary().q[0]).abs() < 1e-9,
        "{} vs {}",
        a.primary().q[0],
        b.primary().q[0]
    );
}

#[test]
fn mount_ports_stay_symbolic() {
    let mut d = presets::arm2();
    d.bodies[1].mount_s = 0.5;
    let m = Model::from_desc(&d);
    assert!(m.derivation.contains(r"\\frac{1}{2} l_{1}"), "{}", &m.derivation[..400]);
}

fn crank_target(s: &mut World, v: f64) {
    s.set_target(0, v);
    s.step(1);
}

#[test]
fn computed_torque_drives_a_four_bar_through_its_crank() {
    let mut s = with(&presets::four_bar(), Kind::ComputedTorque);
    crank_target(&mut s, 1.2);
    assert!(s.primary().plan.ct_ok, "one actuator must suffice for a 1-DOF loop");
    s.step(5_000);
    assert!((s.primary().q[0] - 1.2).abs() < 1e-3, "crank at {}", s.primary().q[0]);
    assert!(s.publish()[off::CONSTRAINT] < 1e-6);
}

#[test]
fn pd_with_gravity_compensation_holds_a_four_bar_exactly() {
    let mut s = with(&presets::four_bar(), Kind::PdGravity);
    crank_target(&mut s, 1.9);
    s.step(10_000);
    assert!((s.primary().q[0] - 1.9).abs() < 1e-3, "crank at {}", s.primary().q[0]);
}

#[test]
fn lqr_on_the_constraint_manifold_stabilises_a_four_bar() {
    let mut s = with(&presets::four_bar(), Kind::Lqr);
    crank_target(&mut s, 1.4);
    s.kick(0, 1.0);
    s.step(8_000);
    assert!((s.primary().q[0] - 1.4).abs() < 1e-3, "crank at {}", s.primary().q[0]);
    assert_eq!(s.publish()[off::DOF], 1.0, "a four-bar has one tangent direction");
}

#[test]
fn inverse_kinematics_moves_the_coupler_of_a_four_bar() {
    let mut s = with(&presets::four_bar(), Kind::ComputedTorque);
    s.step(1);
    let o = s.publish().to_vec();
    // a nearby point on the coupler curve: move the crank a little and read the tool position
    let (x0, y0) = (o[off::TOOL_P], o[off::TOOL_P + 1]);
    s.set_flag(Flag::TargetX, x0 + 0.05);
    s.set_flag(Flag::TargetY, y0 - 0.02);
    s.set_flag(Flag::UseTool, 1.0);
    s.step(5_000);
    let o = s.publish().to_vec();
    let moved = (o[off::TOOL_P] - x0).hypot(o[off::TOOL_P + 1] - y0);
    assert!(moved > 0.02, "tool did not move ({moved})");
    assert!(o[off::CONSTRAINT] < 1e-6);
}

#[test]
fn slider_crank_slot_conserves_energy_and_stays_in_the_slot() {
    let mut s = sim(&frictionless(presets::slider_crank()));
    assert_eq!(s.primary().model.ncon, 1, "a slot is one constraint");
    s.kick(0, 4.0);
    let e0 = total_energy(&mut s);
    let mut worst: f64 = 0.0;
    for _ in 0..5_000 {
        s.step(1);
        worst = worst.max(s.publish()[off::CONSTRAINT]);
    }
    let drift = (total_energy(&mut s) - e0).abs() / e0.abs().max(1.0);
    assert!(drift < 1e-4, "energy drift {drift:e}");
    assert!(worst < 1e-6, "left the slot by {worst:e} m");
    // the rod tip (tool) stays on y = 0
    assert!(s.publish()[off::TOOL_P + 1].abs() < 1e-6);
}

// ── controllers added with the slot architecture ───────────────────────────────────────────────────────

/// Telemetry field `name` of the controller in slot `i`, from the published snapshot.
fn telemetry(w: &mut World, i: usize, name: &str) -> Vec<f64> {
    let kind = w.slot(i).expect("slot running").kind;
    let (o, len) = kind.spec().telemetry_field(name).expect("telemetry field");
    let at = off::SLOT + i * slot::LEN + slot::TELEMETRY + o;
    w.publish()[at..at + len].to_vec()
}

fn tool(w: &mut World) -> [f64; 2] {
    let o = w.publish();
    [o[off::TOOL_P], o[off::TOOL_P + 1]]
}

fn aim(w: &mut World, x: f64, y: f64) {
    w.set_flag(Flag::TargetX, x);
    w.set_flag(Flag::TargetY, y);
    w.set_flag(Flag::UseTool, 1.0);
}

#[test]
fn mpc_balances_the_cart_pole_within_its_input_bounds() {
    let mut s = with(&presets::cartpole(), Kind::Mpc);
    s.set_flag(Flag::Umax, 25.0);
    s.set_state(1, 0.2, 0.0);
    let mut peak: f64 = 0.0;
    for _ in 0..8_000 {
        s.step(1);
        peak = peak.max(s.publish()[off::U].abs());
    }
    assert!(wrap(s.primary().q[1]).abs() < 1e-2, "θ = {}", s.primary().q[1]);
    assert!(peak <= 25.0 + 1e-9, "u exceeded its bound: {peak}");
    assert!(telemetry(&mut s, 0, "iterations")[0] > 0.0);
}

#[test]
fn vmc_and_passivity_reach_their_targets() {
    for d in [presets::arm2(), chain(3)] {
        let mut s = with(&d, Kind::Vmc);
        aim(&mut s, -0.4, 1.0);
        s.step(8_000);
        let [x, y] = tool(&mut s);
        assert!((x + 0.4).hypot(y - 1.0) < 5e-3, "VMC: tool at ({x}, {y})");
    }
    let mut s = with(&presets::arm2(), Kind::Passivity);
    s.set_target(0, 0.3);
    s.set_target(1, 0.8);
    s.step(6_000);
    let q = &s.primary().q;
    assert!((q[0] - 0.3).abs() < 1e-3 && (q[1] - 0.8).abs() < 1e-3, "q = {:?}", &q[..2]);
}

#[test]
fn null_space_posture_does_not_disturb_the_tool() {
    // a redundant 3-link arm: with and without the posture goal the tool ends at the target, the joints differ
    let run = |posture: f64| {
        let mut s = with(&chain(3), Kind::Vmc);
        s.set_flag(Flag::Posture, posture);
        aim(&mut s, -0.6, 1.2);
        s.step(10_000);
        let leak = telemetry(&mut s, 0, "leak")[0];
        (tool(&mut s), s.primary().q, leak)
    };
    let (p0, q0, _) = run(0.0);
    let (p1, q1, leak) = run(6.0);
    for p in [p0, p1] {
        assert!((p[0] + 0.6).hypot(p[1] - 1.2) < 5e-3, "tool at {p:?}");
    }
    // zero up to the 1e-9 regularisation of Λ that keeps it finite at singularities
    assert!(leak < 1e-6, "projected torque accelerates the tool: {leak:e}");
    assert!((0..3).map(|i| (q0[i] - q1[i]).abs()).sum::<f64>() > 1e-2, "posture goal had no effect");
}

#[test]
fn impedance_and_admittance_yield_to_a_push_like_springs() {
    // statically K_d (x − x_d) = F_ext = k_p (pointer − x). Impedance realises this at the tool exactly;
    // admittance realises it for the virtual point x_v, and the stiff (not infinitely stiff) inner position
    // loop lets the tool lag x_v by F_ext / (Λ ω_in²): two springs in series.
    let (xd, yd, k_p) = (-0.5, 1.0, 200.0);
    for kind in [Kind::Impedance, Kind::Admittance] {
        let mut s = with(&presets::arm2(), kind);
        let k_d = s.ctrl_params(kind)[1];
        aim(&mut s, xd, yd);
        s.step(6_000);
        let [x, y] = tool(&mut s);
        assert!((x - xd).hypot(y - yd) < 3e-3, "{kind:?}: free tool at ({x}, {y})");
        s.set_flag(Flag::PushK, k_p);
        s.set_flag(Flag::PushX, xd + 0.1);
        s.set_flag(Flag::PushY, yd);
        s.step(8_000);
        let [x, y] = tool(&mut s);
        let f = s.publish()[off::FEXT];
        assert!((f - k_p * (xd + 0.1 - x)).abs() < 0.5, "{kind:?}: push force {f}");
        let spring = if kind == Kind::Impedance { x } else { telemetry(&mut s, 0, "xv")[0] };
        assert!(
            (k_d * (spring - xd) - f).abs() < 0.05 * f,
            "{kind:?}: K_d Δx = {} vs F = {f}",
            k_d * (spring - xd)
        );
        assert!((x - spring).abs() < 2e-2 && (y - yd).abs() < 1e-2, "{kind:?}: tool at ({x}, {y})");
        s.set_flag(Flag::PushOn, 0.0);
        s.step(6_000);
        let [x, _] = tool(&mut s);
        assert!((x - xd).abs() < 3e-3, "{kind:?}: did not return ({x})");
    }
}

#[test]
fn shadows_run_independently_and_report_in_their_slots() {
    let mut w = with(&presets::arm2(), Kind::Pd);
    w.set_controller(1, Some(Kind::PdGravity));
    w.set_controller(2, Some(Kind::SwingUp)); // not applicable to a fully actuated arm: stays off
    w.set_target(0, 0.3);
    w.set_target(1, 0.8);
    w.step(8_000);
    let o = w.publish().to_vec();
    let block = |i: usize| &o[off::SLOT + i * slot::LEN..off::SLOT + (i + 1) * slot::LEN];
    assert_eq!(block(0)[slot::KIND], f64::from(Kind::Pd as u32));
    assert_eq!(block(1)[slot::KIND], f64::from(Kind::PdGravity as u32));
    assert_eq!(block(2)[slot::KIND], -1.0);
    assert!(block(1)[slot::ERR_Q] < 1e-3, "PD + gravity error {}", block(1)[slot::ERR_Q]);
    assert!(block(0)[slot::ERR_Q] > 10.0 * block(1)[slot::ERR_Q], "plain PD should sag");
    assert_eq!(&block(0)[slot::Q..slot::Q + 2], &o[off::Q..off::Q + 2]);
    // a kick reaches every slot
    w.kick(0, 2.0);
    w.step(1);
    let o = w.publish().to_vec();
    assert!(o[off::SLOT + slot::LEN + slot::ERR_Q] > 1e-3);
}

#[test]
fn controllers_report_applicability() {
    let mut w = sim(&presets::cartpole());
    let json = w.controllers_json();
    assert!(json.contains("\"specs\"") && json.contains("needs an independent actuator"), "{json}");
    w.set_controller(0, Some(Kind::ComputedTorque));
    assert_eq!(w.primary().kind, Kind::Passive, "inapplicable controllers fall back to passive");
}

#[test]
fn pushing_a_redundant_arm_far_away_settles_without_limit_cycles() {
    // the push is capped at a hand-like force, so the default actuators can resist it without saturating
    let spec = [(1.0, 0.8, -FRAC_PI_2 + 0.5), (0.8, 0.7, 0.5), (0.5, 0.5, 0.4)];
    let bodies = spec
        .iter()
        .enumerate()
        .map(|(i, &(m, l, q0))| {
            let mut b = presets::link(i.checked_sub(1), JointKind::Revolute, m, l);
            b.q0 = q0;
            b
        })
        .collect();
    let d = Desc { bodies, g: 9.81, tool: Some(2), closures: Vec::new(), elements: Vec::new() };
    for kind in [Kind::Impedance, Kind::Vmc, Kind::ComputedTorque] {
        let mut s = with(&d, kind);
        aim(&mut s, 0.6, 1.2);
        s.step(4_000);
        s.set_flag(Flag::PushX, 3.0);
        s.set_flag(Flag::PushY, -1.0);
        // the posture goal settles the redundant joint through a shallow, slower null-space mode
        s.step(10_000);
        let (mut lo, mut hi) = ([f64::MAX; 3], [f64::MIN; 3]);
        for _ in 0..500 {
            s.step(1);
            let o = s.publish();
            assert!(o[off::FEXT].hypot(o[off::FEXT + 1]) <= motion_core::sim::PUSH_MAX + 1e-9);
            for i in 0..3 {
                (lo[i], hi[i]) = (lo[i].min(o[off::Q + i]), hi[i].max(o[off::Q + i]));
            }
        }
        assert!((0..3).all(|i| hi[i] - lo[i] < 1e-3), "{kind:?} oscillates: {lo:?} … {hi:?}");
    }
}

#[test]
fn the_energy_budget_closes_with_friction_inputs_pushes_and_loops() {
    // E − E₀ = W_in + W_ext + W_stop − W_friction + W_loops, integrated with the RK4 weights of the motion
    for d in [
        presets::cartpole(),
        presets::arm2(),
        presets::acrobot(),
        presets::four_bar(),
        presets::slider_crank(),
    ] {
        for kind in [Kind::Passive, Kind::PdGravity] {
            let mut s = with(&d, kind);
            s.kick(0, 1.0);
            s.set_flag(Flag::PushX, 0.5);
            s.set_flag(Flag::PushY, 0.5);
            s.step(3_000);
            s.set_flag(Flag::PushOn, 0.0);
            s.step(5_000);
            let o = s.publish().to_vec();
            assert!(o[off::WORK + 3] > 0.0, "friction must dissipate");
            assert!(
                o[off::BUDGET].abs() < 1e-6,
                "{kind:?} n = {}: residual {:e}",
                s.primary().n(),
                o[off::BUDGET]
            );
        }
    }
    // the cart hitting its end stop: the stop's work is part of the budget
    let mut s = with(&presets::cartpole(), Kind::Passive);
    s.primary_mut().set_state(0, 0.0, 6.0);
    s.step(3_000);
    let o = s.publish();
    assert!(o[off::WORK + 2] < -1.0, "the stop should have absorbed energy: {}", o[off::WORK + 2]);
    // the stop switches on mid-substep: a few ppm of the ~20 J (0.2 % without contact substeps)
    assert!(o[off::BUDGET].abs() < 5e-4, "residual {:e}", o[off::BUDGET]);
}

/// Tool position, orientation and elbow targets stacked with strict priorities on a redundant arm.
fn stacked(d: &Desc, kind: Kind, dphi: f64, delbow: [f64; 2]) -> (World, [f64; 2], f64, [f64; 2]) {
    let mut s = with(d, kind);
    s.step(100);
    let o = s.publish().to_vec();
    let nb = d.bodies.len();
    let tool = [o[off::TOOL_P], o[off::TOOL_P + 1]];
    let phi = o[off::FRAMES + 3 * (nb + 1) + 2];
    let body = o[off::GOALS + 5] as usize;
    let elbow =
        [o[off::FRAMES + 3 * (body + 1)] + delbow[0], o[off::FRAMES + 3 * (body + 1) + 1] + delbow[1]];
    aim(&mut s, tool[0], tool[1]);
    s.set_flag(Flag::Orient, 6.0);
    s.set_flag(Flag::OrientTarget, phi + dphi);
    s.set_flag(Flag::Elbow, 6.0);
    s.set_flag(Flag::ElbowX, elbow[0]);
    s.set_flag(Flag::ElbowY, elbow[1]);
    s.step(10_000);
    (s, tool, phi + dphi, elbow)
}

fn tele(w: &mut World, kind: Kind, name: &str, i: usize) -> f64 {
    let (o, _) = kind.spec().telemetry_field(name).expect("telemetry field");
    w.publish()[off::SLOT + slot::TELEMETRY + o + i]
}

#[test]
fn a_seven_joint_arm_meets_position_orientation_and_elbow_goals_in_priority() {
    let d = presets::arm7();
    for kind in [Kind::Vmc, Kind::Impedance] {
        let (mut s, tool, phi, elbow) = stacked(&d, kind, 0.3, [0.05, 0.08]);
        let o = s.publish().to_vec();
        let p = [o[off::TOOL_P], o[off::TOOL_P + 1]];
        assert!(
            (p[0] - tool[0]).hypot(p[1] - tool[1]) < 2e-3,
            "{kind:?}: tool off by {:?}",
            [p[0] - tool[0], p[1] - tool[1]]
        );
        let eo = tele(&mut s, kind, "orient_err", 0);
        let ee = [tele(&mut s, kind, "elbow_err", 0), tele(&mut s, kind, "elbow_err", 1)];
        assert!(eo.abs() < 2e-3, "{kind:?}: orientation error {eo} (target {phi})");
        assert!(ee[0].hypot(ee[1]) < 5e-3, "{kind:?}: elbow error {ee:?} (target {elbow:?})");
        let (leak, leak_sec) = (tele(&mut s, kind, "leak", 0), tele(&mut s, kind, "leak_sec", 0));
        assert!(leak < 1e-6 && leak_sec < 1e-6, "{kind:?}: leakage {leak:e} / {leak_sec:e}");
    }
}

#[test]
fn an_unreachable_lower_goal_cannot_disturb_the_ones_above() {
    // three joints: tool position and orientation use all of them, the elbow goal has no room left
    let spec = [(1.0, 0.8, 0.4), (0.8, 0.7, 0.5), (0.5, 0.5, 0.4)];
    let bodies = spec
        .iter()
        .enumerate()
        .map(|(i, &(m, l, q0))| {
            let mut b = presets::link(i.checked_sub(1), JointKind::Revolute, m, l);
            b.q0 = q0;
            b
        })
        .collect();
    let d = Desc { bodies, g: 9.81, tool: Some(2), closures: Vec::new(), elements: Vec::new() };
    for kind in [Kind::Vmc, Kind::Impedance] {
        // turning the tool by −0.2 rad keeps the wrist within reach of the first two links
        let (mut s, tool, _, _) = stacked(&d, kind, -0.2, [0.3, -0.3]);
        let o = s.publish().to_vec();
        let p = [o[off::TOOL_P], o[off::TOOL_P + 1]];
        assert!((p[0] - tool[0]).hypot(p[1] - tool[1]) < 2e-3, "{kind:?}: the tool gave way to the elbow");
        assert!(tele(&mut s, kind, "orient_err", 0).abs() < 2e-3, "{kind:?}: orientation gave way");
        let ee = [tele(&mut s, kind, "elbow_err", 0), tele(&mut s, kind, "elbow_err", 1)];
        assert!(ee[0].hypot(ee[1]) > 0.1, "{kind:?}: the elbow cannot have been reached: {ee:?}");
    }
}

#[test]
fn an_unreachable_orientation_saturates_calmly() {
    // +0.4 rad would put the wrist beyond the first two links' reach: the arm stretches to the boundary and
    // stays there, the tool keeps its place, nothing oscillates or blows up (damped least squares)
    let spec = [(1.0, 0.8, 0.4), (0.8, 0.7, 0.5), (0.5, 0.5, 0.4)];
    let bodies = spec
        .iter()
        .enumerate()
        .map(|(i, &(m, l, q0))| {
            let mut b = presets::link(i.checked_sub(1), JointKind::Revolute, m, l);
            b.q0 = q0;
            b
        })
        .collect();
    let d = Desc { bodies, g: 9.81, tool: Some(2), closures: Vec::new(), elements: Vec::new() };
    for kind in [Kind::Vmc, Kind::Impedance] {
        let (mut s, tool, _, _) = stacked(&d, kind, 0.4, [0.0, 0.0]);
        let mut worst: f64 = 0.0;
        for _ in 0..1000 {
            s.step(1);
            let o = s.publish();
            worst = worst.max((0..3).map(|i| o[off::QD + i].abs()).fold(0.0, f64::max));
        }
        let o = s.publish().to_vec();
        let p = [o[off::TOOL_P], o[off::TOOL_P + 1]];
        assert!(
            (p[0] - tool[0]).hypot(p[1] - tool[1]) < 1e-2,
            "{kind:?}: the tool gave way: {p:?} vs {tool:?}"
        );
        assert!(worst < 0.05, "{kind:?}: still moving at {worst} rad/s");
    }
}

#[test]
fn an_unreachable_tool_target_leaves_the_arm_at_rest_reaching_towards_it() {
    // beyond the reach the arm stretches towards the target and stays there: no elbow flipping (adaptive
    // damped least squares), no ringing (damping of what the task cannot see), no switching (smooth
    // projectors), and no snaking of a redundant arm (impedance and admittance aim at the nearest reachable
    // point, VMC simply pulls)
    for d in [presets::arm2(), presets::arm7()] {
        let n = d.bodies.len();
        let reach: f64 = d.bodies.iter().map(|b| b.values[1]).sum();
        for kind in [Kind::Vmc, Kind::Impedance, Kind::Admittance] {
            for factor in [1.05, 1.2, 2.0, 4.0] {
                let mut s = with(&d, kind);
                let target = [factor * reach * 0.6, factor * reach * 0.8];
                aim(&mut s, target[0], target[1]);
                s.step(10_000);
                let (mut lo, mut hi, mut speed) = ([f64::MAX; 7], [f64::MIN; 7], 0.0_f64);
                for _ in 0..1000 {
                    s.step(1);
                    let o = s.publish();
                    for i in 0..n {
                        (lo[i], hi[i]) = (lo[i].min(o[off::Q + i]), hi[i].max(o[off::Q + i]));
                        speed = speed.max(o[off::QD + i].abs());
                    }
                }
                let o = s.publish().to_vec();
                assert!(
                    speed < 1e-2 && (0..n).all(|i| hi[i] - lo[i] < 1e-3),
                    "{n} joints, {kind:?} at {factor}·reach: still moving ({speed} rad/s)"
                );
                let (tool, aimed) = (o[off::TOOL_P + 1].atan2(o[off::TOOL_P]), target[1].atan2(target[0]));
                let r = o[off::TOOL_P].hypot(o[off::TOOL_P + 1]);
                assert!(
                    (tool - aimed).abs() < 0.02 && r > 0.99 * reach,
                    "{n} joints, {kind:?} at {factor}·reach: tool at radius {r}, {tool} rad vs {aimed}"
                );
            }
        }
    }
}

fn spring(a: Option<usize>, s_a: f64, b: usize, s_b: f64, ground: [f64; 2], [k, l0, d]: [f64; 3]) -> Element {
    Element { a, s_a, b, s_b, ground, k, l0, d }
}

/// The two-link arm with a spring from the ground to its tip and a spring–damper across the elbow.
fn sprung_arm(damping: f64) -> Desc {
    let mut d = frictionless(presets::arm2());
    d.elements = vec![
        spring(None, 0.0, 1, 1.0, [1.5, 0.5], [40.0, 0.3, damping]),
        spring(Some(0), 0.5, 1, 0.5, [0.0; 2], [20.0, 0.2, damping]),
    ];
    d
}

#[test]
fn springs_conserve_energy_and_dampers_close_the_budget() {
    let mut s = with(&sprung_arm(0.0), Kind::Passive);
    s.set_state(0, 0.4, 2.0);
    let e0 = total_energy(&mut s);
    s.step(5_000);
    let drift = (total_energy(&mut s) - e0).abs() / e0.abs().max(1.0);
    assert!(drift < 1e-6, "relative energy drift with springs {drift:e}");

    let mut s = with(&sprung_arm(2.0), Kind::Passive);
    s.kick(0, 2.0);
    s.step(5_000);
    let o = s.publish().to_vec();
    assert!(o[off::WORK + 3] > 0.1, "the dampers must dissipate: {}", o[off::WORK + 3]);
    assert!(o[off::BUDGET].abs() < 1e-6, "residual {:e}", o[off::BUDGET]);
}

#[test]
fn a_mass_hanging_on_a_spring_settles_where_the_spring_carries_its_weight() {
    // a point mass on a vertical slider, hung from a ground point 1 m above the slider's origin
    let mut mass = presets::link(None, JointKind::Prismatic, 2.0, 0.0);
    (mass.mount_angle, mass.actuated, mass.shape, mass.limit) = (FRAC_PI_2, false, 1, 0.0);
    mass.values = [2.0, 0.0, 0.0, 0.01, 0.0];
    let (k, l0) = (100.0, 0.2);
    let d = Desc {
        bodies: vec![mass],
        g: 9.81,
        tool: None,
        closures: Vec::new(),
        elements: vec![spring(None, 0.0, 0, 0.0, [0.0, 1.0], [k, l0, 5.0])],
    };
    let mut s = with(&d, Kind::Passive);
    s.step(20_000);
    let expected = 1.0 - l0 - 2.0 * 9.81 / k;
    let q = s.primary().q[0];
    assert!((q - expected).abs() < 1e-6, "rests at {q}, expected {expected}");
}

#[test]
fn gravity_compensation_also_cancels_the_springs() {
    let mut s = with(&sprung_arm(1.0), Kind::PdGravity);
    s.set_target(0, 0.3);
    s.set_target(1, 0.8);
    s.step(10_000);
    let q = &s.primary().q[..2];
    assert!((q[0] - 0.3).abs() < 1e-3 && (q[1] - 0.8).abs() < 1e-3, "q = {q:?}");
}

/// A driven arm with a passive branch at `s` along it (`mount`: its mounting angle, `beta`: its joint
/// friction, `target`: its target), and a driven forearm at its tip; the arm starts 0.4 rad up, the branch
/// swung out by 0.6 rad (the Branching arm preset).
fn branching(mount: f64, s: f64, beta: f64, target: f64) -> Desc {
    let mut arm = presets::link(None, JointKind::Revolute, 1.0, 1.0);
    arm.q0 = 0.4;
    let mut branch = presets::link(Some(0), JointKind::Revolute, 0.6, 0.7);
    (branch.mount_s, branch.mount_angle, branch.actuated, branch.values[4]) = (s, mount, false, beta);
    (branch.q0, branch.q_target) = (0.6, target);
    let mut forearm = presets::link(Some(0), JointKind::Revolute, 0.6, 0.7);
    (forearm.mount_s, forearm.mount_angle) = (1.0, -0.6);
    Desc { bodies: vec![arm, branch, forearm], g: 9.81, tool: Some(2), closures: vec![], elements: vec![] }
}

#[test]
fn a_passive_joint_settles_into_equilibrium_and_lqr_carries_it_there() {
    // the branch's target (0.8 rad above the arm) is no equilibrium: q* lets it settle where gravity
    // balances (hanging), and LQR then swings the arm there with the pendulum in tow
    let mut s = with(&branching(-FRAC_PI_2, 0.5, 0.02, 0.8), Kind::Lqr);
    s.step(1);
    let o = s.publish().to_vec();
    assert!(o[off::EQ_RESIDUAL] < 1e-8, "q* is no equilibrium: residual {}", o[off::EQ_RESIDUAL]);
    let hanging = o[off::Q_TARGET + 1];
    assert!(hanging.abs() < 1e-6, "the branch should hang straight down at q* (θ₂* = {hanging})");
    // the arm returns and stays (no spin); the carried pendulum is damped only through the arm and its joint
    // friction, so its swing decays slowly — compare the first and the last five seconds
    let (mut early, mut late, mut arm) = (0.0_f64, 0.0_f64, 0.0_f64);
    for k in 0..20_000 {
        s.step(1);
        let q = &s.primary().q;
        if k < 5_000 {
            early = early.max(q[1].abs());
        } else if k >= 15_000 {
            late = late.max(q[1].abs());
            arm = arm.max(q[0].abs()).max(q[2].abs());
        }
    }
    assert!(arm < 0.05, "the arm should settle at its target: deviation {arm}");
    assert!(late < 0.5 * early, "the pendulum's swing should decay: {early} → {late}");
}

#[test]
fn lqr_refuses_an_equilibrium_that_no_input_can_hold() {
    // a frictionless passive bar standing upright on the arm's own pivot: turning the arm cannot move it
    let d = branching(FRAC_PI_2, 0.0, 0.0, 0.0);
    let mut s = sim(&d);
    s.set_controller(0, Some(Kind::Lqr));
    assert_eq!(s.primary().kind, Kind::Passive, "LQR must not take on an unstabilisable equilibrium");
    s.set_controller(0, Some(Kind::Mpc));
    assert_eq!(s.primary().kind, Kind::Passive, "nor MPC");
}
