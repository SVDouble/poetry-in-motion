//! Regressions for settings, design-cache lifetime, and rejected native inputs.
use motion_core::control::Kind;
use motion_core::layout::{Flag, off, slot};
use motion_core::mech::{JointKind, presets};
use motion_core::model::Model;
use motion_core::world::World;

fn world() -> World {
    World::new(Model::from_desc(&presets::arm2()))
}

#[test]
fn shadow_creation_and_sync_inherit_configuration_and_recompute_equilibrium() {
    let mut w = world();
    assert!(w.set_param(0, 2.4));
    assert!(w.set_flag(Flag::MotorSpeed, 1.0));
    assert!(w.set_flag(Flag::NullDamping, 13.0));
    assert!(w.set_flag(Flag::RefWn, 5.0));
    assert!(w.set_target(0, 0.7));
    assert!(w.set_state(0, 0.2, 5.0));
    assert!(w.set_input(0, 100.0));
    w.set_controller(1, Some(Kind::Passive));
    for _ in 0..2 {
        let p = w.primary();
        let s = w.slot(1).unwrap();
        assert!((s.plan.tuning.motor_speed - 1.0).abs() < 1e-12);
        assert!((s.plan.tuning.null_damping - 13.0).abs() < 1e-12);
        assert_eq!(s.p, p.p);
        assert_eq!(s.plan.q_target, p.plan.q_target);
        assert_eq!(s.plan.q_star, p.plan.q_star);
        assert_eq!(s.plan.m_star, p.plan.m_star);
        assert_eq!(s.q, p.q);
        assert_eq!(s.qd, p.qd);
        w.sync_shadows();
    }
    w.step(1);
    let o = w.publish();
    assert!((o[off::U] - o[off::SLOT + slot::LEN + slot::U]).abs() < 1e-12);
}

#[test]
fn rebuild_retains_settings_but_resets_mechanism_relative_targets() {
    let mut w = world();
    for (flag, value) in [
        (Flag::MotorSpeed, 3.0),
        (Flag::RefWn, 7.0),
        (Flag::PushK, 42.0),
        (Flag::Posture, 9.0),
        (Flag::Orient, 5.0),
        (Flag::Elbow, 6.0),
        (Flag::Umax, 80.0),
        (Flag::OrientTarget, 2.0),
        (Flag::ElbowBody, 1.0),
        (Flag::PushX, 2.0),
    ] {
        assert!(w.set_flag(flag, value));
    }
    assert!(w.set_target(0, 1.2));
    assert!(w.set_dt(0.002));
    w.set_controller(1, Some(Kind::Passive));
    let mut desc = presets::arm2();
    desc.bodies.truncate(1);
    desc.tool = Some(0);
    w.rebuild(Model::from_desc(&desc));
    for i in 0..2 {
        let s = w.slot(i).unwrap();
        assert!((s.plan.tuning.motor_speed - 3.0).abs() < 1e-12);
        assert!((s.plan.ref_wn - 7.0).abs() < 1e-12);
        assert!((s.plan.push_k - 42.0).abs() < 1e-12);
        assert!((s.plan.goals.posture - 9.0).abs() < 1e-12);
        assert!((s.plan.goals.orient - 5.0).abs() < 1e-12);
        assert!((s.plan.goals.elbow - 6.0).abs() < 1e-12);
        assert!(s.plan.goals.orient_target.abs() < 1e-12);
        assert_eq!(s.plan.goals.elbow_body, 0);
        assert!(s.plan.push.is_none());
        assert!(s.plan.q_target[0].abs() < 1e-12);
        assert!((s.dt - 0.002).abs() < 1e-12);
    }
}

#[test]
fn timestep_change_redesigns_an_already_designed_lqr() {
    let mut changed = world();
    changed.set_controller(0, Some(Kind::Lqr));
    let before = changed.publish().to_vec();
    assert!(changed.set_dt(0.01));
    let after = changed.publish().to_vec();
    let mut fresh = world();
    assert!(fresh.set_dt(0.01));
    fresh.set_controller(0, Some(Kind::Lqr));
    let expected = fresh.publish();
    let (at, len) = Kind::Lqr.spec().telemetry_field("k").unwrap();
    let start = off::SLOT + slot::TELEMETRY + at;
    assert!(
        before[start..start + len].iter().zip(&after[start..start + len]).any(|(a, b)| (a - b).abs() > 1e-5)
    );
    for i in start..start + len {
        assert!((after[i] - expected[i]).abs() < 1e-10);
    }
}

#[test]
fn invalid_native_descriptions_and_live_edits_do_not_change_a_valid_world() {
    let mut invalid = presets::arm2();
    invalid.bodies[0].parent = Some(1);
    assert!(Model::try_from_desc(&invalid).is_err());
    invalid = presets::arm2();
    invalid.bodies[0].values[0] = -1.0;
    assert!(Model::try_from_desc(&invalid).is_err());
    let mut w = world();
    w.set_controller(1, Some(Kind::Passive));
    let before = w.publish().to_vec();
    assert!(!w.set_param(0, -1.0));
    assert!(!w.set_param(0, f64::INFINITY));
    assert!(!w.set_state(0, f64::NAN, 0.0));
    assert!(!w.kick(0, f64::MAX));
    assert!(!w.set_flag(Flag::MotorSpeed, f64::NAN));
    assert!(!w.set_flag(Flag::ElbowBody, 0.5));
    assert!(!w.set_dt(f64::NAN));
    assert!(!w.set_input(0, f64::INFINITY));
    assert_eq!(w.publish(), before);
}

#[test]
fn point_masses_zero_gravity_and_massless_fixed_bodies_are_valid() {
    let mut desc = presets::arm2();
    desc.bodies.truncate(1);
    desc.bodies[0].kind = JointKind::Prismatic;
    desc.bodies[0].values = [1.0, 0.0, 0.0, 0.0, 0.0];
    desc.g = 0.0;
    desc.tool = Some(0);
    assert!(Model::try_from_desc(&desc).is_ok());
    let mut fixed = presets::link(Some(0), JointKind::Fixed, 0.0, 0.0);
    fixed.values = [0.0; 5];
    desc.bodies.push(fixed);
    assert!(Model::try_from_desc(&desc).is_ok());
    desc.bodies[0].values[0] = 0.0;
    assert!(Model::try_from_desc(&desc).is_err());
}

#[test]
fn closed_loop_shadows_keep_the_primary_target_assembly_after_a_disturbance() {
    for desc in [presets::four_bar(), presets::slider_crank()] {
        let mut w = World::new(Model::from_desc(&desc));
        let target = w.primary().plan.q_star;
        assert!(w.set_state(0, -1.5, 0.0));
        assert!(w.set_flag(Flag::Baumgarte, 40.0));
        assert!(w.set_param(0, 1.2));
        w.set_controller(1, Some(Kind::Passive));
        let shadow = w.slot(1).unwrap();
        for (a, b) in target.iter().zip(shadow.plan.q_star) {
            assert!(
                (a - b).abs() < 1e-7,
                "different target assembly: {target:?} vs {:?}",
                shadow.plan.q_star
            );
        }
    }
}
