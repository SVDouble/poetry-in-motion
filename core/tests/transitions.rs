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
        assert!((s.plan.config.tuning.motor_speed - 1.0).abs() < 1e-12);
        assert!((s.plan.config.tuning.null_damping - 13.0).abs() < 1e-12);
        assert_eq!(s.p, p.p);
        assert_eq!(s.plan.config.q_target, p.plan.config.q_target);
        assert_eq!(s.plan.design.q_star, p.plan.design.q_star);
        assert_eq!(s.plan.design.m_star, p.plan.design.m_star);
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
        assert!((s.plan.config.tuning.motor_speed - 3.0).abs() < 1e-12);
        assert!((s.plan.config.ref_wn - 7.0).abs() < 1e-12);
        assert!((s.plan.config.push_k - 42.0).abs() < 1e-12);
        assert!((s.plan.config.goals.posture - 9.0).abs() < 1e-12);
        assert!((s.plan.config.goals.orient - 5.0).abs() < 1e-12);
        assert!((s.plan.config.goals.elbow - 6.0).abs() < 1e-12);
        assert!(s.plan.config.goals.orient_target.abs() < 1e-12);
        assert_eq!(s.plan.config.goals.elbow_body, 0);
        assert!(s.plan.config.push.is_none());
        assert!(s.plan.config.q_target[0].abs() < 1e-12);
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
    for flag in [Flag::Coriolis, Flag::UseTool, Flag::PushOn] {
        for value in [0.5, f64::EPSILON / 2.0, 1.0 - f64::EPSILON / 2.0] {
            assert!(!w.set_flag(flag, value), "{flag:?} accepted non-boolean {value}");
        }
    }
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
        let target = w.primary().plan.design.q_star;
        assert!(w.set_state(0, -1.5, 0.0));
        assert!(w.set_flag(Flag::Baumgarte, 40.0));
        assert!(w.set_param(0, 1.2));
        w.set_controller(1, Some(Kind::Passive));
        let shadow = w.slot(1).unwrap();
        for (a, b) in target.iter().zip(shadow.plan.design.q_star) {
            assert!(
                (a - b).abs() < 1e-7,
                "different target assembly: {target:?} vs {:?}",
                shadow.plan.design.q_star
            );
        }
    }
}

#[test]
fn ordered_tasks_support_body_frames_and_follow_shadow_lifetimes() {
    use motion_core::plan::{TaskFrame, TaskGoal};
    let mut w = world();
    let task = TaskGoal::Orientation { frame: TaskFrame::Body(0), target: 0.3, wn: 2.0 };
    w.primary_mut().plan.config.tasks.push(task);
    w.set_controller(1, Some(Kind::Impedance));
    assert_eq!(w.slot(1).unwrap().plan.config.tasks.len(), 1);
    let p = w.primary_mut();
    let tasks = p.plan.sub_tasks(&mut p.model, &p.q, &p.qd);
    assert_eq!(tasks.len(), 1);
    assert_eq!(tasks[0].rows, 1);
    let expected = 4.0 * motion_core::plan::wrap(0.3 - p.model.frame_angles[1].eval(&p.q));
    assert!((tasks[0].a[0] - expected).abs() < 1e-12);
    assert!(
        TaskGoal::Point { frame: TaskFrame::Body(100), target: [0.0; 2], wn: 2.0 }
            .evaluate(&mut p.model, &p.q, &p.qd)
            .is_none()
    );
    w.rebuild(Model::from_desc(&presets::four_bar()));
    assert!(w.primary().plan.config.tasks.is_empty());
    assert!(w.slot(1).unwrap().plan.config.tasks.is_empty());
}

#[test]
fn closed_loop_hierarchies_remain_feasible_with_conflicting_lower_goals() {
    for kind in [Kind::Vmc, Kind::Impedance, Kind::Admittance] {
        let mut w = World::new(Model::from_desc(&presets::four_bar()));
        w.set_controller(0, Some(kind));
        assert_eq!(w.primary().kind, kind);
        w.set_flag(Flag::Orient, 3.0);
        w.set_flag(Flag::OrientTarget, 1.0);
        w.set_flag(Flag::Elbow, 2.0);
        w.set_flag(Flag::ElbowX, -0.4);
        w.set_flag(Flag::ElbowY, 0.6);
        for _ in 0..20 {
            w.step(100);
            let o = w.publish();
            assert!(o[off::CONSTRAINT] < 1e-6, "{kind:?} violated loop");
            assert!(o[off::U].is_finite() && o[off::U].abs() <= 120.0);
            if let Some((at, _)) = kind.spec().telemetry_field("leak_sec") {
                assert!(o[off::SLOT + slot::TELEMETRY + at] < 1e-8, "{kind:?} leaked");
            }
        }
    }
}

#[test]
fn benchmark_setup_and_steps_leave_the_live_simulation_untouched() {
    let mut w = world();
    w.set_controller(0, Some(Kind::ComputedTorque));
    w.step(100);
    let before = w.publish().to_vec();
    let mut bench = w.benchmark();
    bench.run(1000);
    assert_eq!(before, w.publish());
}

#[test]
fn integration_recovery_is_observable_and_reset_starts_new_counters() {
    let mut desc = presets::arm2();
    desc.bodies.truncate(1);
    desc.tool = Some(0);
    let mut w = World::new(Model::from_desc(&desc));
    assert!(w.set_state(0, 0.0, 1e6));
    w.set_controller(1, Some(Kind::Passive));
    w.step(1);
    let o = w.publish();
    assert!((o[off::RECOVERY] - 1.0).abs() < 1e-12);
    assert!((o[off::RECOVERY + 2] - o[off::TIME]).abs() < 1e-12);
    assert!(o[off::QD].abs() < 1e-12);
    assert!((o[off::SLOT + slot::LEN + slot::RECOVERY] - 1.0).abs() < 1e-12);
    w.reset();
    let o = w.publish();
    assert_eq!(&o[off::RECOVERY..off::RECOVERY + 3], &[0.0, 0.0, -1.0]);
}

#[test]
fn authored_parameter_candidates_match_fresh_derivation_and_preserve_old_model() {
    for desc in [presets::arm2(), presets::arm7(), presets::four_bar(), presets::cartpole()] {
        let mut original = Model::from_desc(&desc);
        let old = original.derivation.clone();
        let mut edited = desc;
        edited.g = 3.0;
        for body in &mut edited.bodies {
            body.values[0] *= 1.2;
            body.values[1] *= 0.95;
            body.values[2] = body.values[1] / 2.0;
            body.values[3] = body.values[0] * body.values[1].powi(2) / 12.0;
            body.q0 += 0.02;
            body.q_target += 0.03;
        }
        let candidate = original.prepare(&edited).unwrap();
        assert!(candidate.reused);
        let mut updated = candidate.model;
        let mut fresh = Model::from_desc(&edited);
        assert_eq!(updated.derivation, fresh.derivation);
        assert_eq!(updated.q0, fresh.q0);
        assert_eq!(updated.q_target, fresh.q_target);
        assert_eq!(updated.defaults, fresh.defaults);
        let q = [0.2; motion_core::MAXN];
        let v = [0.1; motion_core::MAXN];
        assert_eq!(updated.terms(&q, &v).m, fresh.terms(&q, &v).m);
        assert_eq!(updated.energies(&q, &v), fresh.energies(&q, &v));
        assert_eq!(original.derivation, old);
        assert_ne!(updated.energies(&q, &v), original.energies(&q, &v));
        edited.bodies[0].mount_angle += 0.1;
        assert!(!original.prepare(&edited).unwrap().reused);
    }
}

#[test]
fn checkpoints_reproduce_controller_memory_settings_and_independent_shadows() {
    use motion_core::history::History;
    for desc in [presets::arm2(), presets::cartpole(), presets::four_bar()] {
        for kind in Kind::ALL {
            let mut w = World::new(Model::from_desc(&desc));
            if kind.applicable(&w.primary().model, &w.primary().plan).is_err() {
                continue;
            }
            w.set_controller(0, Some(kind));
            w.set_controller(1, Some(Kind::PdGravity));
            w.set_flag(Flag::MotorSpeed, 15.0);
            w.set_param(0, 1.7);
            w.step(103);
            w.publish();
            let mut history = History::default();
            let id = history.save(&w);
            assert_ne!(id, 0);
            w.step(137);
            let expected = w.publish().to_vec();
            let future = history.save(&w);
            w.set_flag(Flag::MotorSpeed, 1.0);
            w.set_controller(1, None);
            assert!(history.restore(id, &mut w));
            assert!(!history.restore(future, &mut w));
            assert!((w.primary().t - 0.103).abs() < 1e-12);
            w.step(137);
            assert_eq!(expected, w.publish(), "{kind:?} checkpoint lost execution state");
            history.clear();
            assert!(!history.restore(id, &mut w));
            assert_ne!(history.save(&w), id);
        }
    }
}

#[test]
fn additional_task_roles_do_not_overwrite_builtin_error_telemetry() {
    use motion_core::plan::TaskRole;
    let mut w = World::new(Model::from_desc(&presets::arm7()));
    w.set_controller(0, Some(Kind::Impedance));
    w.set_flag(Flag::OrientTarget, 0.7);
    w.set_flag(Flag::Elbow, 2.0);
    w.set_flag(Flag::ElbowX, -0.4);
    w.set_flag(Flag::ElbowY, 0.6);
    assert!(w.set_tasks(&[0.0, 0.0, -1.2, 0.0, 2.0, 1.0, 1.0, 0.3, -0.2, 2.0]));
    for orient in [2.0, 0.0] {
        w.set_flag(Flag::Orient, orient);
        let p = w.primary_mut();
        let tasks = p.plan.sub_tasks(&mut p.model, &p.q, &p.qd);
        let roles = tasks.iter().map(|t| t.role).collect::<Vec<_>>();
        let mut expected_roles = vec![TaskRole::Elbow, TaskRole::Additional, TaskRole::Additional];
        if orient > 0.0 {
            expected_roles.insert(0, TaskRole::Orientation);
        }
        assert_eq!(roles, expected_roles);
        let orientation = tasks.iter().find(|t| t.role == TaskRole::Orientation).map_or(0.0, |t| t.e[0]);
        let elbow = tasks.iter().find(|t| t.role == TaskRole::Elbow).unwrap().e;
        w.step(1);
        let out = w.publish();
        let field =
            |name| off::SLOT + slot::TELEMETRY + Kind::Impedance.spec().telemetry_field(name).unwrap().0;
        assert!((out[field("orient_err")] - orientation).abs() < 1e-12);
        assert_eq!(&out[field("elbow_err")..field("elbow_err") + 2], &elbow);
    }
}
