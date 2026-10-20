//! Property tests on randomly generated mechanisms (seeded, reproducible): the physics invariants hold for
//! any topology, and every controller runs on every mechanism it claims to support without panicking,
//! producing NaN or blowing up — with shadows, pushes, parameter changes and flag spam on top.

use core::f64::consts::PI;

use motion_core::control::Kind;
use motion_core::layout::{FIELDS, Flag, SLOT_FIELDS, SLOTS, off, slot};
use motion_core::mech::{Closure, ClosureKind, Desc, JointKind, presets};
use motion_core::model::Model;
use motion_core::world::World;

/// xorshift64*: tiny, deterministic.
struct Rng(u64);

impl Rng {
    fn next(&mut self) -> u64 {
        self.0 ^= self.0 >> 12;
        self.0 ^= self.0 << 25;
        self.0 ^= self.0 >> 27;
        self.0.wrapping_mul(0x2545_F491_4F6C_DD1D)
    }
    fn unit(&mut self) -> f64 {
        (self.next() >> 11) as f64 / (1u64 << 53) as f64
    }
    fn range(&mut self, lo: f64, hi: f64) -> f64 {
        lo + (hi - lo) * self.unit()
    }
    fn below(&mut self, n: usize) -> usize {
        (self.next() % n as u64) as usize
    }
    fn chance(&mut self, p: f64) -> bool {
        self.unit() < p
    }
}

/// A random tree of 1–7 bodies (some fixed, some prismatic), optionally closed by a pin or slot.
fn mechanism(r: &mut Rng, friction: bool) -> Desc {
    let nb = 1 + r.below(7);
    let mut bodies = Vec::new();
    let mut moving = 0;
    for i in 0..nb {
        let parent = if i == 0 || r.chance(0.15) { None } else { Some(r.below(i)) };
        let kind = match r.below(10) {
            0 if i > 0 => JointKind::Fixed,
            1 | 2 => JointKind::Prismatic,
            _ => JointKind::Revolute,
        };
        let mut b = presets::link(parent, kind, r.range(0.2, 2.0), r.range(0.3, 1.2));
        b.mount_s = if parent.is_some() { [0.0, 0.5, 1.0][r.below(3)] } else { 0.0 };
        b.mount_xy = [r.range(-1.0, 1.0), r.range(-0.5, 0.5)];
        b.mount_angle = r.range(-PI, PI);
        b.axis_angle = if kind == JointKind::Prismatic { r.range(-1.0, 1.0) } else { 0.0 };
        b.actuated = r.chance(0.7);
        b.values[2] = b.values[1] * r.range(0.2, 0.8);
        b.values[4] = if friction { r.range(0.0, 0.2) } else { 0.0 };
        b.q0 = r.range(-1.0, 1.0);
        b.limit = if kind == JointKind::Prismatic && r.chance(0.5) { r.range(0.8, 2.0) } else { 0.0 };
        if kind != JointKind::Fixed {
            moving += 1;
        }
        bodies.push(b);
    }
    if moving == 0 {
        bodies[0].kind = JointKind::Revolute;
    }
    let tool = if r.chance(0.8) { Some(r.below(nb)) } else { None };
    let mut closures = Vec::new();
    if nb >= 2 && r.chance(0.25) {
        // close the last body's tip onto the ground right where it starts: a loop that closes at q0
        let kind =
            if r.chance(0.5) { ClosureKind::Pin } else { ClosureKind::Slot { angle: r.range(-PI, PI) } };
        closures.push(Closure { kind, a: None, s_a: 0.0, b: nb - 1, s_b: 1.0, ground: [0.0; 2] });
    }
    Desc { bodies, g: 9.81, tool, closures }
}

/// Place a ground closure at the tip's starting position so the loop closes at q0.
fn close_at_start(d: &mut Desc) -> Option<World> {
    if d.closures.is_empty() {
        return Some(World::new(Model::from_desc(d)));
    }
    let open = Desc { closures: Vec::new(), ..d.clone() };
    let mut w = World::new(Model::from_desc(&open));
    let o = w.publish();
    let b = d.closures[0].b;
    let frame = off::FRAMES + 3 * (b + 1);
    let (x, y, phi) = (o[frame], o[frame + 1], o[frame + 2]);
    let l = d.bodies[b].values[1];
    d.closures[0].ground = [x + l * phi.cos(), y + l * phi.sin()];
    let mut w = World::new(Model::from_desc(d));
    (w.publish()[off::CONSTRAINT] < 1e-6).then_some(w)
}

fn finite(o: &[f64]) -> bool {
    o.iter().all(|v| v.is_finite())
}

/// Names of the snapshot fields holding non-finite values (slot fields as `slot<i>.<FIELD>`).
fn bad_fields(o: &[f64]) -> Vec<String> {
    let mut bad = Vec::new();
    for &(name, at, len) in FIELDS {
        if name == "SLOT" {
            for s in 0..SLOTS {
                for &(f, a, l) in SLOT_FIELDS {
                    let b = at + s * slot::LEN + a;
                    if o[b..b + l].iter().any(|v| !v.is_finite()) {
                        bad.push(format!("slot{s}.{f}"));
                    }
                }
            }
        } else if o[at..at + len].iter().any(|v| !v.is_finite()) {
            bad.push(name.to_owned());
        }
    }
    bad
}

/// Mechanisms per property: `FUZZ_CASES`, else fewer in debug builds (`make test`) than in release.
fn cases() -> u64 {
    std::env::var("FUZZ_CASES").ok().and_then(|v| v.parse().ok()).unwrap_or(if cfg!(debug_assertions) {
        12
    } else {
        120
    })
}

#[test]
fn physics_invariants_hold_for_random_mechanisms() {
    let mut checked = 0;
    let cases = cases();
    for seed in 1..=cases {
        let mut r = Rng(seed.wrapping_mul(0x9E37_79B9_7F4A_7C15));
        let mut d = mechanism(&mut r, false);
        let Some(mut w) = close_at_start(&mut d) else { continue };
        checked += 1;
        let n = w.primary().n();
        for i in 0..n {
            w.kick(i, r.range(-1.5, 1.5));
        }
        let e0 = w.publish()[off::ENERGY + 2];
        for _ in 0..20 {
            w.step(100);
            let o = w.publish();
            assert!(finite(o), "seed {seed}: non-finite {:?}", bad_fields(o));
            assert!(o[off::SKEW] < 1e-8, "seed {seed}: Ṁ − 2C not skew ({:e})", o[off::SKEW]);
            for i in 0..n {
                assert!(o[off::EIG_M + i] > 0.0, "seed {seed}: M not positive definite");
                let lhs = o[off::MQDD + i] + o[off::CQD + i] + o[off::N + i] + o[off::AT_LAMBDA + i];
                let scale = 1.0 + o[off::TAU + i].abs() + o[off::MQDD + i].abs();
                assert!(
                    (lhs - o[off::TAU + i]).abs() < 1e-6 * scale,
                    "seed {seed} row {i}: EOM off by {:e}",
                    lhs - o[off::TAU + i]
                );
            }
        }
        let o = w.publish();
        let scale = 1.0 + e0.abs() + o[off::ENERGY].abs();
        // frictionless and unforced (end stops store and return energy): the budget closes and E is kept
        assert!(
            o[off::BUDGET].abs() < 1e-4 * scale,
            "seed {seed}: budget residual {:e} (n = {n}, loops {})",
            o[off::BUDGET],
            d.closures.len()
        );
        let ends = o[off::WORK + 2].abs() < 1e-9;
        if ends && d.closures.is_empty() {
            assert!(
                (o[off::ENERGY + 2] - e0).abs() < 1e-4 * scale,
                "seed {seed}: energy drift {:e}",
                o[off::ENERGY + 2] - e0
            );
        }
        if !d.closures.is_empty() {
            assert!(o[off::CONSTRAINT] < 1e-5, "seed {seed}: loop opened by {:e}", o[off::CONSTRAINT]);
        }
    }
    assert!(checked > cases as usize / 2, "only {checked} mechanisms could be checked");
}

#[test]
fn every_controller_runs_on_every_mechanism_it_supports() {
    let mut runs = 0;
    let cases = cases();
    for seed in 1..=cases {
        let mut r = Rng(seed.wrapping_mul(0xD1B5_4A32_D192_ED03));
        let mut d = mechanism(&mut r, true);
        let Some(mut w) = close_at_start(&mut d) else { continue };
        let n = w.primary().n();
        for &kind in &Kind::ALL {
            w.set_controller(0, Some(kind));
            if w.primary().kind != kind {
                continue; // not applicable here
            }
            runs += 1;
            // up to four shadows of other applicable controllers
            for s in 1..SLOTS {
                w.set_controller(s, Some(Kind::ALL[r.below(Kind::ALL.len())]));
            }
            w.reset();
            for i in 0..n {
                w.set_target(i, r.range(-0.5, 0.5));
            }
            if r.chance(0.5) {
                w.set_flag(Flag::TargetX, r.range(-1.0, 1.0));
                w.set_flag(Flag::TargetY, r.range(-1.0, 1.0));
                w.set_flag(Flag::UseTool, 1.0);
            }
            for step in 0..30 {
                match r.below(10) {
                    0 => w.kick(r.below(n), r.range(-2.0, 2.0)),
                    1 => {
                        w.set_flag(Flag::PushX, r.range(-2.0, 2.0));
                        w.set_flag(Flag::PushY, r.range(-2.0, 2.0));
                    }
                    2 => w.set_flag(Flag::PushOn, 0.0),
                    3 => {
                        let spec = kind.spec();
                        if !spec.params.is_empty() {
                            let i = r.below(spec.params.len());
                            let p = &spec.params[i];
                            w.set_ctrl_param(kind, i, r.range(p.min, p.max));
                        }
                    }
                    4 => w.set_flag(Flag::Umax, r.range(1.0, 200.0)),
                    5 => w.set_flag(Flag::Posture, r.range(0.0, 20.0)),
                    6 => w.set_param(r.below(5 * d.bodies.len()), r.range(0.1, 1.5)),
                    7 => {
                        // secondary goals, reachable or not
                        w.set_flag(Flag::Orient, if r.chance(0.7) { r.range(1.0, 12.0) } else { 0.0 });
                        w.set_flag(Flag::OrientTarget, r.range(-PI, PI));
                    }
                    8 => {
                        w.set_flag(Flag::Elbow, if r.chance(0.7) { r.range(1.0, 12.0) } else { 0.0 });
                        w.set_flag(Flag::ElbowX, r.range(-1.5, 1.5));
                        w.set_flag(Flag::ElbowY, r.range(-1.5, 1.5));
                        w.set_flag(Flag::ElbowBody, r.below(d.bodies.len()) as f64);
                    }
                    _ => {}
                }
                w.step(50);
                let o = w.publish();
                let kinds: Vec<f64> = (0..SLOTS).map(|s| o[off::SLOT + s * slot::LEN + slot::KIND]).collect();
                assert!(
                    finite(o),
                    "seed {seed}, {kind:?}, step {step}: non-finite {:?} (slot kinds {kinds:?})",
                    bad_fields(o)
                );
                for s in 0..SLOTS {
                    let b = off::SLOT + s * slot::LEN;
                    if o[b + slot::KIND] >= 0.0 {
                        assert!(
                            o[b + slot::Q..b + slot::Q + n].iter().all(|q| q.abs() < 1e4),
                            "seed {seed}, slot {s}: diverged"
                        );
                    }
                }
            }
            for s in 1..SLOTS {
                w.set_controller(s, None);
            }
            w.set_flag(Flag::PushOn, 0.0);
            w.set_flag(Flag::UseTool, 0.0);
            w.set_flag(Flag::Umax, 120.0);
            w.set_flag(Flag::Posture, 4.0);
            w.set_flag(Flag::Orient, 0.0);
            w.set_flag(Flag::Elbow, 0.0);
        }
    }
    assert!(runs > 2 * cases as usize, "only {runs} controller runs");
}

/// The derived equations against Euler–Lagrange computed numerically from the compiled energy alone:
/// `d/dt ∂L/∂q̇ − ∂L/∂q = M q̈ + C q̇ + ∂V/∂q` (frictionless, so N = ∂V/∂q) for random states.
#[test]
fn derived_equations_match_euler_lagrange_numerically() {
    use motion_core::dynamics::{Terms, mat_vec};
    let mut checked = 0;
    for seed in 1..=cases() {
        let mut r = Rng(seed.wrapping_mul(0x94D0_49BB_1331_11EB));
        let mut d = mechanism(&mut r, false);
        d.closures.clear();
        let mut model = Model::from_desc(&d);
        let n = model.n;
        let lagrangian = |model: &mut Model, q: &[f64], qd: &[f64]| {
            let mut e = [0.0; 2];
            model.energy.eval(q, qd, &mut e);
            e[0] - e[1]
        };
        for _ in 0..4 {
            let q: Vec<f64> = (0..n).map(|_| r.range(-1.5, 1.5)).collect();
            let qd: Vec<f64> = (0..n).map(|_| r.range(-2.0, 2.0)).collect();
            let qdd: Vec<f64> = (0..n).map(|_| r.range(-3.0, 3.0)).collect();
            let h = 1e-4;
            let shift = |v: &[f64], i: usize, s: f64| -> Vec<f64> {
                v.iter().enumerate().map(|(k, x)| if k == i { x + s } else { *x }).collect()
            };
            // ∂L/∂q̇ᵢ at a state, by central differences
            let dl_dqd = |model: &mut Model, q: &[f64], qd: &[f64], i: usize| {
                (lagrangian(model, q, &shift(qd, i, h)) - lagrangian(model, q, &shift(qd, i, -h))) / (2.0 * h)
            };
            let terms = Terms::eval(&mut model.dynamics, n, &q, &qd);
            let mut mq = [0.0; motion_core::MAXN];
            mq[..n].copy_from_slice(&qdd);
            let mqdd = mat_vec(n, &terms.m, &mq);
            let cqd = terms.c_qd(n, &qd);
            for i in 0..n {
                // d/dt along (q̇, q̈)
                let ahead: (Vec<f64>, Vec<f64>) = (
                    (0..n).map(|k| q[k] + h * qd[k]).collect(),
                    (0..n).map(|k| qd[k] + h * qdd[k]).collect(),
                );
                let back: (Vec<f64>, Vec<f64>) = (
                    (0..n).map(|k| q[k] - h * qd[k]).collect(),
                    (0..n).map(|k| qd[k] - h * qdd[k]).collect(),
                );
                let ddt = (dl_dqd(&mut model, &ahead.0, &ahead.1, i)
                    - dl_dqd(&mut model, &back.0, &back.1, i))
                    / (2.0 * h);
                let dl_dq = (lagrangian(&mut model, &shift(&q, i, h), &qd)
                    - lagrangian(&mut model, &shift(&q, i, -h), &qd))
                    / (2.0 * h);
                let numeric = ddt - dl_dq;
                let derived = mqdd[i] + cqd[i] + terms.n[i];
                let scale = 1.0 + numeric.abs() + mqdd[i].abs() + terms.n[i].abs();
                assert!(
                    (numeric - derived).abs() < 1e-4 * scale,
                    "seed {seed} row {i}: Euler–Lagrange {numeric} vs derived {derived}"
                );
            }
        }
        checked += 1;
    }
    assert_eq!(checked as u64, cases());
}

/// Fully actuated open chains: the tracking controllers reach random joint targets.
#[test]
fn tracking_controllers_reach_random_targets() {
    let mut runs = 0;
    for seed in 1..=cases() {
        let mut r = Rng(seed.wrapping_mul(0xBF58_476D_1CE4_E5B9));
        let mut d = mechanism(&mut r, true);
        d.closures.clear();
        for b in &mut d.bodies {
            b.actuated = true;
        }
        let mut w = World::new(Model::from_desc(&d));
        let n = w.primary().n();
        let targets: Vec<f64> = (0..n)
            .map(|i| {
                let lim = w.primary().model.limit[i];
                if lim > 0.0 { r.range(-0.5, 0.5) * lim } else { r.range(-1.0, 1.0) }
            })
            .collect();
        // the property is about the control laws, not actuator sizing: random arms may need more than the
        // default limit just to hold themselves against gravity
        w.set_flag(Flag::Umax, 1e4);
        for kind in [Kind::ComputedTorque, Kind::PdGravity, Kind::Passivity] {
            w.set_controller(0, Some(kind));
            assert_eq!(
                w.primary().kind,
                kind,
                "seed {seed}: {kind:?} should apply to a fully actuated chain"
            );
            w.reset();
            for (i, &t) in targets.iter().enumerate() {
                w.set_target(i, t);
            }
            // inertia-shaped gains put every mode at ω_n; light links with heavy joint friction are
            // overdamped and creep in, so allow 20 s
            let secs = 20;
            w.step(secs * 1000);
            let o = w.publish();
            let err = o[off::SLOT + slot::ERR_Q];
            assert!(err < 1e-3, "seed {seed}, {kind:?}: error {err} after {secs} s (n = {n})");
            runs += 1;
        }
    }
    assert!(runs > 0);
}
