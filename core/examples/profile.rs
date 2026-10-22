//! Separate costs before changing ownership: cargo run --release --example profile.
use motion_core::{control::Kind, mech::presets, model::Model, world::World};
use std::{hint::black_box, time::Instant};

fn us(mut f: impl FnMut(), n: u32) -> f64 {
    let start = Instant::now();
    for _ in 0..n {
        f();
    }
    start.elapsed().as_secs_f64() * 1e6 / f64::from(n)
}
fn main() {
    println!(
        "case | derive ms | model clone us | fork/design us | redesign us | step us | publish us | 5-slot tick us"
    );
    for (name, desc) in
        [("arm2", presets::arm2()), ("arm7", presets::arm7()), ("fourbar", presets::four_bar())]
    {
        let start = Instant::now();
        let model = Model::from_desc(&desc);
        let derive = start.elapsed().as_secs_f64() * 1e3;
        let clone = us(
            || {
                black_box(model.clone());
            },
            100,
        );
        let mut world = World::new(model);
        world.set_controller(0, Some(Kind::Impedance));
        let fork = us(
            || {
                black_box(world.primary().fork(Kind::Impedance));
            },
            20,
        );
        let redesign = us(
            || {
                world.set_target(0, 0.2);
                world.primary_mut().refresh_plan();
            },
            20,
        );
        world.step(10);
        let step = us(|| world.step(1), 5000);
        let publish = us(
            || {
                black_box(world.publish());
            },
            500,
        );
        for i in 1..5 {
            world.set_controller(i, Some(Kind::Impedance));
        }
        let tick = us(
            || {
                world.step(4);
                black_box(world.publish());
            },
            1000,
        );
        println!(
            "{name} | {derive:.2} | {clone:.2} | {fork:.2} | {redesign:.2} | {step:.2} | {publish:.2} | {tick:.2}"
        );
    }
}
