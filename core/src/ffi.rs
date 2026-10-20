//! WebAssembly exports. Scalar-only signatures; bulk data is shared through linear memory:
//! JavaScript wraps `out_ptr()`/`out_len()` in a `Float64Array` view and reads it without copies.
#![allow(unsafe_code)] // `#[unsafe(no_mangle)]` only — no unsafe blocks

use std::cell::RefCell;

use crate::layout::{self, Flag};
use crate::model::SystemId;
use crate::sim::Sim;

thread_local! {
    static SIM: RefCell<Sim> = RefCell::new(Sim::new(SystemId::CartPole));
    static LAYOUT: String = layout::json();
}

fn with<R>(f: impl FnOnce(&mut Sim) -> R) -> R {
    SIM.with_borrow_mut(f)
}

/// Switch to a system (0 = cart-pole, 1 = two-link arm). Returns 0 on success.
#[unsafe(no_mangle)]
pub extern "C" fn init(system: u32) -> i32 {
    SystemId::from_u32(system).map_or(-1, |id| {
        with(|s| *s = Sim::new(id));
        0
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn reset() {
    with(Sim::reset);
}

/// Advance `n` fixed steps and refresh the snapshot.
#[unsafe(no_mangle)]
pub extern "C" fn step(n: u32) {
    with(|s| {
        s.step_n(n);
        s.publish();
    });
}

#[unsafe(no_mangle)]
pub extern "C" fn publish() {
    with(|s| {
        s.publish();
    });
}

/// Run `n` steps on a throw-away copy of the current simulation (for timing; state is untouched).
#[unsafe(no_mangle)]
pub extern "C" fn bench(n: u32) {
    let mut s = with(|s| s.clone());
    s.step_n(n);
    std::hint::black_box(&s.q);
}

#[unsafe(no_mangle)]
pub extern "C" fn set_dt(dt: f64) {
    with(|s| s.dt = dt.clamp(1e-5, 1e-2));
}

#[unsafe(no_mangle)]
pub extern "C" fn param_count() -> u32 {
    with(|s| s.model.params.len() as u32)
}

#[unsafe(no_mangle)]
pub extern "C" fn param(i: u32) -> f64 {
    with(|s| s.p.get(i as usize).copied().unwrap_or(f64::NAN))
}

#[unsafe(no_mangle)]
pub extern "C" fn set_param(i: u32, v: f64) {
    with(|s| s.set_param(i as usize, v));
}

#[unsafe(no_mangle)]
pub extern "C" fn set_mode(mode: u32) {
    with(|s| s.set_mode(mode));
}

/// Manual actuator input u_i (added to the controller output).
#[unsafe(no_mangle)]
pub extern "C" fn set_input(i: u32, v: f64) {
    with(|s| {
        if let Some(u) = s.u_manual.get_mut(i as usize) {
            *u = v;
        }
    });
}

#[unsafe(no_mangle)]
pub extern "C" fn set_flag(id: u32, v: f64) {
    if let Some(f) = Flag::from_u32(id) {
        with(|s| s.set_flag(f, v));
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn set_state(i: u32, q: f64, qd: f64) {
    with(|s| s.set_state(i as usize, q, qd));
}

/// Impulsive disturbance: add `dqd` to q̇_i.
#[unsafe(no_mangle)]
pub extern "C" fn kick(i: u32, dqd: f64) {
    with(|s| {
        if (i as usize) < s.model.nq {
            s.qd[i as usize] += dqd;
        }
    });
}

#[unsafe(no_mangle)]
pub extern "C" fn out_ptr() -> *const f64 {
    with(|s| s.out_ptr())
}

#[unsafe(no_mangle)]
pub const extern "C" fn out_len() -> u32 {
    layout::off::LEN as u32
}

#[unsafe(no_mangle)]
pub extern "C" fn layout_ptr() -> *const u8 {
    LAYOUT.with(|l| l.as_ptr())
}

#[unsafe(no_mangle)]
pub extern "C" fn layout_len() -> u32 {
    LAYOUT.with(|l| l.len() as u32)
}
