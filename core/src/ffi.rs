//! WebAssembly exports. Scalar-only signatures; bulk data is shared through linear memory:
//! JavaScript writes a mechanism description into `desc_buffer`, calls `build`, then reads the snapshot
//! (`out_ptr`/`out_len`) and the derivation (`derivation_ptr`/`derivation_len`) in place.
#![allow(unsafe_code)] // `#[unsafe(no_mangle)]` only — no unsafe blocks

use std::cell::RefCell;

use crate::layout::{self, Flag};
use crate::mech::presets;
use crate::model::Model;
use crate::sim::Sim;

thread_local! {
    static SIM: RefCell<Sim> = RefCell::new(Sim::new(Model::from_desc(&presets::cartpole())));
    static DESC: RefCell<Vec<f64>> = const { RefCell::new(Vec::new()) };
    static ERROR: RefCell<String> = const { RefCell::new(String::new()) };
    static LAYOUT: String = layout::json();
}

fn with<R>(f: impl FnOnce(&mut Sim) -> R) -> R {
    SIM.with_borrow_mut(f)
}

/// A buffer of `len` f64 for the mechanism description; write into it, then call `build(len)`.
#[unsafe(no_mangle)]
pub extern "C" fn desc_buffer(len: u32) -> *mut f64 {
    DESC.with_borrow_mut(|d| {
        d.clear();
        d.resize(len as usize, 0.0);
        d.as_mut_ptr()
    })
}

/// Derive and compile the described mechanism and start a fresh simulation of it. Returns 0 on
/// success; otherwise the previous mechanism is kept and `error_ptr`/`error_len` explain why.
#[unsafe(no_mangle)]
pub extern "C" fn build(len: u32) -> i32 {
    let res = DESC.with_borrow(|d| Model::build(&d[..(len as usize).min(d.len())]));
    match res {
        Ok(model) => {
            with(|s| {
                let (mode, umax, coriolis) = (s.mode, s.umax, s.coriolis);
                *s = Sim::new(model);
                (s.mode, s.umax, s.coriolis) = (mode, umax, coriolis);
                s.publish();
            });
            ERROR.with_borrow_mut(String::clear);
            0
        }
        Err(e) => {
            ERROR.with_borrow_mut(|s| *s = e);
            -1
        }
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn error_ptr() -> *const u8 {
    ERROR.with_borrow(|s| s.as_ptr())
}

#[unsafe(no_mangle)]
pub extern "C" fn error_len() -> u32 {
    ERROR.with_borrow(|s| s.len() as u32)
}

/// JSON with the LaTeX of every step of the derivation of the current mechanism.
#[unsafe(no_mangle)]
pub extern "C" fn derivation_ptr() -> *const u8 {
    with(|s| s.model.derivation.as_ptr())
}

#[unsafe(no_mangle)]
pub extern "C" fn derivation_len() -> u32 {
    with(|s| s.model.derivation.len() as u32)
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
    with(|s| s.p.len() as u32)
}

#[unsafe(no_mangle)]
pub extern "C" fn set_param(i: u32, v: f64) {
    with(|s| s.set_param(i as usize, v));
}

#[unsafe(no_mangle)]
pub extern "C" fn set_mode(mode: u32) {
    with(|s| s.set_mode(mode));
}

/// Joint-space target q*ᵢ (set-point, LQR equilibrium).
#[unsafe(no_mangle)]
pub extern "C" fn set_target(i: u32, v: f64) {
    with(|s| s.set_target(i as usize, v));
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
    with(|s| s.kick(i as usize, dqd));
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
