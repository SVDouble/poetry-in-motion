//! WebAssembly exports. Scalar-only signatures; bulk data is shared through linear memory:
//! JavaScript writes a mechanism description into `desc_buffer`, calls `build`, then reads the snapshot
//! (`out_ptr`/`out_len`), the derivation (`derivation_*`) and errors (`error_*`) in place.
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

/// Declare `extern "C"` exports without repeating the attribute and ABI on every one-liner.
macro_rules! exports {
    ($($(#[$doc:meta])* fn $name:ident($($arg:ident: $ty:ty),*) $(-> $ret:ty)? $body:block)*) => {
        $(
            $(#[$doc])*
            #[unsafe(no_mangle)]
            pub extern "C" fn $name($($arg: $ty),*) $(-> $ret)? $body
        )*
    };
}

/// A string as a `(pointer, length)` export pair.
macro_rules! string_export {
    ($ptr:ident, $len:ident, $get:expr) => {
        exports! {
            fn $ptr() -> *const u8 { $get(|s: &str| s.as_ptr()) }
            fn $len() -> u32 { $get(|s: &str| s.len() as u32) }
        }
    };
}

string_export!(error_ptr, error_len, |f: fn(&str) -> _| ERROR.with_borrow(|e| f(e)));
string_export!(derivation_ptr, derivation_len, |f: fn(&str) -> _| with(|s| f(&s.model.derivation)));
string_export!(layout_ptr, layout_len, |f: fn(&str) -> _| LAYOUT.with(|l| f(l)));

exports! {
    /// A buffer of `len` f64 for the mechanism description; write into it, then call `build(len)`.
    fn desc_buffer(len: u32) -> *mut f64 {
        DESC.with_borrow_mut(|d| {
            d.clear();
            d.resize(len as usize, 0.0);
            d.as_mut_ptr()
        })
    }

    /// Derive and compile the described mechanism and start a fresh simulation of it. Returns 0 on
    /// success; otherwise the previous mechanism is kept and `error_ptr`/`error_len` explain why.
    fn build(len: u32) -> i32 {
        match DESC.with_borrow(|d| Model::build(&d[..(len as usize).min(d.len())])) {
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

    fn reset() { with(Sim::reset) }

    /// Advance `n` fixed steps and refresh the snapshot.
    fn step(n: u32) {
        with(|s| {
            s.step_n(n);
            s.publish();
        });
    }

    fn publish() {
        with(|s| {
            s.publish();
        });
    }

    /// Run `n` steps on a throw-away copy of the current simulation (for timing; state is untouched).
    fn bench(n: u32) {
        let mut s = with(|s| s.clone());
        s.step_n(n);
        std::hint::black_box(&s.q);
    }

    fn set_dt(dt: f64) { with(|s| s.dt = dt.clamp(1e-5, 1e-2)) }
    fn param_count() -> u32 { with(|s| s.p.len() as u32) }
    fn set_param(i: u32, v: f64) { with(|s| s.set_param(i as usize, v)) }
    fn set_mode(mode: u32) { with(|s| s.set_mode(mode)) }
    /// Joint-space target q*ᵢ (set-point, LQR equilibrium).
    fn set_target(i: u32, v: f64) { with(|s| s.set_target(i as usize, v)) }
    fn set_state(i: u32, q: f64, qd: f64) { with(|s| s.set_state(i as usize, q, qd)) }
    /// Impulsive disturbance: add `dqd` to q̇ᵢ.
    fn kick(i: u32, dqd: f64) { with(|s| s.kick(i as usize, dqd)) }

    /// Manual actuator input uᵢ (added to the controller output).
    fn set_input(i: u32, v: f64) {
        with(|s| {
            if let Some(u) = s.u_manual.get_mut(i as usize) {
                *u = v;
            }
        });
    }

    fn set_flag(id: u32, v: f64) {
        if let Some(f) = Flag::from_u32(id) {
            with(|s| s.set_flag(f, v));
        }
    }

    fn out_ptr() -> *const f64 { with(|s| s.out_ptr()) }
    fn out_len() -> u32 { layout::off::LEN as u32 }
}
