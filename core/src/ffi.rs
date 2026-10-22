//! WebAssembly exports. Scalar-only signatures; bulk data is shared through linear memory:
//! JavaScript writes a mechanism description into `desc_buffer`, calls `build`, then reads the snapshot
//! (`out_ptr`/`out_len`), the derivation (`derivation_*`) and errors (`error_*`) in place.
#![allow(unsafe_code)] // `#[unsafe(no_mangle)]` only — no unsafe blocks

use std::cell::RefCell;

use crate::control::Kind;
use crate::layout::{self, Flag};
use crate::mech::presets;
use crate::model::Model;
use crate::world::{Benchmark, World};

thread_local! {
    static WORLD: RefCell<World> = RefCell::new(World::new(Model::from_desc(&presets::cartpole())));
    static BENCHMARK: RefCell<Option<Benchmark>> = const { RefCell::new(None) };
    static PREPARED: RefCell<Option<Model>> = const { RefCell::new(None) };
    static DESC: RefCell<Vec<f64>> = const { RefCell::new(Vec::new()) };
    static ERROR: RefCell<String> = const { RefCell::new(String::new()) };
    static CONTROLLERS: RefCell<String> = const { RefCell::new(String::new()) };
    static LAYOUT: String = layout::json();
}

fn with<R>(f: impl FnOnce(&mut World) -> R) -> R {
    WORLD.with_borrow_mut(f)
}

fn mutation(accepted: bool) -> i32 {
    ERROR.with_borrow_mut(|e| {
        if accepted {
            e.clear();
        } else {
            *e = "invalid simulation input: check index, finite values, and physical domain".into();
        }
    });
    if accepted { 0 } else { -1 }
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
string_export!(derivation_ptr, derivation_len, |f: fn(&str) -> _| with(|w| f(&w.primary().model.derivation)));
string_export!(layout_ptr, layout_len, |f: fn(&str) -> _| LAYOUT.with(|l| f(l)));
string_export!(controllers_ptr, controllers_len, |f: fn(&str) -> _| CONTROLLERS.with_borrow(|c| f(c)));

exports! {
    /// A buffer of `len` f64 for the mechanism description; write into it, then call `build(len)`.
    fn desc_buffer(len: u32) -> *mut f64 {
        DESC.with_borrow_mut(|d| {
            d.clear();
            d.resize(len as usize, 0.0);
            d.as_mut_ptr()
        })
    }

    /// Derive and compile the described mechanism and restart every slot on it. Returns 0 on success;
    /// otherwise the previous mechanism is kept and `error_ptr`/`error_len` explain why.
    fn build(len: u32) -> i32 {
        if prepare(len) == 0 { commit_build() } else { -1 }
    }

    /// Derive a candidate without changing the running world. Commit only an accepted document revision.
    fn prepare(len: u32) -> i32 {
        let result = DESC.with_borrow(|d| {
            if len as usize != d.len() { return Err("description length does not match buffer".into()); }
            Model::build(d)
        });
        match result {
            Ok(model) => {
                PREPARED.with_borrow_mut(|p| *p = Some(model));
                ERROR.with_borrow_mut(String::clear);
                0
            }
            Err(error) => {
                PREPARED.with_borrow_mut(|p| *p = None);
                ERROR.with_borrow_mut(|e| *e = error); -1
            }
        }
    }

    fn discard_build() { PREPARED.with_borrow_mut(|p| *p = None); }

    fn commit_build() -> i32 {
        let Some(model) = PREPARED.with_borrow_mut(Option::take) else { return mutation(false) };
        with(|w| { w.rebuild(model); w.publish(); });
        mutation(true)
    }

    /// Refresh the controllers JSON (specs, applicability on this mechanism, parameters); returns its length.
    fn controllers() -> u32 {
        let json = with(World::controllers_json);
        CONTROLLERS.with_borrow_mut(|c| *c = json);
        CONTROLLERS.with_borrow(|c| c.len() as u32)
    }

    fn reset() { with(World::reset) }

    /// Advance every slot `n` fixed steps and refresh the snapshot.
    fn step(n: u32) {
        with(|w| {
            w.step(n);
            w.publish();
        });
    }

    fn publish() {
        with(|w| {
            w.publish();
        });
    }

    /// Prepare an isolated copy and controller design before starting an integration timer.
    fn bench_prepare() { BENCHMARK.with_borrow_mut(|b| *b = Some(with(|w| w.benchmark()))) }
    /// Advance the prepared benchmark, leaving the live simulation untouched.
    fn bench(n: u32) { BENCHMARK.with_borrow_mut(|b| { if let Some(b) = b { b.run(n); } }); }

    fn set_dt(dt: f64) -> i32 { mutation(with(|w| w.set_dt(dt))) }
    fn param_value(i: u32) -> f64 { with(|w| w.primary().p.get(i as usize).copied().unwrap_or(f64::NAN)) }
    fn param_count() -> u32 { with(|w| w.primary().p.len() as u32) }
    fn set_param(i: u32, v: f64) -> i32 { mutation(with(|w| w.set_param(i as usize, v))) }
    /// Run controller `kind` in `slot` (0 primary, 1… shadows); an unknown kind turns a shadow off.
    fn set_controller(slot: u32, kind: u32) { with(|w| w.set_controller(slot as usize, Kind::from_u32(kind))) }
    /// Parameter `i` of controller `kind` (shared by all slots running it).
    fn set_ctrl_param(kind: u32, i: u32, v: f64) -> i32 {
        mutation(Kind::from_u32(kind).is_some_and(|k| with(|w| w.set_ctrl_param(k, i as usize, v))))
    }
    /// Restart the shadows from the primary's state.
    fn sync_shadows() { with(World::sync_shadows) }
    /// Joint-space target q*ᵢ.
    fn set_target(i: u32, v: f64) -> i32 { mutation(with(|w| w.set_target(i as usize, v))) }
    fn set_state(i: u32, q: f64, qd: f64) -> i32 { mutation(with(|w| w.set_state(i as usize, q, qd))) }
    /// Impulsive disturbance: add `dqd` to q̇ᵢ (in every slot).
    fn kick(i: u32, dqd: f64) -> i32 { mutation(with(|w| w.kick(i as usize, dqd))) }
    /// Manual actuator input uᵢ (added to the controller output).
    fn set_input(i: u32, v: f64) -> i32 { mutation(with(|w| w.set_input(i as usize, v))) }

    fn set_flag(id: u32, v: f64) -> i32 {
        mutation(Flag::from_u32(id).is_some_and(|f| with(|w| w.set_flag(f, v))))
    }

    fn out_ptr() -> *const f64 { with(|w| w.out().as_ptr()) }
    fn out_len() -> u32 { layout::off::LEN as u32 }
}
