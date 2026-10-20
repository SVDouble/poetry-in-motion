//! Real-time dynamics core for the meowfer robot visualizer.
//!
//! Every system obeys the manipulator equation (Murray, Li & Sastry, eq. 4.24)
//!
//! ```text
//! M(q) q̈ + C(q, q̇) q̇ + N(q, q̇) = τ,      τ = B u + τ_ext
//! ```
//!
//! The kernels for M, C, N, energies, frames and Jacobians are generated symbolically by `sym/`
//! (see `generated/`). This crate integrates them, runs the controllers and publishes a flat `f64`
//! snapshot that the browser reads without copying.

pub mod control;
pub mod dynamics;
mod ffi;
#[rustfmt::skip]
pub mod generated;
pub mod layout;
pub mod lqr;
pub mod mat;
pub mod model;
pub mod sim;

/// Maximum number of generalised coordinates supported by the fixed-size buffers.
pub const MAXN: usize = 4;
/// Maximum number of model parameters.
pub const MAXP: usize = 16;
