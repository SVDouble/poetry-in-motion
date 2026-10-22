//! Real-time dynamics core for the Poetry in Motion robot visualizer.
//!
//! Every system obeys the manipulator equation (Murray, Li & Sastry, eq. 4.24)
//!
//! ```text
//! M(q) q̈ + C(q, q̇) q̇ + N(q, q̇) = τ,      τ = B u + τ_ext
//! ```
//!
//! A mechanism is described generically (a tree of revolute/prismatic joints, see [`mech`]); its equations
//! are derived symbolically in the browser by a small computer-algebra system specialised to planar
//! mechanics ([`cas`]), printed as LaTeX ([`doc`]) and compiled into numeric tapes ([`tape`]) that the
//! simulator ([`sim`]) and the generic controllers ([`control`]) evaluate at 1 kHz.

pub mod cas;
pub mod control;
pub mod doc;
pub mod dynamics;
mod ffi;
pub mod layout;
pub mod lqr;
pub mod mat;
pub mod mech;
pub mod model;
pub mod plan;
pub mod qp;
pub mod sim;
pub mod tape;
pub mod world;

/// Maximum number of joints (generalised coordinates) of a mechanism.
pub const MAXN: usize = 7;

pub mod history;
