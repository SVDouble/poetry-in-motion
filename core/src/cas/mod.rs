//! A small computer-algebra system specialised to planar rigid-body mechanics (see [`expr`]).

pub mod expr;
pub mod latex;

pub use expr::{Angle, Expr, Harmonic, Space};
pub use latex::{Names, Printer};
