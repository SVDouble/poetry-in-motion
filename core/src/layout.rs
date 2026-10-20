//! Layout of the flat `f64` snapshot shared with JavaScript, and the ids of tunable flags.
//!
//! This file is the single source of truth: the browser receives it as JSON (`layout_json`).

use crate::MAXN;

/// Maximum frames and bodies in a snapshot.
pub const MAXF: usize = 6;
pub const MAXB: usize = 4;

macro_rules! layout {
    ($($(#[$doc:meta])* $name:ident : $len:expr),* $(,)?) => {
        /// Offsets into the snapshot buffer.
        pub mod off {
            use super::*;
            layout!(@acc 0usize; $($(#[$doc])* $name : $len),*);
        }
        /// `(name, offset, length)` of every field.
        pub const FIELDS: &[(&str, usize, usize)] = &[$((stringify!($name), off::$name, $len)),*];
    };
    (@acc $o:expr; $(#[$doc:meta])* $name:ident : $len:expr $(, $(#[$rd:meta])* $rest:ident : $rl:expr)*) => {
        $(#[$doc])*
        pub const $name: usize = $o;
        layout!(@acc $o + $len; $($(#[$rd])* $rest : $rl),*);
    };
    (@acc $o:expr;) => {
        /// Total length of the snapshot.
        pub const LEN: usize = $o;
    };
}

layout! {
    TIME: 1,
    STEPS: 1,
    SYSTEM: 1,
    NQ: 1,
    MODE: 1,
    /// Controller phase (cart-pole: 0 idle, 1 swing-up, 2 LQR catch).
    PHASE: 1,
    Q: MAXN,
    QD: MAXN,
    QDD: MAXN,
    U: MAXN,
    /// Total generalised force τ = B u + τ_ext.
    TAU: MAXN,
    TAU_EXT: MAXN,
    M: MAXN * MAXN,
    C: MAXN * MAXN,
    N: MAXN,
    DV: MAXN,
    MQDD: MAXN,
    CQD: MAXN,
    MDOT: MAXN * MAXN,
    /// [T, V, T + V]
    ENERGY: 3,
    /// q̇ᵀ τ (power delivered by actuators and the environment).
    POWER: 1,
    /// q̇ᵀ diag(β) q̇ (power dissipated by friction).
    DISSIPATION: 1,
    /// max |(Ṁ − 2C) + (Ṁ − 2C)ᵀ| — zero by MLS Lemma 4.2.
    SKEW: 1,
    DET_M: 1,
    /// Eigenvalues of M (2-DOF systems).
    EIG_M: 2,
    FRAMES: 3 * MAXF,
    COMS: 2 * MAXB,
    TOOL_P: 2,
    JP: 2 * MAXN,
    JS: 3 * MAXN,
    /// Linearisation about the upright equilibrium, x = (q, q̇).
    LIN_A: 4 * MAXN * MAXN,
    LIN_B: 2 * MAXN,
    LQR_K: 2 * MAXN,
    E_POLE: 1,
    X_REF: 1,
    TARGET: 2,
    REACHABLE: 1,
    Q_REF: MAXN,
    QD_REF: MAXN,
    Q_GOAL: MAXN,
    UMAX: 1,
    RAIL: 1,
}

/// Ids accepted by `set_flag`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u32)]
pub enum Flag {
    /// 0 disables the C q̇ term in the simulation (non-physical "what-if").
    Coriolis = 0,
    /// Actuator saturation |u_i| ≤ umax.
    Umax = 1,
    /// Cart position set-point.
    XRef = 2,
    TargetX = 3,
    TargetY = 4,
    /// ±1 elbow configuration for inverse kinematics.
    Elbow = 5,
    /// Half-length of the rail.
    Rail = 6,
    /// Natural frequency of the reference filter (rad/s).
    RefWn = 7,
    /// Closed-loop natural frequency of the computed-torque controller (rad/s).
    CtWn = 8,
    LqrQx = 9,
    LqrQth = 10,
    LqrR = 11,
}

impl Flag {
    pub const ALL: [Self; 12] = [
        Self::Coriolis,
        Self::Umax,
        Self::XRef,
        Self::TargetX,
        Self::TargetY,
        Self::Elbow,
        Self::Rail,
        Self::RefWn,
        Self::CtWn,
        Self::LqrQx,
        Self::LqrQth,
        Self::LqrR,
    ];

    #[must_use]
    pub fn from_u32(v: u32) -> Option<Self> {
        Self::ALL.into_iter().find(|f| *f as u32 == v)
    }
}

/// JSON description of the layout and flags for the JavaScript side.
#[must_use]
pub fn json() -> String {
    let fields: Vec<String> = FIELDS.iter().map(|(n, o, l)| format!("\"{n}\":[{o},{l}]")).collect();
    let flags: Vec<String> = Flag::ALL.iter().map(|f| format!("\"{f:?}\":{}", *f as u32)).collect();
    format!(
        "{{\"len\":{},\"maxn\":{MAXN},\"maxf\":{MAXF},\"fields\":{{{}}},\"flags\":{{{}}}}}",
        off::LEN,
        fields.join(","),
        flags.join(",")
    )
}

#[cfg(test)]
mod tests {
    #[test]
    fn offsets_are_contiguous() {
        let mut expect = 0;
        for (_, o, l) in super::FIELDS {
            assert_eq!(*o, expect);
            expect += l;
        }
        assert_eq!(expect, super::off::LEN);
    }
}
