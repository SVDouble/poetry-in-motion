//! Layout of the flat `f64` snapshot shared with JavaScript, and the ids of tunable flags.
//!
//! This file is the single source of truth: the browser receives it as JSON (`layout_json`).

use crate::MAXN;

/// Frames in a snapshot: the spatial frame, one per body and the tool.
pub const MAXF: usize = crate::mech::MAXB + 2;

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
    NQ: 1,
    NU: 1,
    MODE: 1,
    /// Controller phase (0 idle, 1 swing-up, 2 LQR catch / balance).
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
    /// Eigenvalues of M, ascending.
    EIG_M: MAXN,
    /// (x, y, φ) of the frames s, 1 … n, t.
    FRAMES: 3 * MAXF,
    COMS: 2 * crate::mech::MAXB,
    TOOL_P: 2,
    JP: 2 * MAXN,
    JS: 3 * MAXN,
    /// LQR linearisation about q*: A (2n × 2n), B (2n × nu), gain K (nu × 2n), feed-forward u*.
    LIN_A: 4 * MAXN * MAXN,
    LIN_B: 2 * MAXN * MAXN,
    LQR_K: 2 * MAXN * MAXN,
    U_FF: MAXN,
    LQR_OK: 1,
    /// Dimension r of the tangent coordinates ξ the LQR works in (n for open chains, n − rank A with loops).
    LQR_DIM: 1,
    /// max |∂V/∂q_u(q*)| over unactuated joints (0 ⇔ q* is an equilibrium).
    EQ_RESIDUAL: 1,
    /// Energy of the unactuated motion relative to the target, and its scale.
    E_U: 1,
    E_REF: 1,
    TARGET: 2,
    IK: 1,
    REACHABLE: 1,
    Q_TARGET: MAXN,
    Q_REF: MAXN,
    QD_REF: MAXN,
    KP: MAXN,
    KD: MAXN,
    UMAX: 1,
    CT_WN: 1,
    /// 1 when computed torque can realise any acceleration on the constraint manifold.
    CT_OK: 1,
    /// Loop closures: count, max |φ(q)| and the multipliers λ; Aᵀλ is the constraint generalised force.
    NC: 1,
    NCON: 1,
    CONSTRAINT: 1,
    LAMBDA: crate::mech::MAXM,
    AT_LAMBDA: MAXN,
}

/// Ids accepted by `set_flag`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u32)]
pub enum Flag {
    /// 0 disables the C q̇ term in the simulation (non-physical "what-if").
    Coriolis = 0,
    /// Actuator saturation |u_i| ≤ umax.
    Umax = 1,
    TargetX = 2,
    TargetY = 3,
    /// 1: track the tool target by inverse kinematics; 0: track the joint targets.
    Ik = 4,
    /// Natural frequency of the reference filter (rad/s).
    RefWn = 5,
    /// Closed-loop natural frequency of computed torque (rad/s).
    CtWn = 6,
    /// Natural frequency used to scale the PD gains (rad/s).
    PdWn = 7,
    LqrQx = 8,
    LqrQv = 9,
    LqrR = 10,
    /// Energy-pumping gain of the swing-up.
    SwingGain = 11,
}

impl Flag {
    pub const ALL: [Self; 12] = [
        Self::Coriolis,
        Self::Umax,
        Self::TargetX,
        Self::TargetY,
        Self::Ik,
        Self::RefWn,
        Self::CtWn,
        Self::PdWn,
        Self::LqrQx,
        Self::LqrQv,
        Self::LqrR,
        Self::SwingGain,
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
    fn grouped_fields_are_adjacent() {
        use super::off::*;
        // `Sim::publish` writes these groups with a single copy each
        assert_eq!([STEPS, NQ, NU, MODE, PHASE], [TIME + 1, TIME + 2, TIME + 3, TIME + 4, TIME + 5]);
        assert_eq!([POWER, DISSIPATION, SKEW, DET_M], [ENERGY + 3, ENERGY + 4, ENERGY + 5, ENERGY + 6]);
        assert_eq!([E_U, E_REF], [EQ_RESIDUAL + 1, EQ_RESIDUAL + 2]);
        assert_eq!([IK, REACHABLE], [TARGET + 2, TARGET + 3]);
        assert_eq!([CT_WN, CT_OK], [UMAX + 1, UMAX + 2]);
        assert_eq!([NCON, CONSTRAINT], [NC + 1, NC + 2]);
        assert_eq!(LQR_DIM, LQR_OK + 1);
    }

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
