//! Layout of the flat `f64` snapshot shared with JavaScript, and the ids of the global flags.
//!
//! The snapshot holds the full state of the primary simulation, then one compact block per slot (primary
//! and shadows) with that slot's controller telemetry. This file is the single source of truth: the browser
//! receives it as JSON (`layout_json`).

use crate::MAXN;
use crate::control::TELEMETRY_LEN;
use crate::mech::{
    DESC_VERSION, HEADER, MAXB, MAXC, MAXE, MAXM, PARAMS_PER_BODY, PER_BODY, PER_CLOSURE, PER_ELEMENT,
};
use crate::plan::{POSTURE, PUSH_K, REF_WN, Tuning};
use crate::sim::{DT, UMAX};

/// Frames in a snapshot: the spatial frame, one per body and the tool.
pub const MAXF: usize = MAXB + 2;
/// Simulation slots: the primary and up to four shadows.
pub const SLOTS: usize = 5;
/// Incompatible snapshot/command ABI changes increment this, independently of document versions.
pub const ABI_VERSION: u32 = 2;

macro_rules! layout {
    ($module:ident, $fields:ident; $($(#[$doc:meta])* $name:ident : $len:expr),* $(,)?) => {
        pub mod $module {
            use super::*;
            layout!(@acc 0usize; $($(#[$doc])* $name : $len),*);
        }
        /// `(name, offset, length)` of every field.
        pub const $fields: &[(&str, usize, usize)] = &[$((stringify!($name), $module::$name, $len)),*];
    };
    (@acc $o:expr; $(#[$doc:meta])* $name:ident : $len:expr $(, $(#[$rd:meta])* $rest:ident : $rl:expr)*) => {
        $(#[$doc])*
        pub const $name: usize = $o;
        layout!(@acc $o + $len; $($(#[$rd])* $rest : $rl),*);
    };
    (@acc $o:expr;) => {
        /// Total length.
        pub const LEN: usize = $o;
    };
}

// Fields on one line are written together by `Sim::publish` (see the adjacency test below).
layout! { off, FIELDS;
    TIME: 1, STEPS: 1, NQ: 1, NU: 1, KIND: 1, PHASE: 1,
    Q: MAXN,
    QD: MAXN,
    QDD: MAXN,
    U: MAXN,
    /// Total generalised force τ = B u + τ_ext.
    TAU: MAXN,
    /// Environment forces: end stops and the push on the tool, Jᵀ F_ext.
    TAU_EXT: MAXN,
    M: MAXN * MAXN,
    C: MAXN * MAXN,
    N: MAXN,
    DV: MAXN,
    MQDD: MAXN,
    CQD: MAXN,
    MDOT: MAXN * MAXN,
    /// [T, V, T + V], q̇ᵀτ, q̇ᵀdiag(β)q̇, max |(Ṁ − 2C) + (Ṁ − 2C)ᵀ|, det M
    ENERGY: 3, POWER: 1, DISSIPATION: 1, SKEW: 1, DET_M: 1,
    /// Energy budget since the last restart: work of [inputs, push + kicks, end stops, friction (loss),
    /// constraint forces], E₀, and the residual E − E₀ − (net work).
    WORK: 5, E0: 1, BUDGET: 1,
    EIG_M: MAXN,
    /// Integration recoveries, loop-step retries, most recent recovery time (s, −1 if none).
    RECOVERY: 3,
    /// (x, y, φ) of the frames s, 1 … nb, t.
    FRAMES: 3 * MAXF,
    COMS: 2 * MAXB,
    TOOL_P: 2,
    JP: 2 * MAXN,
    JS: 3 * MAXN,
    /// Linearisation about q* in tangent coordinates (2r × 2r and 2r × nu), r = DOF.
    LIN_A: 4 * MAXN * MAXN,
    LIN_B: 2 * MAXN * MAXN,
    /// Feed-forward u* holding q*, and the equilibrium residual ‖B u* − Aᵀλ − ∂V/∂q‖.
    U_FF: MAXN,
    EQ_RESIDUAL: 1, DOF: 1, CT_OK: 1,
    /// Tool target, following it (1/0), and whether IK reaches it.
    TARGET: 2, USE_TOOL: 1, REACHABLE: 1,
    Q_TARGET: MAXN,
    Q_REF: MAXN,
    QD_REF: MAXN,
    /// Task reference x_d and the external force on the tool.
    X_REF: 2, FEXT: 2, MANIPULABILITY: 1, UMAX: 1,
    /// Secondary goals: orientation ω and target, elbow ω, target (x, y) and body.
    GOALS: 6,
    /// Loop closures, constraint equations, max |φ(q)|.
    NC: 1, NCON: 1, CONSTRAINT: 1,
    LAMBDA: MAXM,
    AT_LAMBDA: MAXN,
    /// Per body: centre-of-mass velocity and acceleration (x, y pairs), angular velocity and acceleration.
    VCOM: 2 * MAXB,
    ACOM: 2 * MAXB,
    OMEGA: MAXB,
    ALPHA: MAXB,
    /// Force along each spring/damper: k (ℓ − ℓ₀) + c ℓ̇ (positive: pulling its ends together).
    ELEMENT_F: MAXE,
    /// Per body: the centre-of-mass Jacobian ∂p_c/∂q (rows x, y; columns padded to MAXN).
    JCOM: 2 * MAXB * MAXN,
    /// One block per slot (see `slot`).
    SLOT: SLOTS * slot::LEN,
}

layout! { slot, SLOT_FIELDS;
    /// Controller kind (−1 when the slot is off), phase, ‖q − q*‖, ‖x − x_d‖, ‖u‖.
    KIND: 1, PHASE: 1, ERR_Q: 1, ERR_X: 1, EFFORT: 1,
    ENERGY: 3,
    RECOVERY: 3,
    Q: MAXN,
    U: MAXN,
    FRAMES: 3 * MAXF,
    TOOL: 2,
    TELEMETRY: TELEMETRY_LEN,
}

/// Ids accepted by `set_flag` (global settings, applied to every slot).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u32)]
pub enum Flag {
    /// 0 disables the C q̇ term in the simulation (non-physical "what-if").
    Coriolis = 0,
    /// Actuator saturation |uᵢ| ≤ umax.
    Umax = 1,
    TargetX = 2,
    TargetY = 3,
    /// 1: follow the tool target; 0: follow the joint targets q*.
    UseTool = 4,
    /// Natural frequency of the reference filters (rad/s).
    RefWn = 5,
    /// Push on the tool: pointer position, on/off, stiffness (N/m).
    PushX = 6,
    PushY = 7,
    PushOn = 8,
    PushK = 9,
    /// Secondary goals in the task null space.
    Posture = 10,
    Manipulability = 11,
    /// Priority 2: tool orientation (natural frequency, 0 = off) and its target angle.
    Orient = 12,
    OrientTarget = 13,
    /// Priority 3: the elbow (natural frequency, 0 = off), its target point and which body's joint it is.
    Elbow = 14,
    ElbowX = 15,
    ElbowY = 16,
    ElbowBody = 17,
    /// Tuning (see `plan::Tuning`): null-space damping, the damped least squares of the tool level and of
    /// the levels below, the fade band, the reach margin, the IK step and damping, the push limit, the end
    /// stops and the Baumgarte rate.
    NullDamping = 18,
    DlsEps = 19,
    DlsMax = 20,
    DlsLowerEps = 21,
    DlsLowerMax = 22,
    FadeLo = 23,
    FadeHi = 24,
    ReachMargin = 25,
    IkStep = 26,
    IkDamping = 27,
    PushMax = 28,
    StopK = 29,
    StopC = 30,
    StopDepth = 31,
    Baumgarte = 32,
    MotorSpeed = 33,
    Lead = 34,
}

impl Flag {
    pub const ALL: [Self; 35] = [
        Self::Coriolis,
        Self::Umax,
        Self::TargetX,
        Self::TargetY,
        Self::UseTool,
        Self::RefWn,
        Self::PushX,
        Self::PushY,
        Self::PushOn,
        Self::PushK,
        Self::Posture,
        Self::Manipulability,
        Self::Orient,
        Self::OrientTarget,
        Self::Elbow,
        Self::ElbowX,
        Self::ElbowY,
        Self::ElbowBody,
        Self::NullDamping,
        Self::DlsEps,
        Self::DlsMax,
        Self::DlsLowerEps,
        Self::DlsLowerMax,
        Self::FadeLo,
        Self::FadeHi,
        Self::ReachMargin,
        Self::IkStep,
        Self::IkDamping,
        Self::PushMax,
        Self::StopK,
        Self::StopC,
        Self::StopDepth,
        Self::Baumgarte,
        Self::MotorSpeed,
        Self::Lead,
    ];

    /// The value a fresh simulation starts with, for the flags that mirror a user setting (the browser's
    /// defaults come from here); `None` for the momentary ones (targets, the push, the elbow's body).
    #[must_use]
    pub fn default_value(self) -> Option<f64> {
        let t = Tuning::default();
        Some(match self {
            Self::Coriolis => 1.0,
            Self::Umax => UMAX,
            Self::RefWn => REF_WN,
            Self::PushK => PUSH_K,
            Self::Posture => POSTURE,
            Self::Manipulability
            | Self::Orient
            | Self::OrientTarget
            | Self::Elbow
            | Self::ElbowX
            | Self::ElbowY => 0.0,
            Self::NullDamping => t.null_damping,
            Self::DlsEps => t.dls[0],
            Self::DlsMax => t.dls[1],
            Self::DlsLowerEps => t.dls_lower[0],
            Self::DlsLowerMax => t.dls_lower[1],
            Self::FadeLo => t.fade[0],
            Self::FadeHi => t.fade[1],
            Self::ReachMargin => t.reach_margin,
            Self::IkStep => t.ik_step,
            Self::IkDamping => t.ik_damping,
            Self::PushMax => t.push_max,
            Self::StopK => t.stop_k,
            Self::StopC => t.stop_c,
            Self::StopDepth => t.stop_depth,
            Self::Baumgarte => t.baumgarte,
            Self::MotorSpeed => t.motor_speed,
            Self::Lead => t.lead,
            Self::TargetX
            | Self::TargetY
            | Self::UseTool
            | Self::PushX
            | Self::PushY
            | Self::PushOn
            | Self::ElbowBody => {
                return None;
            }
        })
    }

    #[must_use]
    pub fn from_u32(v: u32) -> Option<Self> {
        Self::ALL.into_iter().find(|f| *f as u32 == v)
    }
}

fn fields_json(fields: &[(&str, usize, usize)]) -> String {
    fields.iter().map(|(n, o, l)| format!("\"{n}\":[{o},{l}]")).collect::<Vec<_>>().join(",")
}

/// JSON description of the layout and flags for the JavaScript side.
#[must_use]
pub fn json() -> String {
    let flags: Vec<String> = Flag::ALL.iter().map(|f| format!("\"{f:?}\":{}", *f as u32)).collect();
    let defaults: Vec<String> =
        Flag::ALL.iter().filter_map(|f| f.default_value().map(|v| format!("\"{f:?}\":{v:?}"))).collect();
    // the limits of the mechanism description, which the browser's builder mirrors
    let limits = format!(
        "\"version\":{DESC_VERSION:?},\"header\":{HEADER},\"perBody\":{PER_BODY},\"perClosure\":{PER_CLOSURE},\"perElement\":{PER_ELEMENT},\"paramsPerBody\":{PARAMS_PER_BODY},\"maxBodies\":{MAXB},\"maxDof\":{MAXN},\"maxClosures\":{MAXC},\"maxElements\":{MAXE}"
    );
    format!(
        "{{\"abiVersion\":{ABI_VERSION},\"len\":{},\"maxn\":{MAXN},\"maxf\":{MAXF},\"slots\":{SLOTS},\"slotLen\":{},\"dt\":{DT:?},\"fields\":{{{}}},\"slot\":{{{}}},\"flags\":{{{}}},\"defaults\":{{{}}},\"limits\":{{{limits}}}}}",
        off::LEN,
        slot::LEN,
        fields_json(FIELDS),
        fields_json(SLOT_FIELDS),
        flags.join(","),
        defaults.join(",")
    )
}

#[cfg(test)]
mod tests {
    use super::off::*;
    use super::slot;

    #[test]
    fn grouped_fields_are_adjacent() {
        assert_eq!([STEPS, NQ, NU, KIND, PHASE], [TIME + 1, TIME + 2, TIME + 3, TIME + 4, TIME + 5]);
        assert_eq!([POWER, DISSIPATION, SKEW, DET_M], [ENERGY + 3, ENERGY + 4, ENERGY + 5, ENERGY + 6]);
        assert_eq!([DOF, CT_OK], [EQ_RESIDUAL + 1, EQ_RESIDUAL + 2]);
        assert_eq!([E0, BUDGET], [WORK + 5, WORK + 6]);
        assert_eq!([USE_TOOL, REACHABLE], [TARGET + 2, TARGET + 3]);
        assert_eq!([FEXT, MANIPULABILITY, UMAX], [X_REF + 2, X_REF + 4, X_REF + 5]);
        assert_eq!([NCON, CONSTRAINT], [NC + 1, NC + 2]);
        assert_eq!([slot::PHASE, slot::ERR_Q, slot::ERR_X, slot::EFFORT], [1, 2, 3, 4]);
    }

    #[test]
    fn offsets_are_contiguous() {
        for fields in [super::FIELDS, super::SLOT_FIELDS] {
            let mut expect = 0;
            for (_, o, l) in fields {
                assert_eq!(*o, expect);
                expect += l;
            }
        }
    }
}
