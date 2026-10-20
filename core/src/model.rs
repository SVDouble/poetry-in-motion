//! Runtime descriptor that binds a generated kernel module to the simulator.

use crate::generated::{arm2, cartpole};

type Mcn = fn(&[f64], &[f64], &[f64], &mut [f64], &mut [f64], &mut [f64]);
type QdOut = fn(&[f64], &[f64], &[f64], &mut [f64]);
type QOut = fn(&[f64], &[f64], &mut [f64]);
type Energy = fn(&[f64], &[f64], &[f64]) -> [f64; 2];
type Tool = fn(&[f64], &[f64], &mut [f64], &mut [f64], &mut [f64]);

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u32)]
pub enum SystemId {
    CartPole = 0,
    Arm2 = 1,
}

impl SystemId {
    #[must_use]
    pub const fn from_u32(v: u32) -> Option<Self> {
        match v {
            0 => Some(Self::CartPole),
            1 => Some(Self::Arm2),
            _ => None,
        }
    }

    #[must_use]
    pub const fn model(self) -> &'static Model {
        match self {
            Self::CartPole => &CARTPOLE,
            Self::Arm2 => &ARM2,
        }
    }
}

#[derive(Debug)]
pub struct Model {
    pub id: SystemId,
    pub nq: usize,
    pub nu: usize,
    pub nframes: usize,
    pub nbodies: usize,
    pub has_tool: bool,
    pub params: &'static [&'static str],
    pub defaults: &'static [f64],
    pub damping: &'static [usize],
    /// Actuation matrix, row-major `nq × nu`.
    pub b: &'static [f64],
    pub mcn: Mcn,
    pub mdot: QdOut,
    pub dndq: QOut,
    pub dvdq: QOut,
    pub energy: Energy,
    pub frames: QOut,
    pub coms: QOut,
    pub tool: Tool,
}

impl Model {
    #[must_use]
    pub fn param_index(&self, name: &str) -> Option<usize> {
        self.params.iter().position(|p| *p == name)
    }
}

macro_rules! model {
    ($id:expr, $m:ident, $defaults:expr) => {
        Model {
            id: $id,
            nq: $m::NQ,
            nu: $m::NU,
            nframes: $m::NFRAMES,
            nbodies: $m::NBODIES,
            has_tool: $m::HAS_TOOL,
            params: &$m::PARAMS,
            defaults: &$defaults,
            damping: &$m::DAMPING,
            b: &$m::B,
            mcn: $m::mcn,
            mdot: $m::mdot,
            dndq: $m::dndq,
            dvdq: $m::dvdq,
            energy: $m::energy,
            frames: $m::frames,
            coms: $m::coms,
            tool: $m::tool,
        }
    };
}

const G: f64 = 9.81;

/// Uniform 1 m pole of 0.3 kg on a 1 kg cart: r = L/2, I = m L² / 12.
/// Order: mc, mp, r, Ip, g, βx, βθ.
pub static CARTPOLE: Model = model!(SystemId::CartPole, cartpole, [1.0, 0.3, 0.5, 0.3 / 12.0, G, 0.1, 0.002]);

/// Two uniform 1 m, 1 kg links.
/// Order: m1, m2, l1, l2, r1, r2, I1, I2, g, β1, β2.
pub static ARM2: Model =
    model!(SystemId::Arm2, arm2, [1.0, 1.0, 1.0, 1.0, 0.5, 0.5, 1.0 / 12.0, 1.0 / 12.0, G, 0.05, 0.05]);
