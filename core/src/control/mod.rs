//! Controllers as plug-ins.
//!
//! Each controller is a small struct implementing [`Controller`]; its tunable
//! parameters and the live values it reports (telemetry) are declared in [`SPECS`], which the browser reads
//! to render parameters and explanation pages generically. Everything mechanism-specific a controller needs
//! (targets, references, equilibrium, linearisation, the external push, secondary goals) lives in the shared
//! [`Plan`]; the model provides M, C, N, the constraints and the tool kinematics.

mod joint;
mod optimal;
mod swing;
mod task;

use std::fmt::Write;

use crate::dynamics::{Constraints, Terms, VecN};
use crate::model::Model;
use crate::plan::{Plan, Tool};

/// Everything a controller may read during one control step.
#[derive(Debug)]
pub struct Ctx<'a> {
    pub q: &'a [f64],
    pub qd: &'a [f64],
    pub terms: &'a Terms,
    /// Loop-closure constraints (`None` for open chains).
    pub k: Option<&'a Constraints>,
    /// Tool kinematics (`None` without a tool point).
    pub tool: Option<&'a Tool>,
    /// This controller's parameters, in the order of its [`Spec::params`].
    pub params: &'a [f64],
    pub dt: f64,
    /// Actuator saturation |uᵢ| ≤ umax (MPC plans with it).
    pub umax: f64,
}

pub trait Controller: std::fmt::Debug {
    /// Recompute gains after parameters, targets or the model changed.
    fn design(&mut self, _plan: &Plan, _model: &mut Model, _params: &[f64], _dt: f64) {}
    /// Forget internal state (after a reset or a jump of the state).
    fn reset(&mut self) {}
    /// Actuator inputs u for this step.
    fn update(&mut self, plan: &Plan, model: &mut Model, c: &Ctx<'_>) -> VecN;
    /// Live values shown on the controller's page (see [`Spec::telemetry`]).
    fn telemetry(&self, _t: &mut Telemetry<'_>) {}
    /// Does it regulate around the equilibrium q* (designed there, like LQR and MPC) rather than track the
    /// reference? Then following a tool target moves q* itself, to the inverse kinematics.
    fn regulates(&self) -> bool {
        false
    }
    /// Controller phase (swing-up: 1 pumping, 2 catching).
    fn phase(&self) -> u32 {
        0
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u32)]
pub enum Kind {
    Passive = 0,
    Pd = 1,
    PdGravity = 2,
    ComputedTorque = 3,
    Passivity = 4,
    Vmc = 5,
    Impedance = 6,
    Admittance = 7,
    Lqr = 8,
    Mpc = 9,
    SwingUp = 10,
}

impl Kind {
    pub const ALL: [Self; 11] = [
        Self::Passive,
        Self::Pd,
        Self::PdGravity,
        Self::ComputedTorque,
        Self::Passivity,
        Self::Vmc,
        Self::Impedance,
        Self::Admittance,
        Self::Lqr,
        Self::Mpc,
        Self::SwingUp,
    ];

    #[must_use]
    pub fn from_u32(v: u32) -> Option<Self> {
        Self::ALL.into_iter().find(|k| *k as u32 == v)
    }

    #[must_use]
    pub fn spec(self) -> &'static Spec {
        &SPECS[self as usize]
    }

    #[must_use]
    pub fn defaults(self) -> Vec<f64> {
        self.spec().params.iter().map(|p| p.default).collect()
    }

    /// A fresh controller of this kind.
    #[must_use]
    pub fn create(self) -> Box<dyn Controller> {
        match self {
            Self::Passive => Box::new(joint::Passive),
            Self::Pd => Box::new(joint::Pd::new(false)),
            Self::PdGravity => Box::new(joint::Pd::new(true)),
            Self::ComputedTorque => Box::new(joint::ComputedTorque::default()),
            Self::Passivity => Box::new(joint::Passivity::default()),
            Self::Vmc => Box::new(task::Vmc::default()),
            Self::Impedance => Box::new(task::Impedance::default()),
            Self::Admittance => Box::new(task::Admittance::default()),
            Self::Lqr => Box::new(optimal::Lqr::default()),
            Self::Mpc => Box::new(optimal::Mpc::default()),
            Self::SwingUp => Box::new(swing::SwingUp::default()),
        }
    }

    /// Can this controller run on the built mechanism? `Err` explains why not.
    ///
    /// # Errors
    /// A short human-readable reason.
    pub fn applicable(self, model: &Model, plan: &Plan) -> Result<(), &'static str> {
        let actuated = || if model.nu > 0 { Ok(()) } else { Err("no joint is actuated") };
        let full =
            || if plan.ct_ok { Ok(()) } else { Err("needs an independent actuator per degree of freedom") };
        let tool = || if model.has_tool() { Ok(()) } else { Err("needs a tool point") };
        match self {
            Self::Passive => Ok(()),
            Self::Pd | Self::PdGravity => actuated(),
            Self::Lqr | Self::Mpc => actuated().and(if plan.stabilisable {
                Ok(())
            } else {
                Err("cannot hold q*: an unstable mode there is out of reach of every input")
            }),
            Self::ComputedTorque | Self::Passivity => full(),
            // shaping the tool's potential through Jᵀ needs an actuator per degree of freedom
            Self::Vmc | Self::Impedance | Self::Admittance => tool().and_then(|()| full()),
            Self::SwingUp => actuated().and(if plan.ct_ok {
                Err("for underactuated systems (this one is fully actuated)")
            } else {
                Ok(())
            }),
        }
    }
}

/// A tunable parameter. `redesign`: changing it requires [`Controller::design`] (gains, QP matrices).
#[derive(Debug)]
pub struct Param {
    pub key: &'static str,
    pub tex: &'static str,
    pub default: f64,
    pub min: f64,
    pub max: f64,
    pub log: bool,
    pub unit: &'static str,
    pub redesign: bool,
}

const fn param(key: &'static str, tex: &'static str, default: f64, min: f64, max: f64) -> Param {
    Param { key, tex, default, min, max, log: false, unit: "", redesign: false }
}

impl Param {
    const fn log(mut self) -> Self {
        self.log = true;
        self
    }
    const fn unit(mut self, unit: &'static str) -> Self {
        self.unit = unit;
        self
    }
    const fn redesign(mut self) -> Self {
        self.redesign = true;
        self
    }
}

#[derive(Debug)]
pub struct Spec {
    pub key: &'static str,
    pub name: &'static str,
    pub family: &'static str,
    pub params: &'static [Param],
    /// `(name, length)` of each telemetry field, laid out consecutively.
    pub telemetry: &'static [(&'static str, usize)],
}

impl Spec {
    /// `(offset, length)` of telemetry field `name` within the slot's telemetry block.
    #[must_use]
    pub fn telemetry_field(&self, name: &str) -> Option<(usize, usize)> {
        let mut off = 0;
        for &(key, len) in self.telemetry {
            if key == name {
                return Some((off, len));
            }
            off += len;
        }
        None
    }
}

const N: usize = crate::MAXN;

pub static SPECS: [Spec; 11] = [
    Spec { key: "passive", name: "Passive", family: "none", params: &[], telemetry: &[] },
    Spec {
        key: "pd",
        name: "PD",
        family: "joint",
        params: &[
            param("wn", r"\omega_{n}", 6.0, 0.5, 30.0).log().unit("rad/s"),
            param("zeta", r"\zeta", 1.0, 0.1, 2.0),
        ],
        telemetry: &[("kp", N * N), ("kd", N * N)],
    },
    Spec {
        key: "pd_gravity",
        name: "PD + gravity",
        family: "joint",
        params: &[
            param("wn", r"\omega_{n}", 6.0, 0.5, 30.0).log().unit("rad/s"),
            param("zeta", r"\zeta", 1.0, 0.1, 2.0),
        ],
        telemetry: &[("kp", N * N), ("kd", N * N), ("ug", N)],
    },
    Spec {
        key: "computed_torque",
        name: "Computed torque",
        family: "joint",
        params: &[
            param("wn", r"\omega_{c}", 12.0, 1.0, 40.0).log().unit("rad/s"),
            param("zeta", r"\zeta", 1.0, 0.1, 2.0),
        ],
        telemetry: &[("qdd", N)],
    },
    Spec {
        key: "passivity",
        name: "Passivity-based",
        family: "energy",
        params: &[
            param("lambda", r"\lambda", 8.0, 0.5, 40.0).log().unit("1/s"),
            param("kd", r"k_{D}", 10.0, 0.5, 60.0).log().unit("1/s"),
        ],
        telemetry: &[("s", N), ("lyap", 1), ("lyap_dot", 1)],
    },
    Spec {
        key: "vmc",
        name: "Virtual model (energy shaping)",
        family: "energy",
        params: &[
            param("k", "K", 300.0, 10.0, 3000.0).log().unit("N/m"),
            param("zeta", r"\zeta", 0.7, 0.05, 2.0),
            param("null", r"k_{\perp}", 1.0, 0.0, 1.0),
        ],
        telemetry: &[
            ("x", 2),
            ("xd", 2),
            ("force", 2),
            ("tau_task", N),
            ("tau_null", N),
            ("projector", N * N),
            ("leak", 1),
            ("w", 1),
            ("lyap", 1),
            ("lyap_dot", 1),
            ("orient_err", 1),
            ("elbow_err", 2),
            ("leak_sec", 1),
        ],
    },
    Spec {
        key: "impedance",
        name: "Impedance",
        family: "interaction",
        params: &[
            param("mass", r"\Lambda_{d}", 1.0, 0.05, 10.0).log().unit("kg"),
            param("k", "K_{d}", 300.0, 1.0, 3000.0).log().unit("N/m"),
            param("zeta", r"\zeta", 0.7, 0.05, 2.0),
            param("null", r"k_{\perp}", 1.0, 0.0, 1.0),
        ],
        telemetry: &[
            ("x", 2),
            ("xd", 2),
            ("e", 2),
            ("fext", 2),
            ("xdd_cmd", 2),
            ("tau_null", N),
            ("leak", 1),
            ("w", 1),
            ("orient_err", 1),
            ("elbow_err", 2),
            ("leak_sec", 1),
        ],
    },
    Spec {
        key: "admittance",
        name: "Admittance",
        family: "interaction",
        params: &[
            param("mass", r"\Lambda_{d}", 2.0, 0.05, 10.0).log().unit("kg"),
            param("k", "K_{d}", 100.0, 0.0, 3000.0).unit("N/m"),
            param("zeta", r"\zeta", 0.7, 0.05, 2.0),
            param("wn_in", r"\omega_{\text{in}}", 20.0, 2.0, 60.0).log().unit("rad/s"),
        ],
        telemetry: &[("xv", 2), ("xv_dot", 2), ("fext", 2), ("x", 2)],
    },
    Spec {
        key: "lqr",
        name: "LQR",
        family: "optimal",
        params: &[
            param("qx", "Q_{x}", 100.0, 1.0, 10000.0).log().redesign(),
            param("qv", r"Q_{\dot x}", 10.0, 0.1, 1000.0).log().redesign(),
            param("r", "R", 1.0, 0.001, 100.0).log().redesign(),
        ],
        telemetry: &[("k", N * 2 * N), ("cost", 1)],
    },
    Spec {
        key: "mpc",
        name: "Model predictive (MPC)",
        family: "optimal",
        params: &[
            param("horizon", "N", 25.0, 5.0, 60.0).redesign(),
            param("step", r"\Delta t", 0.02, 0.005, 0.1).unit("s").redesign(),
            param("qx", "Q_{x}", 100.0, 1.0, 10000.0).log().redesign(),
            param("qv", r"Q_{\dot x}", 10.0, 0.1, 1000.0).log().redesign(),
            param("r", "R", 1.0, 0.001, 100.0).log().redesign(),
            param("iters", r"i_{\max}", 80.0, 5.0, 400.0).log(),
        ],
        telemetry: &[
            ("plan", 60),
            ("predicted", 60),
            ("cost", 1),
            ("iterations", 1),
            ("active", 1),
            ("size", 1),
        ],
    },
    Spec {
        key: "swing_up",
        name: "Energy swing-up",
        family: "underactuated",
        params: &[
            param("ke", "k_{E}", 12.0, 0.0, 40.0),
            param("kc", "k_{c}", 2.0, 0.0, 10.0),
            param("kv", "k_{v}", 2.5, 0.0, 10.0),
            param("amax", r"a_{\max}", 12.0, 1.0, 40.0),
            param("catch", r"\Delta q_{\text{catch}}", 0.35, 0.05, 1.5).unit("rad | m"),
            param("catch_speed", r"\dot q_{\text{catch}}", 4.0, 0.5, 20.0).unit("rad/s | m/s"),
        ],
        telemetry: &[("energy", 1), ("energy_ref", 1)],
    },
];

/// Writes a controller's telemetry fields by name into its slot of the snapshot.
#[derive(Debug)]
pub struct Telemetry<'a> {
    spec: &'static Spec,
    buf: &'a mut [f64],
}

impl<'a> Telemetry<'a> {
    #[must_use]
    pub fn new(kind: Kind, buf: &'a mut [f64]) -> Self {
        Self { spec: kind.spec(), buf }
    }

    pub fn put(&mut self, name: &str, v: &[f64]) {
        let Some((off, len)) = self.spec.telemetry_field(name) else {
            debug_assert!(false, "unknown telemetry field {name}");
            return;
        };
        let k = v.len().min(len);
        self.buf[off..off + k].copy_from_slice(&v[..k]);
    }
}

/// Telemetry slots must fit this many values.
pub const TELEMETRY_LEN: usize = 160;

/// JSON description of every controller (for the browser).
#[must_use]
pub fn specs_json() -> String {
    let mut out = String::from("[");
    for (i, s) in SPECS.iter().enumerate() {
        let params: Vec<String> = s
            .params
            .iter()
            .map(|p| {
                format!(
                    "{{\"key\":\"{}\",\"tex\":\"{}\",\"default\":{},\"min\":{},\"max\":{},\"log\":{},\"unit\":\"{}\"}}",
                    p.key,
                    p.tex.replace('\\', "\\\\"),
                    p.default,
                    p.min,
                    p.max,
                    p.log,
                    p.unit
                )
            })
            .collect();
        let mut off = 0;
        let tele: Vec<String> = s
            .telemetry
            .iter()
            .map(|(k, len)| {
                let e = format!("\"{k}\":[{off},{len}]");
                off += len;
                e
            })
            .collect();
        let _ = write!(
            out,
            "{}{{\"id\":{i},\"key\":\"{}\",\"name\":\"{}\",\"family\":\"{}\",\"params\":[{}],\"telemetry\":{{{}}}}}",
            if i > 0 { "," } else { "" },
            s.key,
            s.name,
            s.family,
            params.join(","),
            tele.join(",")
        );
    }
    out.push(']');
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn registry_is_consistent() {
        for (i, k) in Kind::ALL.iter().enumerate() {
            assert_eq!(*k as usize, i);
            let len: usize = k.spec().telemetry.iter().map(|(_, l)| l).sum();
            assert!(len <= TELEMETRY_LEN, "{} telemetry too long ({len})", k.spec().key);
        }
    }
}
