//! The primary simulation and up to four shadows that compare controllers side by side.
//!
//! All slots share the mechanism, settings and disturbances; each has its own state and controller.
//! Controller parameters are kept per kind and shared by every slot running that kind.

use crate::control::{Kind, specs_json};
use crate::layout::{Flag, SLOTS, off, slot};
use crate::memory::vec_bytes;
use crate::model::Model;
use crate::sim::Sim;

#[derive(Clone, Debug)]
pub struct World {
    primary: Sim,
    /// Slots 1 … SLOTS − 1.
    shadows: Vec<Option<Sim>>,
    /// Parameters of every controller kind, in the order of its spec.
    params: Vec<Vec<f64>>,
    out: Vec<f64>,
}

impl World {
    #[must_use]
    pub fn new(model: Model) -> Self {
        Self {
            primary: Sim::new(model, Kind::Passive),
            shadows: (1..SLOTS).map(|_| None).collect(),
            params: Kind::ALL.iter().map(|k| k.defaults()).collect(),
            out: vec![0.0; off::LEN],
        }
    }

    #[must_use]
    pub const fn primary(&self) -> &Sim {
        &self.primary
    }

    pub const fn primary_mut(&mut self) -> &mut Sim {
        &mut self.primary
    }

    /// The simulation in slot `i`, if that slot is running.
    #[must_use]
    pub fn slot(&self, i: usize) -> Option<&Sim> {
        if i == 0 { Some(&self.primary) } else { self.shadows.get(i - 1)?.as_ref() }
    }

    fn each(&mut self, mut f: impl FnMut(&mut Sim)) {
        f(&mut self.primary);
        self.shadows.iter_mut().flatten().for_each(f);
    }

    fn update(&mut self, mut f: impl FnMut(&mut Sim) -> bool) -> bool {
        if !f(&mut self.primary) {
            return false;
        }
        for s in self.shadows.iter_mut().flatten() {
            f(s);
        }
        true
    }

    /// Replace the mechanism, keeping the controllers of every slot and the global settings.
    pub fn rebuild(&mut self, model: Model) {
        let old = &self.primary;
        let (settings, kind) = (old.settings(), old.kind);
        self.primary = Sim::new(model, Kind::Passive);
        self.primary.apply_settings(settings);
        self.set_controller(0, Some(kind));
        let shadows = std::mem::take(&mut self.shadows);
        self.shadows = (1..SLOTS).map(|_| None).collect();
        for (i, s) in shadows.into_iter().enumerate() {
            self.set_controller(i + 1, s.map(|s| s.kind));
        }
    }

    /// Run controller `kind` in slot `i`; `None` turns a shadow off. A new shadow starts from the primary's
    /// state. Controllers that cannot run on this mechanism fall back to passive in the primary.
    pub fn set_controller(&mut self, i: usize, kind: Option<Kind>) {
        if i >= SLOTS {
            return;
        }
        self.primary.refresh_plan();
        let ok = |k: Kind, s: &Sim| k.applicable(&s.model, &s.plan).is_ok();
        if i == 0 {
            let k = kind.filter(|&k| ok(k, &self.primary)).unwrap_or(Kind::Passive);
            self.primary.set_controller(k);
        } else {
            self.shadows[i - 1] = kind.filter(|&k| ok(k, &self.primary)).map(|k| self.primary.fork(k));
        }
    }

    pub fn set_ctrl_param(&mut self, kind: Kind, i: usize, v: f64) -> bool {
        let Some(p) = kind.spec().params.get(i) else { return false };
        if !v.is_finite() {
            return false;
        }
        self.params[kind as usize][i] = v.clamp(p.min, p.max);
        if p.redesign {
            self.each(|s| {
                if s.kind == kind {
                    s.redesign_controller();
                }
            });
        }
        true
    }

    #[must_use]
    pub fn ctrl_params(&self, kind: Kind) -> &[f64] {
        &self.params[kind as usize]
    }

    /// Bring every shadow back to the primary's state (and restart their controllers).
    pub fn sync_shadows(&mut self) {
        self.primary.refresh_plan();
        for sh in self.shadows.iter_mut().flatten() {
            *sh = self.primary.fork(sh.kind);
        }
    }

    pub fn reset(&mut self) {
        self.each(Sim::reset);
    }
    pub fn set_dt(&mut self, dt: f64) -> bool {
        self.update(|s| s.set_dt(dt))
    }
    pub fn set_param(&mut self, i: usize, v: f64) -> bool {
        if !self.primary.can_set_param(i, v)
            || self.shadows.iter_mut().flatten().any(|s| !s.can_set_param(i, v))
        {
            return false;
        }
        self.each(|s| s.apply_param(i, v));
        true
    }
    /// Validate the entire list before installing it in any slot.
    pub fn set_tasks(&mut self, flat: &[f64]) -> bool {
        let Ok(tasks) = crate::plan::TaskGoal::parse(flat, &self.primary.model) else { return false };
        self.each(|s| s.plan.config.tasks.clone_from(&tasks));
        true
    }

    pub fn set_target(&mut self, i: usize, v: f64) -> bool {
        self.update(|s| s.set_target(i, v))
    }
    pub fn set_flag(&mut self, f: Flag, v: f64) -> bool {
        self.update(|s| s.set_flag(f, v))
    }
    pub fn kick(&mut self, i: usize, dqd: f64) -> bool {
        if !self.primary.can_kick(i, dqd) || self.shadows.iter_mut().flatten().any(|s| !s.can_kick(i, dqd)) {
            return false;
        }
        self.update(|s| s.kick(i, dqd))
    }
    pub fn set_input(&mut self, i: usize, v: f64) -> bool {
        if i >= self.primary.model.nu || !v.is_finite() {
            return false;
        }
        self.each(|s| {
            if let Some(u) = s.u_manual.get_mut(i) {
                *u = v;
            }
        });
        true
    }
    /// Set a joint state in the primary and restart the shadows from it.
    pub fn set_state(&mut self, i: usize, q: f64, qd: f64) -> bool {
        if !self.primary.set_state(i, q, qd) {
            return false;
        }
        self.sync_shadows();
        true
    }

    pub fn step(&mut self, n: u32) {
        let params = &self.params;
        for s in std::iter::once(&mut self.primary).chain(self.shadows.iter_mut().flatten()) {
            let p = &params[s.kind as usize];
            for _ in 0..n {
                s.step(p);
            }
        }
    }

    /// Refresh the snapshot: the primary's full state, then every slot's block (kind −1 when off).
    pub fn publish(&mut self) -> &[f64] {
        let (params, out) = (&self.params, &mut self.out);
        self.primary.publish(out, &params[self.primary.kind as usize]);
        let slots =
            std::iter::once(Some(&mut self.primary)).chain(self.shadows.iter_mut().map(Option::as_mut));
        for (s, block) in slots.zip(out[off::SLOT..].as_chunks_mut::<{ slot::LEN }>().0) {
            if let Some(s) = s {
                s.publish_slot(block, &params[s.kind as usize]);
            } else {
                block.fill(0.0);
                block[slot::KIND] = -1.0;
            }
        }
        &self.out
    }

    #[must_use]
    pub fn out(&self) -> &[f64] {
        &self.out
    }

    /// Checkpoint storage, including inline values and allocation capacities. Shared code is excluded;
    /// shared coefficients/designs are charged in full. This excludes allocator bookkeeping.
    #[must_use]
    pub fn retained_bytes(&self) -> usize {
        size_of::<Self>()
            + self.primary.heap_bytes()
            + vec_bytes(&self.shadows)
            + self.shadows.iter().flatten().map(Sim::heap_bytes).sum::<usize>()
            + vec_bytes(&self.out)
            + vec_bytes(&self.params)
            + self.params.iter().map(vec_bytes).sum::<usize>()
    }

    /// Copy the ongoing primary, including controller memory, before starting the integration timer.
    #[must_use]
    pub fn benchmark(&self) -> Benchmark {
        let sim = self.primary.clone();
        let params = self.params[sim.kind as usize].clone();
        Benchmark { sim, params }
    }

    /// `{"specs": […], "applicable": {kind: null | "reason"}, "params": [[…] per kind]}`.
    #[must_use]
    pub fn controllers_json(&mut self) -> String {
        self.primary.refresh_plan();
        let s = &self.primary;
        let app: Vec<String> = Kind::ALL
            .iter()
            .map(|k| match k.applicable(&s.model, &s.plan) {
                Ok(()) => "null".to_owned(),
                Err(e) => format!("\"{e}\""),
            })
            .collect();
        let params: Vec<String> = self
            .params
            .iter()
            .map(|p| format!("[{}]", p.iter().map(f64::to_string).collect::<Vec<_>>().join(",")))
            .collect();
        format!(
            "{{\"specs\":{},\"applicable\":[{}],\"params\":[{}]}}",
            specs_json(),
            app.join(","),
            params.join(",")
        )
    }
}

/// Isolated ongoing integration measurement; preparing it never changes the live world.
#[derive(Debug)]
pub struct Benchmark {
    sim: Sim,
    params: Vec<f64>,
}
impl Benchmark {
    pub fn run(&mut self, n: u32) {
        for _ in 0..n {
            self.sim.step(&self.params);
        }
        std::hint::black_box(&self.sim.q);
    }
}
