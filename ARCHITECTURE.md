# Current architecture

The app runs entirely in the browser: a Rust CAS and simulation engine execute inside a WASM worker;
TypeScript authors documents and displays the resulting equations and observations. SymPy is an
independent test oracle. Canvas, SVG and TikZ use the same mechanical drawing path.

## Ownership and transitions

| State | Owner and transition policy |
|---|---|
| Authored draft | `DocumentSession.draft`; each edit/undo immediately changes its revision. History stores authored documents. |
| Installed document | `DocumentSession.installed`; captured from the build ticket accepted before worker commit. A newer draft can coexist with an older installed model. |
| Runtime world | Worker + Rust `World`; failed preparation retains the running world. An accepted commit installs its own metadata and initial observation. |
| Persistent settings | Captured/applied by `Sim::settings` / `apply_settings`; copied before shadow design, retained across rebuilds. |
| Mechanism-relative targets | Configuration of the current mechanism; reset on replacement. Additional typed tasks and body indices follow this lifetime. |
| Physical overrides | Current parameter vector, distinct from authored defaults. Runtime mass/length overrides do not enforce authored uniform-rod dependencies. |
| Per-slot state | Separate joints, integration budget, reference filters, controller memory and diagnostic counters. |
| Observations | Generation + sequence + stats + data accepted coherently; failed reads retain the last accepted frame. |

A prepared build can commit only while its captured revision is current. Once accepted, its acknowledgment
installs that captured document even if a later draft is pending. A failed later draft leaves the last
installed simulation usable. Empty documents clear the installed UI view. Worker failure settles pending
build/observation requests and disables further commands.

Experiments own an abort scope and generation. Physical waits advance on simulation time, remain pending
while paused, and acknowledge preceding commands. Leaving the stage, editing, rebuilding or replacing a
run cancels it and synchronously releases its push before a new owner starts.

## Core execution

`mech` / `cas` derive symbolic expressions; `doc` produces the explanation. `Model` compiles numeric tapes
and describes the topology. Each `Tape` contains three kinds of data:

- `Arc<Program>`: immutable symbolic tables, indices and harmonics shared by clones.
- `Arc<Coefficients>`: folded/merged physical coefficients; setting parameters replaces them for that tape.
- `Workspace`: private harmonic, monomial and state evaluation scratch.

Derivation text is also shared. Model description/topology arrays remain small owned copies. Slot states,
reference/controller memory and evaluation scratch never alias. This allows future parameter divergence
without silently changing another slot.

`Plan` separates `Configuration`, cached `Design`, and live `References`. Validated simulation setters
apply an ordered invalidation: Plan invalidation includes controller redesign; controller-only invalidation
cannot clear pending Plan work. Native code changing cache-dependent configuration must use those setters.

| Input change | Recomputed state |
|---|---|
| Physical parameters, joint targets, regulator setpoint, Baumgarte rate | Equilibrium/linearization, then controller design. |
| Timestep, controller kind, redesign-marked controller parameters | Controller design. |
| Tool targets, secondary task goals | Live reference/task evaluation; regulators update their setpoint when appropriate. |
| Reset/state jump | References, controller memory and energy budget restart; setpoint-dependent design invalidates. |

Built-in orientation and elbow controls are available when the tool has remaining degrees of freedom,
including on constrained mechanisms. They become typed `TaskGoal`s, followed by additional ordered frame
point/orientation tasks. Posture and manipulability remain lower preferences. Loop hierarchies use
`H = Z (Zᵀ M Z)⁻¹ Zᵀ`, where `Z` spans the constraint tangent space. Every lower correction is tangent;
constraint curvature supplies the affine base acceleration. Rank loss and damping remain best-effort
tracking, with higher-level leakage checked independently of target error. This is a planar interface.

## Contracts and diagnostics

`core/src/layout.rs` is the snapshot/flag metadata authority. `make gen` writes browser catalogs and
independent oracle files; `make gen-check` checks freshness, including in CI. Runtime offsets/defaults
still come from WASM. `parseLayout` checks compatibility, field widths, bounds, overlap, required defaults,
flags and capacities against the compiled browser contract. Real-WASM tests catch a stale binary.

The WASM ABI, worker transport/shared-memory header and authored document format have independent version
numbers. The shared TypeScript `Core` interface defines scalar exports for runtime and test harnesses.
Controller parameters retain compact numeric arrays at the ABI but implementations read keys declared in
their registry specs.

`RECOVERY` in the primary snapshot and each slot reports `[recoveries, loop retries, last recovery time]`.
Counters start at zero on reset/new slots; last recovery is −1 until one occurs. Recovery restores the last
good position at rest and restarts the energy budget. Loop retries redo a step with finer substeps and do
not alone imply recovery. Recovery is visible in scope signals and the energy gauge, and invalidates a
guided experiment's verification result. The intentionally incomplete Coriolis-off demonstration restores
complete dynamics after one simulated second on constrained mechanisms, keeping the energy comparison
bounded; the recovery check remains active for it.

Worker step-cost timing prepares an isolated primary and its initial controller design before the timer.
It measures integration of that restarted copy, not live-world total tick cost. See [BENCHMARKS.md](BENCHMARKS.md)
for reproducible native cost breakdowns and their limits. Physics, publication, rendering, equations, plots
and history retain distinct time bases; profiling is required before changing their cadences.

## Presentation and history

`DocumentSession` coordinates accepted documents and builds. `model/fields` centralizes physical domains,
units, authored uniform-rod dependencies and explicit control ranges. Authored parameter edits still rebuild
so history and reset behavior remain consistent.

`scene/drawing` defines the Pen context, `pens` implements backends/labels, `primitives` draws mechanical
symbols, `loads` computes force/moment balance, `schematic` assembles a Figure, and `schematic-view` manages
screen layers and redraws. Colors belong to the Pen, so no active picture changes a module-global palette.

Timeline history consists of observations. Resume here restores the primary joint state, restarts
controllers/shadows and keeps current simulation time. It is not a full execution checkpoint. Exact replay,
spatial mechanisms and a UI for arbitrary ordered task lists require separate product/design work.
