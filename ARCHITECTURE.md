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
build, observation, restore and profiling requests and disables further commands.

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
numbers (currently 3, 2 and 4). Requested joint targets travel separately from `Q_TARGET`, which reports the
computed equilibrium. Runtime target controls use this requested-target state rather than mutating the
authored or installed document. The shared TypeScript `Core` interface defines scalar exports for runtime
and test harnesses.
Controller parameters retain compact numeric arrays at the ABI but implementations read keys declared in
their registry specs.

`RECOVERY` in the primary snapshot and each slot reports `[recoveries, loop retries, last recovery time]`.
Counters start at zero on reset/new slots; last recovery is −1 until one occurs. Recovery restores the last
good position at rest and restarts the energy budget. Loop retries redo a step with finer substeps and do
not alone imply recovery. Recovery is visible in scope signals and the energy gauge, and invalidates a
guided experiment's verification result. The intentionally incomplete Coriolis-off demonstration restores
complete dynamics after one simulated second on constrained mechanisms, keeping the energy comparison
bounded; the recovery check remains active for it.

Worker step-cost timing clones the ongoing primary, including controller memory, before the timer.
It measures isolated integration. Opt-in worker profiling (`make e2e-profile`) measures actual ticks and
separates integration, publication, snapshot transport, checkpoint retention and periodic benchmark work
for both transports. Only one profiling request runs at a time, and phase clocks are read only during
that request. It excludes UI rendering. See [BENCHMARKS.md](BENCHMARKS.md) for measurements and limits.
Physics, publication, rendering, equations, plots and history retain distinct time bases; profiling is
required before changing their cadences.

## Presentation and history

`DocumentSession` owns the revision counter and coordinates accepted documents and builds.
`model/fields` centralizes physical domains, units, authored uniform-rod dependencies and explicit control
ranges. Authored physical/default/initial-state edits return a `PreparedModel` containing the independent
candidate and its compilation-reuse decision.
They use the installed symbolic structure when compatible. They replace defaults,
parameter metadata, initial conditions, targets and limits; refold coefficients; and validate initial inertia.
The same revision-checked commit/reset path applies, so undo, reset and uniform-rod semantics are preserved.
Changes to topology, mounts, actuators, tool assignment, closures or elements derive a new structure.
Rejection leaves the installed world and overrides untouched. Shadows clone the valid design and folded
model, then initialize their own controller and references; pending invalidation remains pending.

`scene/drawing` defines the Pen context, `pens` implements backends/labels, `primitives` draws mechanical
symbols, `loads` computes force/moment balance, `schematic` assembles a Figure, and `schematic-view` manages
screen layers and redraws. Colors belong to the Pen, so no active picture changes a module-global palette.

The worker retains complete `World` checkpoints: time, joints, physical overrides, targets/settings,
references, controller memory/designs, shadow assignments, diagnostic counters and energy budgets.
Immutable tapes, coefficients and optimal-control matrices are shared; mutable execution state remains
private. Restoring a checkpoint discards its future and does not reset controllers or replay joint setters.
The UI adopts restored configuration without sending commands that would reset the core. Native and
real-WASM tests reproduce future snapshots exactly after the same sequence of fixed steps.

History samples at most 30 times per simulated second, retains at most 600 frames, and evicts earlier when
a conservative 64 MiB retained-size budget is reached. Estimates count inline values and vector capacities,
including plan and controller allocations, with shared coefficients/designs charged in full per checkpoint.
They exclude immutable compiled code, derivation text and allocator bookkeeping. This is not a bound on
total browser or WASM memory. IDs survive clears;
reset/rebuild invalidate old IDs. Expired restores preserve the live world. UI history lazily retains the
transferred double-precision observations, parameters and push position; it follows worker eviction.
The UI holds a bounded array of frame references; expiry and branching use ordinary array operations.
This is in-session branching from saved state; portable command-log replay is a separate feature.

Goals exposes up to 16 additional ordered point/orientation tasks on ground, body or tool frames.
The core validates and installs the whole list atomically across slots; zero frequency disables a task.
Built-ins precede this list, with posture/manipulability last. Experiments clear additional tasks as part
of setup; mechanism replacement clears their body references. Spatial kinematics, Jacobians, constraints
and expression growth still need a separate design review.

## Rust compatibility and checks

The core uses Rust 2024 with the minimum toolchain declared in the workspace manifest (currently 1.90).
CI checks all native targets and the WASM library on that minimum version as well as running tests and
strict Clippy on stable. Safe Rust is enforced except for the exported-symbol attributes in the scalar
FFI. Formatting, pedantic/nursery Clippy and warning-free rustdoc were checked during the Rust audit.
Boolean command flags require exactly 0 or 1; physical tolerances do not apply to protocol tags.
