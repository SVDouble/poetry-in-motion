# Architecture review

Reviewed on 2026-10-22, after `9ecb67d`.

This review covers the Rust symbolic and numeric engine, the WASM boundary and worker, the TypeScript
application and rendering, the Python oracle, and the build and verification workflows. Recommendations
below are follow-up work; the constants cleanup is already committed separately.

## Current design

The dependency flow is clear:

```text
Mechanism document → Rust description → symbolic derivation
                                           ├─ LaTeX/JSON → equation views
                                           └─ numeric tapes → Model → Plan + Controller → Sim → World
                                                                                              ↓
UI commands → SimClient → worker → scalar WASM exports                       flat snapshot + metadata
                                                                                              ↓
                                                        schematic, live equations, plots, timeline

Independent SymPy derivation → generated Rust oracle kernels → physics verification
```

The strongest choices are worth preserving: displayed equations and simulated dynamics come from the same
derivation; SymPy supplies an independent check; physics runs outside the UI thread; controller specs drive
generic controls and telemetry; and a common drawing interface serves canvas, SVG, and TikZ. Static hosting
and the small dependency set suit the educational purpose.

## Fix correctness and ownership first

### 1. Give simulation settings one lifecycle

[`Sim::fork`](core/src/sim.rs) copies goals and several settings manually but omits `plan.tuning`.
[`World::rebuild`](core/src/world.rs) retains `umax`, `coriolis`, and `dt`, while constructing a fresh plan.
Consequently shadow creation/synchronization and rebuilding can discard configured behavior. The browser
reapplies settings during simulation loading, which compensates for some rebuild paths but leaves the core
contract inconsistent.

Confirmed through the real WASM module: set `MotorSpeed = 1`, manual input to 100, and joint speed to 5;
then create a passive shadow and advance one step. The primary input is 0 and the shadow input is 100.
Rebuilding also loses the configured motor speed.

Introduce a typed `SimulationSettings` value containing persistent user configuration, separate from
targets, references, and controller state. Copy/apply that value as a unit when creating shadows or
rebuilding. Define explicitly which targets survive a topology change. Test settings changes before and
after shadow creation, synchronization, and rebuilds. Timestep changes should also invalidate controller
designs: `World::set_dt` currently changes the step without marking them dirty.

### 2. Publish only validated snapshot reads

[`SimClient.pull`](web/src/sim/client.ts) copies shared data directly into the visible snapshot and updates
stats before checking the second sequence number. If all four attempts fail, the visible buffers may hold
an inconsistent read. The frame loop uses the snapshot regardless of the return value.

Read data and stats into reusable staging buffers, validate the sequence, then commit them together.
Keep the previous validated snapshot on failure. Add a focused concurrency regression test. Retain the
SharedArrayBuffer path and message fallback.

### 3. Validate documents before derivation

[`migrate`](web/src/model/description.ts) fills legacy fields but largely trusts the incoming document.
[`Desc::parse`](core/src/mech.rs) checks sizes and references, but accepts non-finite fields, coerces unknown
joint codes, and does not enforce physical parameter validity. A WASM probe confirmed that both a NaN
description version and a negative mass return build success.

Separate migration from validation. Validate finite numbers, integral indices/counts, supported tags,
parent ordering, and physical parameter domains before symbolic work. Preserve valid special cases such
as zero rotational inertia for a point mass. Validate imported documents in the UI before traversing their
graphs, and enforce the engine's invariants again in Rust. Use an explicit versioned envelope for shared
links; malformed input should yield a useful error without replacing the current valid mechanism.

## Consolidate contracts and editing semantics

### 4. Generate the remaining shared contract

The constants cleanup now publishes defaults, timestep, and mechanism limits from Rust. Description
limits/record sizes still have TypeScript copies, protected by the new compatibility tests. WASM `Core`
interfaces also repeat across [`worker.ts`](web/src/sim/worker.ts), [`testing.ts`](web/src/sim/testing.ts),
and [`wasm.test.ts`](web/src/sim/wasm.test.ts). Snapshot/flag names and controller phases have similar
cross-language coordination costs.

Generate a small checked-in TypeScript contract from Rust declarations, with a freshness check in the
existing build workflow. Share one WASM interface between runtime and test harnesses. Include a protocol
version and validate the layout's offsets, lengths, and required defaults at startup. Controller settings
could follow the existing `SPECS` pattern, carrying ranges and units along with defaults; keep pedagogical
explanations in the browser.

### 5. Share field definitions between builder surfaces

[`Editor.fields/bodyFields`](web/src/builder/editor.ts) and
[`bodyControls`](web/src/builder/inspector.ts) describe the same body properties independently. For
example, the side-panel length maximum is 2 while the inspector permits 3; their mass ranges also differ.
Grid and angle steps are now shared, but the larger source of drift remains.

Define body/joint/element field descriptors once: key, units, constraints, applicability, and edit effect.
Let sliders and steppers render those descriptors, with explicit presentation overrides when needed.
Use one document-edit command path for validation, normalization, history, and selection remapping.

### 6. Shrink the application coordinator

[`main.ts`](web/src/main.ts) is over 1,000 lines and owns document state, history, build scheduling, stage
transitions, menus, pointer/keyboard interaction, exports, and rendering cadence.

Extract a document/session store, a build coordinator, and viewport interaction handling. Keep `App` as
composition and stage lifecycle wiring. Distinguish edits to the parameter vector (mass, length, COM,
inertia, friction, gravity) from edits requiring recompilation: the former can reuse `Model::set_params`
and tape folding. Mount coefficients and topology changes still require derivation. Preserve the existing
sequence guard that rejects stale build responses.

## Generalize the engine at its existing seams

### 7. Separate compiled mechanism data from mutable evaluation state

[`Model`](core/src/model.rs) combines topology, compiled expressions, parameters, scratch buffers, and the
derivation string. [`Tape`](core/src/tape.rs) likewise holds both compiled tables and mutable evaluation
arrays. Each shadow deep-clones the model, and `World::bench` creates another fork on every measurement.

Split read-only compiled mechanism data from parameter folding and per-simulation evaluation workspace.
Share compiled data between slots while retaining independent state, references, controllers, and scratch
buffers. This reduces copy cost and makes snapshot/checkpoint ownership clearer. Profile fork/build costs
and full worker-tick cost before changing the hot loop; tape evaluation itself already avoids allocations.

### 8. Turn the fixed goal hierarchy into typed tasks

[`Plan`](core/src/plan.rs) combines settings, equilibrium solving, linearization, IK, references, pushes,
and secondary goals. Its `Goals`/`SubTask` representation fixes orientation and one elbow point into the
API. [`stacked_accel`](core/src/control/task.rs) supports secondary levels for open chains but falls back
to the tool task alone for closed loops.

First separate equilibrium/design data, reference generation, and goal configuration. Then represent
secondary goals as typed point, orientation, posture, and manipulability tasks with an ordered priority.
Reuse the existing stack solver rather than introducing another controller framework. A subsequent step
can resolve the stack in the constraint tangent space so closed loops gain the same hierarchy. This would
support multiple body targets and multiple elbow goals without more named flags. Preserve leakage and
constraint tests at every level.

### 9. Organize numerical policy by mathematical purpose

Projection, finite differences, rank decisions, regularization, iteration limits, and divergence handling
are spread across `dynamics.rs`, `plan.rs`, `mat.rs`, `control/task.rs`, and `sim.rs`.

Name and document related numerical policies locally, including units and scale assumptions. Generalize
shared solver helpers where the mathematics is the same. Equal numeric literals can have different
meanings: a rank threshold, contact tolerance, and finite-difference step should remain distinct. Expose
user-meaningful behavior through settings; keep implementation tolerances internal and test them across
mechanism scales. Report numerical fallback/divergence events in telemetry instead of silently showing
the restored state as ordinary motion.

## Keep observation and presentation modular

### 10. Split the schematic around the existing Pen abstraction

[`schematic.ts`](web/src/scene/schematic.ts) is over 1,700 lines, containing all three Pen implementations,
mechanical symbols, force/reaction calculations, labels, layering, and the view class. Its mutable
module-level ink context also makes drawing depend on ambient state.

Extract Pen backends, mechanical primitives, diagnostic geometry/free-body calculations, and view/layer
management. Pass an explicit drawing context. Preserve one mechanism drawing routine for screen and
exports. Continue testing force balance and export output; replacing this with unrelated renderers would
lose a useful consistency guarantee.

### 11. Clarify observation, history, and replay

[`Timeline`](web/src/app/timeline.ts) stores Float32 snapshots at 30 Hz for 20 seconds, while
[`Recorder`](web/src/scope/scope.ts) keeps separate signal history. Equation, plot, and snapshot cadences
are configured in several modules. The production bundle also puts all stages and controller content in
one JavaScript chunk, currently about 545 kB minified.

Give observation cadence an explicit configuration and share time/reset events across plots and rewind.
Extract recorder storage from plot widgets; consider a circular buffer if profiling shows shifting arrays
is material. Keep observed snapshots distinct from resumable checkpoints: current rewind restores joint
state and restarts controller/reference state. Exact replay would need controller state and commands too.
Lazy-load educational pages or stages if startup profiling justifies it; KaTeX is a shared dependency, so
measure the benefit before rearranging chunks.

## Verification and documentation

### 12. Extend the independent tests and align documentation

The test strategy is a major strength: invariant/property tests, independent SymPy kernels, real-WASM
tests, builder reconstruction, and 228 guided-experiment checks. Keep the Python oracle independent from
the production CAS. Add independent examples for branched/fixed/prismatic mechanisms and constrained
cases, plus boundary fixtures at the published body/DOF/closure/element limits.

CI already checks oracle freshness through `test_generated_files_are_up_to_date`; retain that guarantee.
Add a small browser smoke job for worker startup, both snapshot transports, settings, rebuilds, and
timeline stepping. Keep the full crawl for broader regression coverage, replacing fixed sleeps with
observable readiness where possible. Test worker failures after startup too: the current client can leave
pending build promises unresolved when the worker dies.

[`PROJECT.md`](PROJECT.md) mixes historical choices with current requirements: it still lists production
SymPy generation and three.js/WebGPU rendering as decisions, while runtime derivation and canvas are now
implemented. Mark superseded decisions clearly and move current architectural invariants into maintained
documentation. Remove stale 3D-renderer comments in the schematic as part of that cleanup.

## Suggested sequence

1. Fix settings inheritance, snapshot commit semantics, and document validation, with targeted regressions.
2. Generate the shared contract and unify builder field descriptors.
3. Extract document/build coordination, then split rendering modules without changing output.
4. Introduce shared compiled mechanism data and typed task stacks when adding more shadows or goals.
5. Broaden oracle/browser coverage and update architectural decisions alongside those changes.

The existing crate/package boundaries are adequate for this sequence. Most benefit comes from clearer
state ownership and contracts inside them; additional packages or a UI framework are not prerequisites.
