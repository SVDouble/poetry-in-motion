# Architecture review

Refined on 2026-10-22 against `1a3dbed`; implementation baseline: `9ecb67d`.

The main architectural issue is ownership across transitions: a new shadow, mechanism, experiment, or
snapshot can inherit some state while retaining other state from a different context. Fix those
transitions first. The existing Rust engine, worker, TypeScript UI, and independent Python oracle form
useful boundaries to retain.

## Scope and evidence

This review covers symbolic derivation, numeric evaluation, controllers, simulation, the worker protocol,
document editing, rendering, history, and verification. Findings distinguish three kinds of evidence:

- **Reproduced:** observed through the release WASM module or a controlled harness running production code.
- **Code path:** a specific failure path follows from the implementation; its frequency is unmeasured.
- **Proposal:** a maintainability improvement or feature extension, with a stated reason to undertake it.

The earlier baseline passed `make check`, production build, 228 guided-experiment runs, and the browser
crawl across 16 presets. Those checks do not establish correctness of the transition cases below. This
refinement adds investigative probes and changes documentation; it does not implement the proposed fixes.

## Responsibilities to preserve

| Area | Responsibility and existing strength |
|---|---|
| `core/src/cas`, `mech.rs`, `doc.rs` | Derive equations and their explanation from one symbolic representation. |
| `tape.rs`, `model.rs`, `dynamics.rs` | Compile/evaluate the equations and solve constrained dynamics. |
| `plan.rs`, `control/`, `sim.rs`, `world.rs` | Form references and control inputs, integrate state, and coordinate independent slots. |
| `ffi.rs`, `web/src/sim/` | Transfer commands, metadata, and observations across the WASM/worker boundary. |
| `web/src/model`, `builder`, `app` | Author mechanisms, coordinate builds, and manage the interactive session. |
| `scene`, `eq`, `scope` | Present the mechanism, equations, and measurements. The Pen interface shares drawing logic across canvas, SVG, and TikZ. |
| `sym/`, Rust/WASM/browser tests | Check equations independently and verify physical, controller, and UI behavior. |

Within those areas, make these ownership distinctions explicit. These are logical responsibilities;
they need not become separate classes or packages.

| State | Authority and lifetime |
|---|---|
| Authored mechanism | UI document: graph, built parameter defaults, initial conditions; revision changes on every edit. |
| Installed mechanism | Worker/core: compiled model and derivation associated with one accepted document revision. |
| Persistent simulation settings | Core configuration applied consistently to current and future slots; UI maintains controls for it. |
| Targets and selected bodies/joints | Mechanism-relative state; rebuild needs an explicit reset or remapping policy. |
| Runtime physical overrides | Current parameter vector; intentionally distinct from authored defaults in the Model tab. |
| Per-kind controller parameters | `World` owns them; slots using that kind receive the same parameters. |
| Per-slot execution state | Joint state, references, controller memory, time, and energy budget. |
| Display preferences and observations | UI-owned preferences; validated snapshots carry the installed mechanism's identity. |

## Priority and findings

**P1** means current behavior can mix incompatible state or invalidate a simulation result. **P2** means
a session/lifecycle defect or a maintenance improvement to address after those cases. Effort descriptions
below identify the smallest useful change, not a schedule estimate.

| ID | Priority | Evidence | First deliverable |
|---|---|---|---|
| F1 | P1 | Reproduced + code path | Correct shadow inheritance and design-cache invalidation. |
| F2 | P1 | Code path | Commit snapshot data and stats only after a successful sequence check. |
| F3 | P1 | Code path | Bind build results, installed metadata, and snapshots to a document revision. |
| F4 | P1 | Reproduced + code path | Validate descriptions and live mutations before they affect engine state. |
| F5 | P2 | Reproduced + code path | Give experiments simulation-time waits and cancellation with scoped cleanup. |

### F1. Shadow inheritance includes stale derived state

**Evidence.** [`Sim::fork`](core/src/sim.rs#L107) constructs and designs a fresh simulation, then copies
parameters and targets. It omits `plan.tuning` and does not unconditionally invalidate the already computed
plan. [`settle`](core/src/sim.rs#L144) resets references/controller state but only marks the plan dirty in
a particular setpoint case. [`World::rebuild`](core/src/world.rs#L53) preserves only some global settings.

Two WASM probes demonstrate different consequences:

- With motor speed set to 1, positive joint speed 5, and manual input 100, create a passive shadow and step
  once. Primary input is 0; shadow input is 100 because its motor tuning reverted to the default.
- Build a single driven revolute joint with zero gravity and initial/target angle 0. Set its target to
  0.7, create a passive shadow, then publish. Primary target error is approximately 0.7; shadow target
  error is 0. The shadow copied the requested target but retained its original equilibrium cache.

**Minimal fix.** Copy all inherited configuration before computing derived state, or explicitly invalidate
and refresh the plan after copying. Preserve the intended restart of shadow references and controller
memory. Define configuration capture/application helpers for fork, synchronization, and rebuild. Audit
invalidation separately: [`World::set_dt`](core/src/world.rs#L112) changes the integration step without
invalidating discrete LQR/MPC designs, which depend on that step.

**Acceptance.** Set nondefault tuning, parameters, and targets before creating/synchronizing a shadow;
check inherited settings and recomputed equilibrium/gains. Check rebuild retention separately, including
reset/remapping of mechanism-relative targets. Changing `dt` after controller design must trigger redesign.
These fixes can precede a larger `SimulationSettings` extraction.

### F2. Failed snapshot reads can modify the visible state

**Evidence — code path.** [`SimClient.pull`](web/src/sim/client.ts#L97) writes directly into `snap.data`
and updates visible stats before validating the second sequence number. A failed attempt followed by
exhausted retries leaves those writes visible even though `pull()` returns false. The
[`frame` loop](web/src/main.ts#L990) renders the snapshot regardless. This is a consistency defect in the
failure path; no torn frame was reproduced during the earlier crawl.

**Minimal fix.** Copy data and stats into reusable staging buffers. Validate the sequence, then commit the
observation and `lastSeq` together. Keep the previous accepted observation on failure. Retain bounded
retries so the UI cannot spin indefinitely while the writer is active.

**Acceptance.** Force a writer update between the reader's sequence checks. A failed pull must leave data,
stats, and accepted sequence unchanged; a subsequent successful pull must update them coherently. Exercise
both the shared-memory and message transports. F3 adds a separate requirement: coherent data must also
belong to the installed mechanism.

### F3. Build request identity does not establish document/snapshot identity

**Evidence — code path.** [`edited`](web/src/main.ts#L343) immediately changes `this.mech` and schedules a
build; `buildSeq` increments only when [`rebuild`](web/src/main.ts#L390) starts. A possible interleaving is:

1. Build A is awaiting its response.
2. Edit B replaces `this.mech`; B's debounce timer has not fired.
3. A completes and passes the unchanged sequence check.
4. A's derivation is paired with B's document, parameter vector, and robot view.

The sequence guard does protect against builds that have already started. It does not cover this debounce
window. Build errors also leave the edited document alongside the last successful runtime, so the UI
needs an explicit distinction between the draft and installed mechanism.

[`protocol.ts`](web/src/sim/protocol.ts) correlates build requests by `id`, but snapshots carry no installed
model generation. A seqlock-valid observation can therefore still be interpreted using another model's
metadata during a rebuild. These interleavings are inferred from the code, not reproduced browser failures.

**Minimal fix.** Increment the document revision when an edit occurs and capture the exact document with
each build request. Install the captured document and result only if that revision is still current. Keep
the last successful installed view while displaying draft/build errors. Carry an installed generation
through the build acknowledgment and both snapshot transports; accept an observation only with matching
metadata. Keep generation and data under the same consistency check in the shared-memory transport.

Complete the request lifecycle in [`SimClient`](web/src/sim/client.ts#L43): worker errors after startup
currently have no path that settles outstanding build promises. Fail pending requests and expose a terminal
worker state. Coalescing unsent builds is a subsequent optimization; a token cannot interrupt a synchronous
WASM derivation already executing in the worker.

**Acceptance.** Deterministically resolve A during B's debounce window, deliver an obsolete snapshot after
a new build, and fail the current build. None may mix revisions. Worker failure must settle every pending
request exactly once and prevent further sends until recovery. Preserve the FFI's existing behavior of
retaining the previous model when parsing/building fails.

### F4. Validation must cover imports and live mutation

**Evidence.** [`migrate`](web/src/model/description.ts#L542) fills legacy fields but largely trusts values.
[`Desc::parse`](core/src/mech.rs#L119) checks sizes/references, yet converts some invalid tags and indices
and accepts non-finite fields. WASM probes returned build success for a NaN description version and for a
negative mass. NaN is an ABI-level example; it is not a valid literal in a JSON share link. Negative mass
can occur in JSON input.

The same policy must cover [`Sim::set_param`](core/src/sim.rs#L180), which currently checks finiteness but
not the parameter domain, and state/kick entry points. [`Model::from_desc`](core/src/model.rs) also lets
native callers construct models without going through the flat parser. A parser-only patch would leave
other routes inconsistent.

**Minimal fix.** Define mechanism validation independently of the flat codec. Separate legacy migration
from validation in the UI and validate before traversing imported parent chains. Enforce finite values,
integral indices/counts, supported tags, graph invariants, and documented physical domains in the core.
Validate mutations before committing them and provide a useful error outcome at the boundary.

Physical admissibility and slider ranges need separate definitions. Decide how massless links and point
masses are supported; zero rotational inertia can be valid. Positive mass alone also does not guarantee
that every allowed configuration has a nonsingular effective inertia. Preserve deliberate physical
experiments such as zero gravity and zero friction. A versioned share-link envelope is useful when another
migration is needed; introducing it is independent of fixing current validation.

**Acceptance.** Cover malformed imports, cyclic/invalid parents, fractional counts, unknown tags,
non-finite values, invalid physical values, and rejected live mutations. Check valid point-mass and
zero-gravity/friction examples too. Rejection must preserve the last valid installed model and state.

### F5. Experiment time and cancellation differ between runtime and tests

**Evidence.** The browser supplies [`setTimeout` waits](web/src/app/simulate.ts#L33), while
[`verify.test.ts`](web/src/control/verify.test.ts#L361) implements waits by advancing physics. At a different
playback speed, while paused, or under load, an identical action sequence therefore waits for different
amounts of simulated motion. The verification harness also equates milliseconds with step count, retaining
an assumption that `dt = 1 ms`.

[`runExperiment`](web/src/control/runner.ts#L66) checks cancellation between actions, but individual actions
continue after their internal waits. A controlled harness canceled a push during its wait and observed
`setPush(null)`, `PushOn = 0`, and release/done marks afterward. That cleanup can interfere with a newer
push. [`leave`](web/src/app/simulate.ts#L130) and `load` do not invalidate the run token either.

**Minimal fix.** Give a run an explicit cancellation scope and model generation. Recheck ownership after
each await and before mutations; cancel on replacement, stage exit, and model replacement. Cleanup must
release resources still owned by the canceled run without clearing a newer run's push. Distinguish
`waitSimTime` for physical experiment durations from presentation delays or an acknowledgment that a new
snapshot is available. Derive test stepping from `layout.dt`. Define pause behavior explicitly; the
recommended physical-time wait remains pending while paused and is abortable.

**Acceptance.** Run an experiment at multiple playback speeds and timesteps; compare action marks in
simulated time within stepping/sampling tolerance. Cancel during setup and during a push, start another
run, leave the stage, and rebuild. The obsolete run must neither mutate the new session nor mark itself
complete, and cancellation must leave no unowned active push.

## Refactoring and generalization after the fixes

### R1. Consolidate contracts in small steps

First share one TypeScript WASM `Core` interface between runtime and harnesses. Generate the remaining
numeric limits and name/type catalogs from explicit Rust metadata, with freshness and real-WASM ABI
checks. Preserve runtime metadata as the authority for snapshot offsets and defaults. Generated TypeScript
alone cannot establish compatibility with a stale WASM asset.

Extend [`parseLayout`](web/src/sim/layout.ts#L186) to check field bounds/lengths and required metadata.
Use explicit compatibility versions for the WASM layout and worker transport as those contracts change;
the mechanism description version has a different purpose. Controller parameter indices such as
`params[4]` are another coordination point: derive named accessors or typed parameter views from their
specs while retaining the compact numeric ABI. Introduce generation incrementally as a replacement for
manual duplication; a general-purpose schema framework is unnecessary for the first step.

### R2. Share editing semantics while retaining distinct lifetimes

[`Editor.bodyFields`](web/src/builder/editor.ts#L583),
[`bodyControls`](web/src/builder/inspector.ts#L44), and the
[`Model` tab](web/src/app/goals.ts#L510) repeat field semantics and use different ranges. Define units,
admissibility, uniform-rod dependencies, and edit effects once; let each control choose an explicit
presentation range. Preserve runtime overrides as separate from the authored defaults unless the product
explicitly gains an Apply-to-document action.

Extract document/build coordination from `App` around the F3 revision contract. Then consider avoiding
re-derivation for authored parameter-only edits. That fast path must still update the document, history,
initial/reset behavior, and uniform-rod `r`/`I`; calling `set_param` alone does not preserve those semantics.
Mount coefficients and topology still require recompilation. Module length identifies places to inspect;
revision ownership and edit behavior provide the actual extraction boundaries.

### R3. Make design dependencies explicit before splitting Plan

[`Plan`](core/src/plan.rs#L173) mixes goal configuration, reference state, and equilibrium/linearization
caches. Record which inputs invalidate each cache, including model parameters, targets, timestep-dependent
controller design, and tuning used in linearization. Apply those rules through setters or explicit revision
checks. This addresses F1 directly. Extract goal configuration, reference generation, and design results
when those boundaries make the invalidation rules easier to enforce.

For additional body targets, generalize [`sub_tasks`](core/src/plan.rs#L539) into an ordered list of typed
point/orientation tasks, keeping posture/manipulability as explicit lower-priority preferences. For
closed-loop hierarchies, extend [`stacked_accel`](core/src/control/task.rs#L409) to respect constraint
tangent coordinates. This is a feature extension: the current fallback deliberately resolves only the
tool task with loops. Require leakage, feasibility, and constraint tests before exposing more goal levels.
A spatial/3D extension needs a separate review of kinematics, Jacobians, constraints, and expression
growth; a generalized task interface alone does not establish support for spatial mechanisms.

### R4. Measure before sharing model storage or changing observation cadence

[`Model`](core/src/model.rs#L25) and [`Tape`](core/src/tape.rs#L31) combine reusable compiled tables with
parameter-dependent coefficients and mutable evaluation scratch. Slots deep-clone them, and
[`World::bench`](core/src/world.rs) forks another simulation for each measurement. The benchmark includes
fork/setup cost and restarted controller/reference state; it does not measure only an ongoing step.

Measure derivation, fork/setup, controller redesign, integration, snapshot publication, and complete worker
tick time separately on the larger presets with shadows. If cloning or memory is material, separate
read-only compiled data, folded parameter data, and per-slot workspace. Retain independent controller and
reference state and account for future divergence of slot parameters before sharing folded data.

Rendering, equation refresh, plot sampling, physics stepping, and rewind retention have different purposes.
Give each an explicit time basis and reset/model-generation handling; equal frequencies alone do not
justify one shared constant. [`Timeline`](web/src/app/timeline.ts) is an observation history. Its Resume
here operation currently restores primary joint state, restarts controllers/shadows, and keeps current
simulation time; it does not restore a historical checkpoint. Define that behavior before promising exact
replay. Full checkpoints or deterministic command replay are separate features.

### R5. Split rendering and numerical helpers along existing responsibilities

Extract Pen backends, mechanical primitives, diagnostic/free-body calculations, and view-layer management
from [`schematic.ts`](web/src/scene/schematic.ts). Pass the drawing context explicitly to remove its
module-level mutable ink state. Preserve the shared drawing path, force-balance tests, and export parity.

For numerical helpers, document units, scaling, convergence criteria, and regularization where each
algorithm lives. Consolidate a tolerance only when its mathematical meaning is shared. Add telemetry for
[`integration recovery`](core/src/sim.rs#L463), which currently restores the old position, zeroes velocity,
and restarts the energy budget without exposing that event. This helps distinguish a successful experiment
from one interrupted by numerical recovery. Optimize recorder storage or lazy-load page content when
profiling shows a user-visible benefit; the existing bundle warning alone does not establish one.

## Delivery and verification

| Change set | Scope | Completion evidence |
|---|---|---|
| 1 | F1 inheritance and invalidation | Native and real-WASM regressions for targets, tuning, rebuild, synchronization, and timestep changes. |
| 2 | F2 snapshot commit | Deterministic contention tests and both transport paths. |
| 3 | F3 revisions and worker lifecycle | Controlled build-response ordering, mismatched generations, build rejection, and worker failure. |
| 4 | F4 validation | Import/ABI/mutation rejection plus valid idealizations and preservation of the installed state. |
| 5 | F5 experiment lifecycle | Controlled waits/cancellation and browser checks across speed, pause, stage exit, and rebuild. |
| Later | R1–R5 | Choose a bounded change tied to a repeated maintenance cost, measured bottleneck, or requested feature. |

Keep the independent SymPy derivation and existing physics/property/experiment tests as regression gates.
Add independent branched/prismatic/constrained fixtures and capacity-boundary cases where coverage is
missing. Add a focused CI browser job for the transition cases above; the broader crawl remains useful
for interaction coverage. Replace fixed sleeps with observable readiness in new tests. Oracle freshness
already runs through [`test_generated_files_are_up_to_date`](sym/tests/test_derive.py#L65).

Update [`PROJECT.md`](PROJECT.md#L47) to mark superseded production-SymPy and three.js/WebGPU decisions;
its historical entries currently sit beside the runtime-CAS/canvas design. Keep current invariants and
transition policies in maintained documentation, with this review serving as the implementation backlog.
