# Architecture review

Reviewed code: `9ecb67d`; refined review: `b8bdb60` (2026-10-22).
Implementation status updated on 2026-10-22.

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
crawl across 16 presets. Those checks do not establish correctness of the transition cases below. The
implementation status below distinguishes the completed fixes from the remaining proposals.

## Implementation status

The five findings below describe the reviewed baseline. They are now addressed by the implementation:

| Finding | Implemented behavior | Regression coverage |
|---|---|---|
| F1 | Persistent settings survive rebuilds; shadows refresh design from inherited targets and preserve the primary's linkage assembly; timestep changes invalidate controller designs. | `core/tests/transitions.rs` |
| F2 | Reads stage data/stats before commit. Shared-memory frames and explicit observations share sequence ordering, including wrap handling. | `web/src/sim/lifecycle.test.ts` |
| F3 | Builds prepare candidates without replacing the world. Revision checks authorize commit; installed documents, acknowledgments, and snapshots carry matching generations. Worker failures settle pending requests. | `web/src/sim/lifecycle.test.ts`, `web/src/sim/validation.test.ts` |
| F4 | Native descriptions, flat codecs, imported documents, and live mutations validate before changing state. Rejected physical overrides restore the UI value. | `core/tests/transitions.rs`, `web/src/model/validation.test.ts`, `web/src/sim/validation.test.ts` |
| F5 | Experiment waits follow simulation time and acknowledge prior commands. Cancellation guards every asynchronous action boundary and synchronously releases the current push before ownership changes. | `web/src/control/runner.test.ts`, `web/src/control/wait.test.ts`, `web/e2e/transitions.py` |

A build is accepted when its prepared result passes the UI revision check. If a later edit occurs while
that accepted commit is in flight, its acknowledgment installs the captured accepted document as the last
successful runtime while the newer draft remains pending. A rejected newer draft leaves that runtime
usable. Rebuilds retain persistent gains/tuning and reset mechanism-relative targets to the new document.

`make e2e-smoke` checks actual workers with shared memory and message fallback, including pausing a push
past its former wall-clock deadline and canceling it on stage exit. CI runs this target with the core job's
WASM artifact. The larger refactoring pass now implements the concrete maintenance work in R1–R5 below.
Conditional optimizations and separate product features remain explicitly listed as follow-ups.

The F1–F5 implementation passed `make check`, production build, all 121 then-current web tests
(including 228 guided-experiment runs), the 16-preset browser crawl with zero errors, and smoke checks
for both snapshot transports. Refactoring-pass verification is recorded below.

## Larger refactoring implementation

| Area | Implemented boundary | Evidence |
|---|---|---|
| R1 | One WASM Core interface; Rust-generated field widths, flag names, required defaults and description limits; checked bounds, overlap, capacities and compatibility; controller reads by registry key. | `make gen-check`, real-WASM layout corruption checks, existing controller/physics/experiment tests. |
| R2 | `DocumentSession` owns drafts, accepted installs, revision checks and scheduled rebuilds. Physical field descriptors separate admissibility, units and presentation ranges; uniform dependencies remain authored-only. | Document ordering/failure/empty-scene tests; field/domain tests; browser transitions. |
| R3 | Plan has separate Configuration, Design and References. Ordered invalidation preserves stronger cache dependencies. Typed frame point/orientation tasks extend the built-ins, with posture/manipulability below them. Constraint-tangent hierarchies serve VMC, impedance and admittance. | Native leakage/progress/affine-constraint tests, conflicting-goal loop tests, real-WASM feasible/infeasible parallel-platform targets, shadow/rebuild task lifetimes and existing physics checks. |
| R4 | Separate cost profiler; shared compiled tape tables, shared replaceable parameter coefficients, private evaluation buffers and shared derivation text. Worker timing prepares its isolated simulation before starting the integration timer. | [Measured clone reduction](BENCHMARKS.md), isolation tests and live-world preservation during benchmarking. |
| R5 | Pen/context definitions, backends, mechanical primitives, force balance and view management are separate modules. Pens carry their colors explicitly. Recovery counts, loop retries and last recovery time are published per slot; scopes and the energy gauge expose them. | Force-balance/export regressions, recovery/reset coverage, and guided experiments reject numerical recoveries. |

WASM ABI version is now 2; worker transport remains version 1 and document encoding remains version 4.
The maintained responsibilities and lifecycle rules are in [ARCHITECTURE.md](ARCHITECTURE.md).

Remaining work is deliberately conditional: authored parameter edits still rederive to preserve history,
initial/reset behavior and uniform-rod dependencies; browser profiling can guide further design/storage
optimization; recorder changes and lazy loading need a demonstrated user-visible benefit. Exact rewind
checkpoints/replay, new UI controls for arbitrary ordered task lists, and spatial/3D support are product
extensions, not consequences of these refactors. The native typed task API is available now.

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

**Evidence.** [`Sim::fork`](core/src/sim.rs) constructs and designs a fresh simulation, then copies
parameters and targets. It omits `plan.tuning` and does not unconditionally invalidate the already computed
plan. [`settle`](core/src/sim.rs) resets references/controller state but only marks the plan dirty in
a particular setpoint case. [`World::rebuild`](core/src/world.rs) preserves only some global settings.

Two WASM probes demonstrate different consequences:

- With motor speed set to 1, positive joint speed 5, and manual input 100, create a passive shadow and step
  once. Primary input is 0; shadow input is 100 because its motor tuning reverted to the default.
- Build a single driven revolute joint with zero gravity and initial/target angle 0. Set its target to
  0.7, create a passive shadow, then publish. Primary target error is approximately 0.7; shadow target
  error is 0. The shadow copied the requested target but retained its original equilibrium cache.

**Minimal fix.** Copy all inherited configuration before computing derived state, or explicitly invalidate
and refresh the plan after copying. Preserve the intended restart of shadow references and controller
memory. Define configuration capture/application helpers for fork, synchronization, and rebuild. Audit
invalidation separately: [`World::set_dt`](core/src/world.rs) changes the integration step without
invalidating discrete LQR/MPC designs, which depend on that step.

**Acceptance.** Set nondefault tuning, parameters, and targets before creating/synchronizing a shadow;
check inherited settings and recomputed equilibrium/gains. Check rebuild retention separately, including
reset/remapping of mechanism-relative targets. Changing `dt` after controller design must trigger redesign.
These fixes can precede a larger `SimulationSettings` extraction.

### F2. Failed snapshot reads can modify the visible state

**Evidence — code path.** [`SimClient.pull`](web/src/sim/client.ts) writes directly into `snap.data`
and updates visible stats before validating the second sequence number. A failed attempt followed by
exhausted retries leaves those writes visible even though `pull()` returns false. The
[`frame` loop](web/src/main.ts) renders the snapshot regardless. This is a consistency defect in the
failure path; no torn frame was reproduced during the earlier crawl.

**Minimal fix.** Copy data and stats into reusable staging buffers. Validate the sequence, then commit the
observation and `lastSeq` together. Keep the previous accepted observation on failure. Retain bounded
retries so the UI cannot spin indefinitely while the writer is active.

**Acceptance.** Force a writer update between the reader's sequence checks. A failed pull must leave data,
stats, and accepted sequence unchanged; a subsequent successful pull must update them coherently. Exercise
both the shared-memory and message transports. F3 adds a separate requirement: coherent data must also
belong to the installed mechanism.

### F3. Build request identity does not establish document/snapshot identity

**Evidence — code path.** [`edited`](web/src/main.ts) immediately changes `this.mech` and schedules a
build; `buildSeq` increments only when [`rebuild`](web/src/main.ts) starts. A possible interleaving is:

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

Complete the request lifecycle in [`SimClient`](web/src/sim/client.ts): worker errors after startup
currently have no path that settles outstanding build promises. Fail pending requests and expose a terminal
worker state. Coalescing unsent builds is a subsequent optimization; a token cannot interrupt a synchronous
WASM derivation already executing in the worker.

**Acceptance.** Deterministically resolve A during B's debounce window, deliver an obsolete snapshot after
a new build, and fail the current build. None may mix revisions. Worker failure must settle every pending
request exactly once and prevent further sends until recovery. Preserve the FFI's existing behavior of
retaining the previous model when parsing/building fails.

### F4. Validation must cover imports and live mutation

**Evidence.** [`migrate`](web/src/model/description.ts) fills legacy fields but largely trusts values.
[`Desc::parse`](core/src/mech.rs) checks sizes/references, yet converts some invalid tags and indices
and accepts non-finite fields. WASM probes returned build success for a NaN description version and for a
negative mass. NaN is an ABI-level example; it is not a valid literal in a JSON share link. Negative mass
can occur in JSON input.

The same policy must cover [`Sim::set_param`](core/src/sim.rs), which currently checks finiteness but
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

**Evidence.** The browser supplies [`setTimeout` waits](web/src/app/simulate.ts), while
[`verify.test.ts`](web/src/control/verify.test.ts) implements waits by advancing physics. At a different
playback speed, while paused, or under load, an identical action sequence therefore waits for different
amounts of simulated motion. The verification harness also equates milliseconds with step count, retaining
an assumption that `dt = 1 ms`.

[`runExperiment`](web/src/control/runner.ts) checks cancellation between actions, but individual actions
continue after their internal waits. A controlled harness canceled a push during its wait and observed
`setPush(null)`, `PushOn = 0`, and release/done marks afterward. That cleanup can interfere with a newer
push. [`leave`](web/src/app/simulate.ts) and `load` do not invalidate the run token either.

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

## Larger refactoring rationale and remaining work

### R1. Consolidated contracts

Runtime and test harnesses now share the TypeScript `Core` interface. Rust metadata generates browser
field names/widths, flag names, required defaults and mechanism limits; freshness is checked locally and
in CI. Runtime metadata still supplies offsets, defaults and timestep. `parseLayout` rejects incompatible
versions, missing fields, wrong widths, overlap, out-of-bounds fields and inconsistent capacities. Real-WASM
tests exercise corruption and stale-contract rejection rather than trusting generated TypeScript alone.

WASM ABI and worker transport versions are independent of the description format. Controller implementations
read parameter keys through their registry rather than duplicating numeric offsets. This retains the
compact scalar/array ABI without introducing a general schema framework.

### R2. Shared editing semantics and document ownership

`model/fields.ts` centralizes units, physical admissibility, uniform-rod dependencies and explicit control
presentation ranges. Slider minima are not physical validity rules: idealizations such as zero inertia,
zero friction and zero mass remain subject to the core's effective-inertia check. Runtime overrides stay
separate from authored defaults; changing runtime length/mass does not silently rewrite rod inertia.

`DocumentSession` now owns drafts, installed documents, revision capture, scheduled rebuilds and accepted
commit installation. `App` manages their presentation. Tests cover obsolete candidates, newer edits during
accepted commits, failed builds and empty scenes.

Authored parameter-only edits still rederive. A future fast path must also update history, initial/reset
behavior and uniform-rod dependencies; calling `set_param` alone would change those semantics. Mounts and
topology continue to require recompilation.

### R3. Separate Plan lifetimes and generalize task hierarchies

`Configuration`, cached `Design` and live `References` are separate Plan state groups. Ordered invalidation
makes Plan work include controller redesign; controller-only changes cannot clear pending Plan work. The
cache dependency table is maintained in [ARCHITECTURE.md](ARCHITECTURE.md) and alongside the implementation.
Validated simulation setters apply those rules; direct native configuration manipulation is not a substitute
for them when changing cache-dependent inputs.

Typed `TaskGoal` point/orientation targets identify ground, body or tool frames and form an ordered list.
The existing orientation/elbow controls precede additional native tasks; posture and manipulability remain
lower preferences. Closed-loop hierarchies now use the constraint tangent metric and an affine curvature
acceleration. Tests check constraint feasibility, higher-priority leakage, lower-level progress, infeasible
goals and task lifetimes. A real-WASM parallel-platform test checks both an independently rotating platform
frame and a piston frame whose orientation is already determined by its held tool point.

Existing hierarchy controls are enabled on constrained mechanisms with remaining degrees of freedom.
A UI for arbitrary additional native tasks remains a feature extension. Spatial/3D support still requires a
separate review of kinematics, Jacobians, constraints and expression growth.

### R4. Measure costs and share reusable model storage

The new profiler separates derivation, model cloning, fork/design, redesign, integration, publication and
a five-slot step/publication batch. [Measurements](BENCHMARKS.md) showed the seven-joint model clone fall
from about 34 µs to below 1 µs after sharing compiled tape tables and derivation text. Parameter coefficients
can be shared until replaced; evaluation scratch and execution state remain private. Model description and
small topology arrays remain owned. This preserves the ability for slot parameters to diverge later.

Worker timing now prepares its isolated primary and initial controller design before starting the step
cost timer. It still measures a restarted copy, not the total live worker tick. Native measurements also
exclude browser/WASM and transport overhead; they do not justify changing observation cadence or establish
a stable throughput improvement. Design work remains a larger fork cost than cloning.

Rendering, equation refresh, plots, stepping and rewind retention retain different time bases. Timeline is
observation history: Resume here restores primary joint state, restarts controllers/shadows and retains
current simulation time. Full checkpoints or deterministic replay remain separate features.

### R5. Rendering boundaries and numerical observability

Pen/context definitions, backends, mechanical primitives, force/moment balance, Figure composition and
screen-layer management now have separate modules. Each Pen carries its palette; active pictures no longer
change module-global inks. Force-balance tests use the pure load calculation boundary directly, and
canvas/SVG/TikZ retain the shared drawing path.

Recovery telemetry reports per-slot recovery count, loop-step retries and the latest recovery time.
Counters reset with a simulation reset; recovery still restores the previous position at rest and restarts
the energy budget. Scope signals and the energy gauge expose these events. Guided verification rejects
recovery during its sampled run. This revealed an unstable four-bar demonstration with deliberately
incomplete Coriolis-off dynamics; constrained demonstrations now restore full dynamics after one simulated
second, preserving the observable energy-budget discrepancy without requiring recovery.

Finite-difference coordinate/time scales and constraint regularization are documented where they are used.
Similar-looking tolerances with different mathematical meanings remain local. Recorder optimization and
lazy loading remain conditional on a measured user-visible benefit; the bundle warning alone is insufficient.

## Delivery and verification

| Change set | Scope | Completion evidence |
|---|---|---|
| 1 | F1 inheritance and invalidation | Native and real-WASM regressions for targets, tuning, rebuild, synchronization, and timestep changes. |
| 2 | F2 snapshot commit | Deterministic contention tests and both transport paths. |
| 3 | F3 revisions and worker lifecycle | Controlled build-response ordering, mismatched generations, build rejection, and worker failure. |
| 4 | F4 validation | Import/ABI/mutation rejection plus valid idealizations and preservation of the installed state. |
| 5 | F5 experiment lifecycle | Controlled waits/cancellation and browser checks across speed, pause, stage exit, and rebuild. |
| Refactoring pass | R1–R5 | Implemented boundaries and evidence are listed above; conditional optimizations remain separate. |

Keep the independent SymPy derivation and existing physics/property/experiment tests as regression gates.
Add independent branched/prismatic/constrained fixtures and capacity-boundary cases where coverage is
missing. The focused CI browser job now covers the transition cases above; the broader crawl remains
useful for interaction coverage. Replace fixed sleeps with observable readiness in new tests. Oracle freshness
already runs through [`test_generated_files_are_up_to_date`](sym/tests/test_derive.py).

Update [`PROJECT.md`](PROJECT.md) to mark superseded production-SymPy and three.js/WebGPU decisions;
its historical entries currently sit beside the runtime-CAS/canvas design. Keep current invariants and
transition policies in maintained documentation, with this review serving as the implementation backlog.

## Refactoring-pass verification (2026-10-22)

- Fresh release WASM build, then `make -o wasm check`: 86 Rust tests, six Python tests and 129 web tests
  pass, including all 228 guided experiment runs with recovery detection. Formatting, lint, types and
  generated-file freshness pass.
- Production bundle passes. The existing JavaScript chunk-size warning remains; no loading-policy change
  is inferred from it.
- Chromium browser crawl: all 16 presets pass with zero errors.
- Browser smoke checks exercise shared-memory and message-fallback transports, paused physical-time waits,
  document transitions and experiment cancellation.

The code was held stable during the final browser crawl; documentation records the implementation and
separates remaining conditional optimizations from delivered refactors.
