# Simulation cost measurements

Measured locally on 2026-10-22 using release Rust (`cargo run --release -p motion-core --example profile`).
These are elapsed averages from one run per revision, not stable performance guarantees. Derivation is
measured once; model cloning 100 times; fork/design and redesign 20 times; integration 5,000 times;
publication 500 times. The five-slot tick advances four fixed steps and publishes all slots 1,000 times.
It measures native engine work; browser/WASM execution and worker transport overhead require separate
browser profiling. It must not be presented as a complete worker-tick measurement.

## Initial storage refactor

These first-pass measurements precede the shadow-design reuse measured below.

| Case | Model clone before → after (µs) | Fork/design before → after (µs) | Integration before → after (µs/step) | Five-slot tick before → after (µs) |
|---|---:|---:|---:|---:|
| arm2 | 3.81 → 0.59 | 54.33 → 55.45 | 1.38 → 1.83 | 29.82 → 31.55 |
| arm7 | 33.87 → 0.74 | 828.82 → 992.21 | 15.30 → 15.09 | 325.04 → 265.14 |
| fourbar | 3.74 → 0.54 | 37.81 → 28.32 | 3.16 → 2.98 | 64.88 → 63.28 |

These runs also span controller-contract changes, so they are not a controlled throughput comparison.
The isolated model-clone reduction supports sharing compiled tapes and derivation text. The smaller timing
changes are noisy and do not establish a throughput improvement. Fork and controller design still do
substantially more work than cloning; this refactor does not remove that work. Each tape shares immutable
symbolic tables and parameter-folded coefficients through `Arc`, while owning its evaluation scratch.
Setting parameters replaces that tape's coefficients, so future per-slot physical divergence is supported.
Small mechanism description/topology arrays remain owned by each Model.

Observation/render/plot/history cadences retain their separate meanings. No scope-recorder optimization or
sampling-frequency change is justified by these measurements.

The worker calls `bench_prepare` before its integration timer. Preparation now clones the ongoing
primary, including controller memory; `bench(n)` advances that isolated copy. Live-world publication,
transport and checkpoint costs are measured separately below.

## Authored edits, shadows and checkpoints

The follow-through run uses the same release native example, expanded to time compatible authored edits
and 600 checkpoint saves, including eviction once the budget fills. All five slots use impedance control.
Compatible edits refold coefficients and update defaults/metadata; they do not compile a new structure.

| Case | Full derivation (ms) | Compatible edit (µs) | Shadow setup before → after (µs) | Checkpoint save (µs) | Estimated retained KiB/checkpoint |
|---|---:|---:|---:|---:|---:|
| arm2 | 0.38 | 12.30 | 47.02 → 0.98 | 13.15 | 118.1 |
| arm7 | 21.50 | 142.29 | 596.06 → 2.08 | 22.83 | 559.9 |
| fourbar | 0.54 | 19.87 | 25.25 → 1.60 | 12.49 | 130.2 |

Shadow setup reuses valid equilibrium/linearization results and folded coefficients; it still starts
independent controller/reference state. Pending invalidation triggers the required work. These short runs
support removing duplicated design work, not a general throughput claim. The corresponding five-slot
four-step/publication averages were 31.10, 246.49 and 80.36 µs. Other desktop activity and code generation
can affect elapsed times; no statistical confidence interval is claimed.

Checkpoint sizes are conservative estimates used for eviction and count some shared data repeatedly.
They are not measured heap sizes. Retention stops at 600 frames or a 64 MiB estimate, whichever comes first;
the seven-joint five-slot case therefore has a shorter rewind window. Immutable tables, coefficients and
optimal-control matrices are shared; controller memory, references, budget, diagnostics, parameters and
evaluation scratch are independent. UI observations use transferred doubles and allocate only as frames
arrive, replacing the upfront 600-buffer allocation. WASM linear-memory capacity can remain allocated
after checkpoints are dropped.

## Rust audit: revised storage accounting

The Rust audit replaced fixed allowances and logical lengths with inline type sizes, vector capacities,
and explicit controller/plan allocations. The same native five-slot impedance fixtures now report:

| Case | Earlier estimate (KiB/checkpoint) | Capacity-based estimate (KiB/checkpoint) |
|---|---:|---:|
| arm2 | 118.1 | 66.6 |
| arm7 | 559.9 | 662.1 |
| fourbar | 130.2 | 79.9 |

Simple cases no longer pay arbitrary padding. The seven-joint case increases because spare coefficient
capacity is now counted. LQR matrices, the LQR inside swing-up, MPC allocations, VMC projector storage and
plan buffers are accounted for explicitly. Shared code and derivation text remain excluded, shared
coefficients/designs are charged per checkpoint, and allocator bookkeeping is not modeled. These are
estimates rather than measured heap sizes; the 64 MiB and 600-frame policies are unchanged. This correction
supports a more faithful eviction estimate, not a throughput claim.

## Actual browser worker ticks

Reproduce with `CHROMIUM=/path/to/chromium make e2e-profile`. The empty browser fixture starts one actual
WASM worker, builds each mechanism, applies one compatible mass edit and assigns impedance to all five
slots. After 500 ms warm-up it measures 750 actual ticks, including periodic isolated benchmark work.
The worker's requested interval is 4 ms; these cases averaged four fixed integration steps per tick.
Each row is one local Chromium run on 2026-10-22, in microseconds unless stated otherwise.

| Case / transport | Integration mean | Publication mean | Transport mean | History mean | Total mean | Total p95 | Total max |
|---|---:|---:|---:|---:|---:|---:|---:|
| arm2 / shared | 82.9 | 18.4 | 9.6 | 21.7 | 139.7 | 280 | 2,125 |
| arm7 / shared | 349.1 | 44.6 | 5.9 | 17.6 | 422.9 | 705 | 2,385 |
| fourbar / shared | 101.6 | 9.6 | 2.8 | 14.2 | 132.3 | 235 | 1,710 |
| arm2 / fallback | 76.1 | 16.9 | 23.9 | 16.7 | 137.1 | 300 | 600 |
| arm7 / fallback | 400.7 | 50.0 | 27.3 | 17.6 | 503.7 | 800 | 2,700 |
| fourbar / fallback | 154.7 | 16.4 | 24.4 | 17.2 | 217.7 | 400 | 1,400 |

Periodic benchmark work averaged 0.3–3.6 µs per tick, with a maximum of 2,100 µs; a peak is a single tick,
not a continuous cost. History includes full checkpoint cloning plus observation/parameter transfer on
sample ticks and averages over *all* ticks. Transport includes writing shared memory or posting fallback
snapshots at their existing cadences. Total also includes tick bookkeeping. This measures synchronous
worker work; event delivery, UI processing, DOM, equation rendering, plots, frame rate, and timer scheduling
latency remain outside it. The fallback clock was quantized to about 100 µs, so short phases often read
zero and rounded aggregates should not be treated as precise speed comparisons.

Worker-reported full-derivation / compatible-edit times were 10.07 / 0.16 ms for arm2, 47.82 / 0.66 ms for
arm7 and 0.59 / 0.05 ms for fourbar with isolation. These exclude module startup; the first case can include
cold engine execution. Candidate preparation excludes the later world commit and frontend rendering.
The JSON output preserves all phase distributions for further investigation.

These measurements do not establish a main-thread rendering, scope-recorder or initial-loading problem.
Observation cadences and loading policy remain unchanged; the bundle warning alone does not justify a
lazy-loading refactor. The production build is 537,692 bytes of WASM and 562.10 kB of main JavaScript
(202.83 kB and 189.24 kB gzip respectively).
