# Simulation cost measurements

Measured locally on 2026-10-22 using release Rust (`cargo run --release -p motion-core --example profile`).
These are elapsed averages from one run per revision, not stable performance guarantees. Derivation is
measured once; model cloning 100 times; fork/design and redesign 20 times; integration 5,000 times;
publication 500 times. The five-slot tick advances four fixed steps and publishes all slots 1,000 times.
It measures native engine work; browser/WASM execution and worker transport overhead require separate
browser profiling. It must not be presented as a complete worker-tick measurement.

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

Observation/render/plot/history cadences retain their separate meanings. No recorder optimization or
sampling-frequency change is justified by these measurements.

The worker now calls `bench_prepare` before starting its integration timer. Preparation forks the primary
and initializes controller design; `bench(n)` advances only that isolated copy. This removes setup from
the displayed per-step measurement without claiming it measures live-world publication or transport.
