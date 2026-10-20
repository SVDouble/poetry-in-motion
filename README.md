# Poetry in Motion

**Poetry in Motion** is an in-browser lab for robot dynamics. Robots are simulated in real time on the client, and every
quantity that governs their motion is typeset next to them and updates as they move: the equations of motion,
the inertia, Coriolis and gravity terms, coordinate frames, Jacobians and the control laws.

```
M(q) q̈ + C(q, q̇) q̇ + N(q, q̇) = τ        (Murray, Li & Sastry, eq. 4.24)
```

Demo systems:

| System | What to try |
|---|---|
| **Cart-pole** (pendulum on a rail) | balance it by hand, LQR balancing, energy-based swing-up |
| **Two-link planar arm** (RR) | passive motion, PD, PD + gravity compensation, computed torque; drag the target |

## Architecture

```
sym/   Python + SymPy: derives every system by Lagrange (MLS ch. 4) and generates
       ├─ core/src/generated/*.rs   allocation-free numeric kernels (M, C, N, Ṁ, ∂N/∂q, energy, frames, Jacobians)
       └─ web/src/generated/*.ts    the LaTeX of the same quantities
core/  Rust → WebAssembly: RK4 at 1 kHz, controllers (LQR by a doubling Riccati solver, swing-up,
       PD, computed torque) and a flat f64 snapshot of everything the UI shows
web/   TypeScript + Vite: physics in a Web Worker, snapshots shared through a SharedArrayBuffer (seqlock),
       three.js (WebGPU with WebGL 2 fallback), KaTeX equations whose numbers are patched in place, uPlot charts
```

Design notes:

- **The symbolic layer is the single source of truth.** The Rust kernels and the LaTeX come from the same
  SymPy expressions, so the formula on screen is the code that runs. Tests check the derivation against MLS
  (4.11) and Lemma 4.2, and CI fails if the generated files are stale.
- **The equations are typeset once.** Live numbers are KaTeX `\htmlId` slots whose text nodes are rewritten at
  30 Hz, and only for sections that are open and on screen.
- **Physics never waits for rendering.** The worker steps at a fixed 1 kHz on its own clock. The UI copies the
  latest consistent snapshot every frame without allocating.
- **Plain WebAssembly exports** (`extern "C"`, scalar arguments only), with no `wasm-bindgen` glue. The snapshot
  layout is described by the core itself (`layout_json`), and the UI checks that it matches at start-up.

Performance on a laptop CPU: about 0.25 µs per full step (controller plus 4 RK4 evaluations) natively and about
0.5 µs in WebAssembly, so the core uses well under 0.1 % of a core at 1 kHz. Run `make bench`, or read the live
counter in the header.

## Getting started

Requirements: [uv](https://docs.astral.sh/uv/) (it installs Python 3.15), Rust ≥ 1.90 with the
`wasm32-unknown-unknown` target (`rustup target add wasm32-unknown-unknown`, or `pacman -S rust-wasm`), and
Node ≥ 22.

```sh
make setup   # uv sync + npm ci + git hooks
make dev     # build the wasm core and start Vite on http://localhost:5173
```

| Command | |
|---|---|
| `make gen` | re-derive the equations and regenerate Rust + LaTeX (after editing `sym/src/motion_sym/systems.py`) |
| `make check` | everything CI runs: formatters, linters, type checkers, codegen freshness, all tests |
| `make fmt` | format and auto-fix (rustfmt, ruff, Biome) |
| `make build` | production bundle in `web/dist` |
| `make bench` | native throughput of the simulation step |

Tooling: rustfmt and clippy (pedantic + nursery, warnings are errors), ruff and mypy (strict), Biome and
TypeScript (strict). Tests are cargo test (physics and controllers), pytest (derivation) and Vitest (formatting,
and the real wasm module through its ABI).

### Deploying

The output is static files. `SharedArrayBuffer` needs cross-origin isolation, so the host must send
`Cross-Origin-Opener-Policy: same-origin` and `Cross-Origin-Embedder-Policy: require-corp`. `web/public/_headers`
does this on Cloudflare Pages and Netlify. Without these headers the app still works: it falls back to
`postMessage` snapshots.

`?webgl` in the URL forces the WebGL 2 backend.

## Controls

`1`–`4` switch controller · `P` pause · `R` reset · cart-pole: `← →` push, `Space` knock, `B` drop ·
arm: click/drag target, `A D` / `← →` joint torques, `E` flip elbow.

## Adding a robot

1. Describe its bodies, frames and actuation in `sym/src/motion_sym/systems.py`, then run `make gen`.
2. Register the generated module in `core/src/model.rs` (`SystemId`, defaults) and add a controller if needed.
3. Add a `SystemDef` in `web/src/systems/` with its scene, parameters and any extra equation sections.

Notation follows R. M. Murray, Z. Li, S. S. Sastry, *A Mathematical Introduction to Robotic Manipulation*
(CRC, 1994). The project goals are in [PROJECT.md](PROJECT.md).
