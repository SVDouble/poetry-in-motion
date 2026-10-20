# Poetry in Motion

**Poetry in Motion** is an in-browser lab for robot dynamics, organised in three stages:

1. **Build** a planar mechanism in a visual editor. It is a graph: a tree of bodies, each hanging from its
   parent (or the ground) by a **revolute**, **prismatic** or **fixed** joint, plus **loop-closing pins**
   between two bodies or a body and the ground (four-bar, slider-crank, …). Each joint has its own settings:
   its port along the parent (s·l, symbolic), mounting angle, slide axis and range, actuated or passive,
   damping, and initial and target values. Edit in the panel, or directly in the view:
   - click a **+ port** to attach a child or pin a loop there;
   - drag **◆** to move a joint; drag **○** to rotate or stretch a body (snaps to 15° / 5 cm, Alt for free);
   - drag tree entries to re-hang bodies, use × to remove one, and Ctrl+Z to undo.

   Presets: cart-pole, two- and three-link arm, double pendulum, wall pendulum, branching arm, acrobot,
   cart double pendulum, four-bar linkage, slider-crank, gantry.
2. **Derive** its equations of motion step by step, symbolically, with generic coefficients (mᵢ, lᵢ, rᵢ, Iᵢ, βᵢ, g):
   kinematics, centre-of-mass velocities, kinetic and potential energy, the inertia matrix, the Christoffel
   symbols, the Coriolis matrix, gravity and friction, the scalar equations, and the Jacobians.
3. **Simulate** it in real time. The same equations drive the physics and the controllers, and every quantity
   updates live next to the robot. The controllers (PD, PD + gravity, computed torque, LQR, energy-based
   swing-up) are derived from the model and explained in the panel.

```
M(q) q̈ + C(q, q̇) q̇ + N(q, q̇) = τ        (Murray, Li & Sastry, eq. 4.24)
```

Nothing is special-cased: the cart-pole and the arm are presets of the builder, and every controller works from
M, C, N, V, B and the tool Jacobian of whatever was built. Closed loops are handled as holonomic constraints
φ(q) = 0, derived symbolically like everything else (MLS ch. 6): `M q̈ + C q̇ + N + Aᵀλ = τ`, `A = ∂φ/∂q`.

## Architecture

```
core/  Rust → WebAssembly
       ├─ cas/      a small computer-algebra system for planar mechanics (exact, canonical, LaTeX printer)
       ├─ mech.rs   generic mechanism graph (tree + loop closures) + Lagrangian derivation (MLS ch. 4, 6)
       ├─ doc.rs    the derivation as LaTeX/JSON for the step-by-step view
       ├─ tape.rs   symbolic expressions compiled into allocation-free numeric tapes
       ├─ sim.rs    RK4 at 1 kHz, KKT solve with Baumgarte stabilisation for loops, end stops, snapshots
       └─ control/  generic PD, PD + gravity, computed torque (+ IK), LQR (doubling Riccati), swing-up (PFL)
web/   TypeScript + Vite: builder, step-by-step derivation, physics in a Web Worker with SharedArrayBuffer
       snapshots, three.js (WebGPU with WebGL 2 fallback), KaTeX equations patched in place, uPlot charts
sym/   Python + SymPy: an independent derivation of the reference systems, generated into Rust kernels that
       the tests use as an oracle for the in-browser CAS
```

Design notes:

- **Symbolic, in the browser, in milliseconds.** A planar open chain only ever produces terms of the form
  *c · (parameter monomial) · (prismatic coordinates and velocities) · cos/sin(k·θ)*. Expanding every product of
  trigonometric factors into sums gives a *Fourier canonical form*: each expression has exactly one
  representation, so no `simplify()` is needed, and identities like c₁c₁₂ + s₁s₁₂ = c₂ hold by construction.
  Deriving the cart-pole takes 0.3 ms, a 6-link chain 7 ms.
- **The formula on screen is the code that runs.** The derived expressions are compiled into numeric tapes for
  the simulator. The tests check them against the independent SymPy derivation on random parameters and states,
  and check MLS (4.11), Lemma 4.2 (Ṁ − 2C skew-symmetric) and energy conservation.
- **Coefficients stay symbolic.** Changing a mass or a length in the simulation changes numbers, not equations,
  so nothing is re-derived.
- **The equations are typeset once.** Live numbers are KaTeX `\htmlId` slots whose text nodes are rewritten at
  30 Hz, and only for sections that are open and on screen.
- **Physics never waits for rendering.** The worker steps at a fixed 1 kHz on its own clock. The UI copies the
  latest consistent snapshot every frame without allocating.
- **Plain WebAssembly exports** (`extern "C"`, scalar arguments only), with no `wasm-bindgen` glue. The
  mechanism goes in as a flat `f64` array, and the derivation comes out as JSON.
- **Shareable.** The mechanism is encoded in the URL, so a built robot can be sent as a link.

Performance (`make bench`, native): 0.4 µs per controlled RK4 step for the cart-pole, 2 µs for a 4-link chain and
9 µs for a 6-link chain, all far faster than real time at 1 kHz.

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
| `make gen` | regenerate the SymPy oracle kernels in `core/tests/oracle/` |
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

Simulate: `1`–`6` switch controller · `P` pause · `R` reset · `Space` kick the passive joints · actuated joints:
`← →`, `A D`, `J L`, … · fully actuated arms: drag to move the tool target (inverse kinematics) · arms with
an actuated rail: click to set its target.

## Adding joint types or controllers

- Joint kinematics live in `core/src/mech.rs` (`derive`). Everything downstream, including M, C, N, the tapes,
  the LaTeX and the controllers, is generic.
- Controllers live in `core/src/control/mod.rs`. Their explanations live in `web/src/eq/live.ts`.

Notation follows R. M. Murray, Z. Li, S. S. Sastry, *A Mathematical Introduction to Robotic Manipulation*
(CRC, 1994). The project goals are in [PROJECT.md](PROJECT.md).

## License

Licensed under the [Apache License, Version 2.0](LICENSE). See [NOTICE](NOTICE).
