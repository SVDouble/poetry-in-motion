# Poetry in Motion

**Poetry in Motion** is an in-browser lab for robot dynamics, organised in three stages:

1. **Build** a planar mechanism in a visual editor. It is a graph: a tree of bodies, each hanging from its
   parent (or the ground) by a **revolute**, **prismatic** or **fixed** joint, plus **loop closures** between
   two bodies or a body and the ground: **pins** (four-bar) and **slots** (slider-crank), and **springs and
   dampers** between any two points. Build it in the view:
   - drag a part from the **palette** (bar, slider, cart, weld, spring, damper, spring–damper, pin, slot) onto
     a lit port to attach it there, or onto empty space to fix it to the ground; springs, dampers and loops
     take a second point;
   - click any part to open its **inspector**: shape, joint, drive, and every number with ◀ ▶ arrows (hold to
     repeat, Shift for fine steps, or type a value) — length, mass, centre of mass, inertia, mounting, range,
     initial value, friction, stiffness, rest length, damping, attachment points;
   - drag **○** to rotate or stretch a body (snaps to 15° / 5 cm, Alt for free, Shift to set its initial
     angle) — onto a port of another body, it closes a loop there; drag **◆** to move a joint — onto a port,
     it re-hangs the body there;
   - the side panel holds the same settings as a tree (drag entries to re-hang), Delete removes, Ctrl+Z undoes.

   Presets: cart-pole, two-, three- and seven-joint arm, double pendulum, wall pendulum, branching arm,
   acrobot, cart double pendulum, four-bar linkage, slider-crank, gantry, mass–spring–damper, sprung
   pendulum — and a test rebuilds every one of them from nothing with the builder's own operations.
2. **Derive** its equations of motion step by step, symbolically, with generic coefficients (mᵢ, lᵢ, rᵢ, Iᵢ, βᵢ, g):
   kinematics, centre-of-mass velocities, kinetic and potential energy, the inertia matrix, the Christoffel
   symbols, the Coriolis matrix, gravity and friction, the scalar equations, and the Jacobians.
3. **Simulate** it in real time. The same equations drive the physics and the controllers, and every quantity
   updates live next to the robot. Eleven controllers: passive, PD, PD + gravity, computed torque,
   passivity-based (Slotine–Li), virtual model control / energy shaping, impedance, admittance, LQR, MPC
   (condensed box QP solved by FISTA) and energy swing-up. Each has its own page: parameters first, then
   guided experiments (one click sets up a scenario, switches the plots to what matters and explains what to
   notice), then the idea, the generic derivation, the same derivation for the built mechanism with live
   numbers, stability and limitations. The plots can show dozens of signals (joints, every term of the
   equations of motion, energy and power, task space, per-controller comparisons and each controller's
   internals such as its Lyapunov function, null-space leakage or active MPC constraints). One controller runs as the primary and up to four others as **shadows**: independent
   simulations of the same mechanism under the same pushes, drawn as ghosts and compared in the plots. The
   **Goals** tab sets joint or tool targets and a task hierarchy below the tool: its orientation, an elbow
   point (the planar counterpart of a 7-DOF arm's "arm angle"), then posture and manipulability. Each level
   acts in the dynamically consistent null space of the levels above (`P_k = P_{k−1} − Ĵ#_k Ĵ_k`, damped least
   squares for the command, exact projectors), so a lower goal can never disturb a higher one; the
   seven-joint arm preset has room for all of them. Shift-drag pushes the tool with a spring, Alt-drag places
   the elbow target. Everything works on closed loops too: inverse dynamics solves `B u − Aᵀλ = f`, and
   LQR/MPC live in tangent coordinates of the constraint manifold. Their weights are shaped by the
   mechanism's inertia (Q = diag(Q_x M, Q_ẋ M), R = R Bₐᵀ M Bₐ), so a setting means the same response on a
   light pendulum and a heavy arm; on a fully actuated mechanism they are designed at a pose at most a short
   lead ahead of the state while it travels to its target (a reference governor), so they only ever correct
   small deviations — from wherever it starts, and towards the inverse kinematics of a tool target. Every number that
   shapes behaviour beyond the physics — null-space damping, the damped-least-squares bands, the priority
   fade, how far inside the workspace an unreachable target is aimed, the inverse-kinematics step and
   damping, the push limit, the end stops, the Baumgarte rate, the motors' no-load speed (their torque–speed
   line) — is a setting (Goals tab, with defaults to return to), the same for every mechanism.

```
M(q) q̈ + C(q, q̇) q̇ + N(q, q̇) = τ        (Murray, Li & Sastry, eq. 4.24)
```

Nothing is special-cased: the cart-pole and the arm are presets of the builder, and every controller works from
M, C, N, V, B and the tool Jacobian of whatever was built. Guided experiments are offered by structure
(underactuated, redundant, loops, …) and size their kicks and limits from the mechanism itself; the gallery
shelves are read off the structure too. Closed loops are handled as holonomic constraints
φ(q) = 0, derived symbolically like everything else (MLS ch. 6): `M q̈ + C q̇ + N + Aᵀλ = τ`, `A = ∂φ/∂q`.

Seeing the dynamics: the View menu draws velocities of the centres of mass, forces (weights, springs and
dampers, loop forces from the multipliers λ, the push), and a **free-body diagram** of any clicked body — cut
free, with its joint reactions from Newton's law for the subtree it carries (split into drive and normal
force at a prismatic joint), the reactions it passes on, its joint moments and m a as the resultant. Arrows
use fixed round scales per mechanism, listed in a key, so they compare across time. Per-joint **torque bars** show which of M q̈, C q̇, N makes up τ; an
**energy gauge** shows what is stored, supplied and lost. Pausing shows a **timeline**: scrub back through
the last 20 s (the plots mark the moment and read its values), step frame by frame, resume from any moment. Pointing at a formula in the derivation lights up
what it is about in the figure (a frame, a centre of mass with its Jacobian columns ∂p/∂q_j, a joint's row
of the equations of motion, gravity, a spring, a loop, the tool); hovering a
joint or a centre of mass shows its numbers.

The whole app is drawn like a mechanics textbook: ink on paper in Computer Modern, and the mechanism as a
figure in the symbols of such texts — bars, hinge circles (a second ring for a driven joint), pin supports on
hatched ground, carts on rails, the centre-of-mass symbol, joint angles as arcs from dash-dotted reference
lines, coordinate frames {s}, {i}, {t}, torques as arcs, the push as a zigzag spring. The current pose exports
as SVG or as TikZ code that compiles as is.

## Architecture

```
core/  Rust → WebAssembly
       ├─ cas/      a small computer-algebra system for planar mechanics (exact, canonical, LaTeX printer)
       ├─ mech.rs   generic mechanism graph (tree + loop closures) + Lagrangian derivation (MLS ch. 4, 6)
       ├─ doc.rs    the derivation as LaTeX/JSON for the step-by-step view
       ├─ tape.rs   symbolic expressions compiled into allocation-free numeric tapes
       ├─ dynamics.rs  M, C, N; constraints; KKT forward dynamics; projection; constrained inverse dynamics
       ├─ plan.rs   what all controllers share: targets, references, IK, equilibrium, linearisation, push,
       │            secondary goals
       ├─ control/  one struct per controller behind a `Controller` trait; parameters and telemetry declared
       │            in a registry (`SPECS`) that the browser renders generically
       ├─ qp.rs     box-constrained QP (FISTA) for MPC
       ├─ sim.rs    one simulation: RK4 at 1 kHz, KKT solve with Baumgarte stabilisation, end stops, snapshot
       └─ world.rs  the primary and up to four shadow simulations, per-controller parameters
web/   TypeScript + Vite
       ├─ app/      the shell (main.ts), the Simulate stage, controller dock/menu/tab, Goals and Model tabs
       ├─ control/  controller pages (theory) and labs (watch lists, experiments); the slot store
       ├─ scope/    signal catalogue (one id per plottable quantity), recorder, plots and presets
       ├─ builder/ eq/ scene/ sim/   editor, derivation and live equations (KaTeX patched in place),
       │            the figure (one drawing routine for a 2D canvas, SVG and TikZ; the ground is drawn once,
       │            the rest only when it moved), the physics Web Worker (SharedArrayBuffer)
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

Performance (`make bench`, native, best of 7): about 0.5 µs per controlled RK4 step for the cart-pole, 1.7 µs for a
4-link chain and 4.3 µs for a 6-link chain, all far faster than real time at 1 kHz. After the parameters are folded
in, terms that share their state monomial and harmonic are merged (the 6-link M, C, N shrink from 1959 to 1036
terms), and every angle's sine and cosine come from one `sin_cos`.

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
| `make fuzz` | property tests on 1000 random mechanisms (`FUZZ_CASES=n`) |
| `make e2e` | crawl the running app in a headless browser (`CHROMIUM=/path/to/chromium`) |

Tooling: rustfmt and clippy (pedantic + nursery, warnings are errors), ruff and mypy (strict), Biome and
TypeScript (strict). Tests:

- cargo test: physics and controllers on the presets, and property tests on random mechanisms (the derived
  equations against Euler–Lagrange computed numerically from the energy alone, energy and budget conservation,
  every controller under shadows, pushes and parameter changes, convergence of the tracking controllers);
- pytest: the derivation against SymPy;
- Vitest: formatting, the real wasm module through its ABI, and every guided experiment on every mechanism
  that offers it, checked against its claim;
- `make e2e`: a headless-browser crawl of every stage, tab, controller, experiment and plot operation.

### Deploying

The output is static files. `SharedArrayBuffer` needs cross-origin isolation, so the host must send
`Cross-Origin-Opener-Policy: same-origin` and `Cross-Origin-Embedder-Policy: require-corp`. `web/public/_headers`
does this on Cloudflare Pages and Netlify. Without these headers the app still works: it falls back to
`postMessage` snapshots.

## Controls

Simulate (the Keys toggle shows them on the viewport): `1`–`9` choose the primary controller ·
`Shift`+`1`–`9` toggle it as a shadow · `P` pause ·
`R` reset · `Space` kick · actuated joints: `← →`, `A D`, `J L`, … · drag: tool target · `Shift`+drag: push the
tool · arms with an actuated rail: click to set its target.

## Adding joint types or controllers

- Joint kinematics live in `core/src/mech.rs` (`derive`). Everything downstream, including M, C, N, the tapes,
  the LaTeX and the controllers, is generic.
- A controller is a struct implementing `Controller` in `core/src/control/`, plus a `Kind` and a `Spec`
  (parameters, telemetry) in `control/mod.rs`. Its page is a function in `web/src/control/` registered in
  `pages.ts`, its experiments live in `lab.ts`; sliders, slots, plots and its telemetry signals need no
  changes (a test checks that every signal and parameter an experiment names exists).

Notation follows R. M. Murray, Z. Li, S. S. Sastry, *A Mathematical Introduction to Robotic Manipulation*
(CRC, 1994). The project goals are in [PROJECT.md](PROJECT.md).

## License

Licensed under the [Apache License, Version 2.0](LICENSE). See [NOTICE](NOTICE).
