# Poetry in Motion — Project Goals & Requirements

## Vision
A browser-based, highly performant **robot dynamics visualization and simulation platform**,
built as an **educational resource** for undergraduate, graduate and post-graduate students.

The main point is the educational output. Students should see the math that governs a robot
(equations of motion, coordinate frames, kinematics) **rendered as typeset equations and
updating live** while the robot moves in a real simulation.

## Hard requirements
1. **Runs entirely in the browser.** No install and no server-side compute. All simulation and
   math run on the client.
2. **Very fast, both computationally and visually.** The physics loop is decoupled from
   rendering, the frame rate stays smooth, and the UI stays responsive while equations update.
3. **A real simulation, not an animation.** Motion comes from integrating the equations of
   motion, with user input (forces, torques, controllers) in the loop.
4. **Live, LaTeX-quality rendering of the governing equations**, including:
   - Manipulator equation: **τ = M(q) q̈ + C(q, q̇) q̇ + g(q)**, shown symbolically and with
     live numeric values for M, C, g, q, q̇, q̈ and τ
   - Local / body coordinate frames and the transforms between them (e.g. DH or homogeneous
     transforms)
   - Forward kinematics, Jacobians, and energy (kinetic, potential, total)
   - Everything else needed to fully describe the robot's state and dynamics
5. **Static hosting.** The app can be deployed as static files (CDN or pages host).

## Demo scope (v0)
- **Cart-pole (pendulum on a rail):** an underactuated system. The user can try to balance it
  by hand or switch on a controller (e.g. LQR, energy-based swing-up).
- **2-link planar arm (two revolute joints, RR):** joint-torque input, gravity, and
  controllers such as PD or computed-torque.
- For both: a 3D/2D view, coordinate frames drawn on the robot, a live equations panel, and
  plots of state over time.

## Audience & pedagogy
- Undergrad: intuition, energy, and what each term in M, C and g physically means.
- Grad / post-grad: derivation (Lagrange / Newton–Euler), properties such as M ≻ 0 and
  Ṁ − 2C being skew-symmetric, underactuation, and control design.
- Show the symbolic form and the live numeric instance side by side.

## Longer-term direction (not v0)
- A generic engine: load arbitrary serial (and later branched) robots, e.g. from URDF.
- More systems: acrobot, double pendulum, 3-DOF+ arms, mobile robots.
- Students define their own robot and see the equations derived for it.
- Shareable, embeddable scenes (links or iframes for course pages).

## Current decisions (updated 2026-10-22)
- **Notation:** Murray, Li & Sastry (MLS). The equation is written M(q)q̈ + C(q,q̇)q̇ + N(q,q̇) = τ, with N for
  gravity + friction, because g is reserved for rigid transforms g_ab ∈ SE(2/3), as in the book. C comes from
  the Christoffel symbols (MLS 4.23), so Ṁ − 2C is skew-symmetric.
- **Symbolic layer:** the Rust runtime CAS derives each authored planar mechanism in the browser worker.
  Numeric tapes and the displayed derivation come from that same symbolic representation. SymPy (`sym/`)
  supplies independent generated oracle kernels for tests; it is no longer the production derivation path.
- **Compute:** Rust compiled to WebAssembly (`core/`). Plain `extern "C"` exports with scalar arguments and no
  wasm-bindgen; bulk data is read directly from linear memory.
- **Runtime:** physics runs in a Web Worker at a fixed 1 kHz with RK4. Snapshots go to the UI through a
  SharedArrayBuffer seqlock, with `postMessage` as a fallback.
- **Rendering:** planar Canvas 2D, SVG and TikZ share one Figure and Pen interface. The earlier three.js /
  WebGPU proposal was superseded by this implementation. Equations use KaTeX, typeset once, with live
  number slots patched in place. Plots use uPlot. Spatial simulation/rendering remains a separate design task.
- **Tooling:** Make orchestrates the three toolchains. rustfmt + clippy, ruff + mypy and Biome + tsc check the code;
  cargo test, pytest and Vitest test it; GitHub Actions runs CI.

- **Ownership and contracts:** document sessions distinguish drafts from installed models; workers commit
  accepted revisions and tag observations with generations. Rust generates browser catalogs (`make gen`),
  and startup validates the WASM ABI and worker transport independently of document-format versions.
  See [ARCHITECTURE.md](ARCHITECTURE.md) for current invariants and [BENCHMARKS.md](BENCHMARKS.md) for measurements.

## Status
- v0 (2026-10-20): cart-pole and RR arm with hand-derived (SymPy) kernels and live equations.
- v1 (2026-10-20): **Build → Derive → Simulate**. A generic planar builder (a tree of revolute/prismatic joints,
  up to 6 DOF); an in-browser CAS that derives the equations symbolically with generic coefficients and explains
  them step by step; generic controllers (PD, PD + gravity, computed torque with IK, LQR about any equilibrium,
  energy-based swing-up via collocated PFL), each derived in the panel. The SymPy package is now a test oracle.

- v1.1 (2026-10-20): the mechanism is a **graph**. Fixed joints (walls, brackets), joint ports anywhere along
  a parent (s·l, symbolic), and loop-closing pins as holonomic constraints with Lagrange multipliers (KKT
  solve, Baumgarte stabilisation, projection of the initial pose). The builder gains direct manipulation
  (ports, handles, snapping), drag-and-drop re-parenting, per-entry removal, undo/redo and ground anchors.
  The model-based controllers (computed torque, LQR, swing-up) stay limited to trees.

- v1.2 (2026-10-20): every controller works on closed loops (constrained inverse dynamics, exact gravity
  compensation, LQR in tangent coordinates of the constraint manifold, IK with the loops closed, generalised
  partial feedback linearisation); slot closures (point on a line, one constraint); Shift-drag sets θ(0).
  Code: shared constrained-dynamics helpers in the core, an export macro, and the web app split into
  `app/*` modules with table-driven editor fields and shared widgets.

## Next ideas
- Contacts and joint limits for revolute joints; an angle constraint (weld) between two bodies.
- 3D (spatial) mechanisms: twists, the product of exponentials and the body Jacobian in SE(3).
- Guided lessons and exercises (e.g. "make the energy drift by switching off C").
- Trajectory optimisation and other controllers (sliding mode, passivity-based, MPC).
- On-screen controls for actuated joints on touch devices.
