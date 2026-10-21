/** Pages of the optimal controllers about the equilibrium q*: LQR and MPC. */
import type { SectionBuilder } from '../eq/panel';
import { idleNote, type Page, type PageContext } from './page';

/** The linearisation about q* in tangent coordinates (shared by LQR, MPC and the swing-up catch). */
export function linearisation(b: SectionBuilder, c: PageContext): void {
  const { snap, nu, loops } = c;
  const r = snap.v('DOF') || c.dof;
  b.text(
    loops
      ? `Near q* the motion stays on the ${c.dof}-dimensional constraint manifold. With $q = q^{*} + Z\\xi$, $Z$ an orthonormal basis of $\\ker A(q^{*})$, the state is $x = (\\xi, \\dot\\xi)$; the Jacobians of the constrained dynamics are taken numerically from the same model:`
      : 'Near q* the dynamics are approximately linear. With $x = (q - q^{*}, \\dot q)$, linearising $\\ddot q = M^{-1}(Bu - C\\dot q - N)$ gives ($C\\dot q$ is quadratic in $\\dot q$ and drops out):',
  );
  b.math(
    loops
      ? String.raw`\dot x = A_\xi x + B_\xi (u - u^{*}),\qquad \ddot\xi = Z^{T}\ddot q\big(q^{*} + Z\xi,\ Z\dot\xi,\ u\big)`
      : String.raw`\dot x = A x + B_x (u - u^{*}),\quad A = \begin{bmatrix} 0 & I \\ -M^{-1}\frac{\partial^2 V}{\partial q^2} & -M^{-1}\operatorname{diag}\beta \end{bmatrix}_{q^{*}},\quad B_x = \begin{bmatrix} 0 \\ M^{-1}B \end{bmatrix}_{q^{*}}`,
  );
  b.math(
    String.raw`A = ${b.live.mat(2 * r, 2 * r, (i, j) => snap.v('LIN_A', i * 2 * r + j))},\quad B_x = ${b.live.mat(2 * r, nu, (i, j) => snap.v('LIN_B', i * nu + j))}`,
  );
  b.math(
    String.raw`u^{*} = ${b.live.vec(nu, (i) => snap.v('U_FF', i))},\qquad \Big\| ${loops ? String.raw`B u^{*} - A^{T}\lambda^{*}` : 'B u^{*}'} - \frac{\partial V}{\partial q}(q^{*})\Big\|_\infty = ${b.live.num(() => snap.v('EQ_RESIDUAL'))}`,
    { caption: 'feed-forward holding q*; q* is an equilibrium only if the residual vanishes' },
  );
}

export const lqr = (c: PageContext): Page => {
  const r = () => c.snap.v('DOF') || c.dof;
  return {
    lead: 'The linear-quadratic regulator: the optimal linear feedback for the linearised mechanism.',
    cite: 'Anderson & Moore; MLS §4.5',
    idea(b) {
      b.text(
        'Near an equilibrium every smooth system is almost linear. For linear dynamics and a quadratic cost the optimal controller is a constant state feedback, computed once from a Riccati equation. It balances what PD cannot: passive joints are stabilised through their coupling with the actuated ones.',
      );
    },
    generic(b) {
      b.math(
        String.raw`\min_{u(\cdot)}\ \int_0^\infty x^{T}Q\,x + (u - u^{*})^{T}R\,(u - u^{*})\,dt\quad\text{s.t.}\quad \dot x = A x + B(u - u^{*})`,
      );
      b.text(
        'At the 1 kHz control rate the system is discretised exactly (zero-order hold) and the discrete algebraic Riccati equation is solved by the doubling algorithm:',
      );
      b.math(
        String.raw`P = A_d^{T}PA_d - A_d^{T}PB_d\,(R + B_d^{T}PB_d)^{-1}B_d^{T}PA_d + Q,\qquad K = (R + B_d^{T}PB_d)^{-1}B_d^{T}PA_d`,
      );
      b.math(String.raw`u = u^{*} - K\,x,\qquad Q = \operatorname{diag}(Q_x I, Q_{\dot x} I),\ R = R\,I`);
    },
    concrete(b) {
      linearisation(b, c);
      b.math(
        String.raw`K = ${b.live.mat(c.nu, 2 * r(), (i, j) => c.tele('k', i * 2 * r() + j))},\qquad x^{T}Qx + \Delta u^{T}R\Delta u = ${b.live.num(() => c.tele('cost'))}`,
      );
      idleNote(b, c);
    },
    stability(b) {
      b.text(
        'If (A, B) is stabilisable and Q ≻ 0, the closed loop A − BK is exponentially stable, with guaranteed robustness margins (infinite gain margin, 60° phase margin in continuous time). Stability holds only locally for the nonlinear mechanism: the region of attraction may be small (the acrobot’s is tiny).',
      );
    },
    limits(b) {
      b.text(
        'Local only; ignores actuator limits (it simply saturates); requires q* to be an equilibrium that the inputs can hold. Changing Q or R or the target re-solves the Riccati equation.',
      );
    },
  };
};

export const mpc = (c: PageContext): Page => {
  const { nu } = c;
  const steps = () => Math.max(1, Math.round(c.tele('size')) || Math.round(c.p('horizon')));
  return {
    lead: 'Model predictive control: re-plan an optimal input sequence over a finite horizon every Δt, respecting the actuator limits.',
    cite: 'Rawlings, Mayne & Diehl; Beck & Teboulle 2009',
    idea(b) {
      b.text(
        'LQR knows nothing about limits. MPC predicts the next N steps with the linear model, chooses the input sequence that minimises the cost <i>subject to</i> |u| ≤ u<sub>max</sub>, applies the first input and repeats with fresh measurements. Lower the actuator limit to watch it plan around saturation where LQR simply clips.',
      );
    },
    generic(b) {
      b.math(
        String.raw`\min_{\Delta u_0..\Delta u_{N-1}}\ \sum_{k=0}^{N-1}\big(x_k^{T}Qx_k + \Delta u_k^{T}R\,\Delta u_k\big) + x_N^{T}P\,x_N\quad\text{s.t.}\quad x_{k+1} = A_d x_k + B_d\Delta u_k,\ \ |u^{*} + \Delta u_k| \le u_{\max}`,
      );
      b.text('Stacking the predictions eliminates the states (condensed form):');
      b.math(
        String.raw`X = \Phi\,x_0 + \Gamma\,U\quad\Rightarrow\quad \min_{U\in\mathcal B}\ \tfrac12 U^{T}HU + (F x_0)^{T}U,\qquad H = \Gamma^{T}\bar Q\Gamma + \bar R,\ F = \Gamma^{T}\bar Q\Phi`,
      );
      b.text(
        String.raw`$H$ and $F$ are computed once per design; each re-plan only forms the gradient $Fx_0$ and solves a box-constrained QP with FISTA (accelerated projected gradient, step $1/L$ with $L = \lambda_{\max}(H)$), warm-started with the shifted previous plan:`,
      );
      b.math(
        String.raw`U^{+} = \Pi_{\mathcal B}\Big(Y - \tfrac1L(HY + Fx_0)\Big),\quad t^{+} = \tfrac{1 + \sqrt{1 + 4t^2}}{2},\quad Y^{+} = U^{+} + \tfrac{t - 1}{t^{+}}(U^{+} - U)`,
      );
      b.text(
        'The terminal weight P is the LQR Riccati solution: when no limit is active, MPC returns exactly the LQR input.',
      );
    },
    concrete(b) {
      linearisation(b, c);
      b.math(
        String.raw`N = ${b.live.num(
          () => c.tele('size'),
          (v) => String(v),
        )},\ \Delta t = ${b.live.num(() => c.p('step'))}\,\mathrm{s}\ \Rightarrow\ \text{horizon } ${b.live.num(() => steps() * c.p('step'))}\,\mathrm{s},\quad ${nu}N\text{ unknowns}`,
      );
      b.math(
        String.raw`\text{iterations} = ${b.live.num(
          () => c.tele('iterations'),
          (v) => String(v),
        )},\quad \text{active bounds} = ${b.live.num(
          () => c.tele('active'),
          (v) => String(v),
        )},\quad J^{*} = ${b.live.num(() => c.tele('cost'))}`,
      );
      const shown = 8;
      b.math(
        String.raw`u_{0..${shown - 1}} = ${b.live.mat(1, shown, (_i, k) => c.tele('plan', k))},\quad \|\xi_{1..${shown}}\| = ${b.live.mat(1, shown, (_i, k) => c.tele('predicted', k))}`,
        { caption: 'the first planned inputs (first actuator) and the predicted distance from q*' },
      );
      idleNote(b, c);
    },
    stability(b) {
      b.text(
        'With the LQR terminal cost the MPC value function is a Lyapunov function near q* (Mayne et al. 2000): stability is inherited from LQR wherever the constraints are inactive at the end of the horizon. Infeasible situations cannot occur because only inputs are constrained.',
      );
    },
    limits(b) {
      b.text(
        'Linear model: like LQR it is local. A finite iteration budget gives a suboptimal (but feasible) plan. Long horizons with fine steps cost N² memory and time per re-plan — the core caps the problem at 180 unknowns.',
      );
    },
  };
};
