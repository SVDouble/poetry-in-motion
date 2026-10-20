/**
 * Simulation panel: the derived equations with their live numeric values, and the controllers —
 * each control law derived from the equations of motion, with the active one highlighted.
 */
import type { Derivation } from '../model/derivation';
import type { Snapshot } from '../sim/layout';
import type { SectionSpec } from './panel';
import { sci, texMat } from './tex';

export const MODE = { PASSIVE: 0, PD: 1, PD_GRAVITY: 2, COMPUTED_TORQUE: 3, LQR: 4, SWING_UP: 5 } as const;
export const PHASE = { IDLE: 0, SWING: 1, BALANCE: 2 } as const;

export const C_M = 't-m';
export const C_C = 't-c';
export const C_N = 't-n';
export const C_TAU = 't-tau';

const cls = (c: string, s: string) => String.raw`\htmlClass{${c}}{${s}}`;
const col = (xs: string[]) => texMat(xs.map((x) => [x]));

export interface LiveContext {
  d: Derivation;
  snap: Snapshot;
  mode: () => number;
  param: (index: number) => number;
}

/** Homogeneous transform entry of the pose (x, y, φ). */
function hom(x: number, y: number, phi: number, i: number, j: number): number {
  const c = Math.cos(phi);
  const s = Math.sin(phi);
  const m = [
    [c, -s, x],
    [s, c, y],
    [0, 0, 1],
  ];
  return m[i]?.[j] ?? 0;
}

export function liveSections({ d, snap, mode, param }: LiveContext): SectionSpec[] {
  const n = d.n;
  const nu = d.nu;
  const q = col(d.coords.map((c) => c.tex));
  const sections: SectionSpec[] = [
    {
      id: 'l-state',
      title: 'State',
      build(b) {
        b.math(
          String.raw`q = ${q} = ${b.live.vec(n, (i) => snap.v('Q', i))},\quad
          \dot q = ${b.live.vec(n, (i) => snap.v('QD', i))},\quad
          \ddot q = ${b.live.vec(n, (i) => snap.v('QDD', i))}`,
        );
      },
    },
    {
      id: 'l-eom',
      title: 'Equations of motion',
      cite: 'MLS (4.24)',
      build(b) {
        b.math(
          String.raw`${cls(C_M, String.raw`M(q)\,\ddot q`)} + ${cls(C_C, String.raw`C(q,\dot q)\,\dot q`)} + ${cls(C_N, String.raw`N(q,\dot q)`)} = ${cls(C_TAU, String.raw`\tau`)}`,
        );
        b.text(
          'Each row balances generalised forces: <span class="k t-m">inertial</span>, <span class="k t-c">Coriolis &amp; centrifugal</span>, <span class="k t-n">gravity &amp; friction</span> and <span class="k t-tau">applied</span>.',
        );
        const loops = d.closures.length > 0;
        if (loops) {
          b.text('With loop closures the constraint forces $A^T\\lambda$ join the balance:');
        }
        b.math(
          String.raw`${b.live.vec(n, (i) => snap.v('MQDD', i), undefined, `lv ${C_M}`)}
          + ${b.live.vec(n, (i) => snap.v('CQD', i), undefined, `lv ${C_C}`)}
          + ${b.live.vec(n, (i) => snap.v('N', i), undefined, `lv ${C_N}`)}
          ${loops ? String.raw`+ ${b.live.vec(n, (i) => snap.v('AT_LAMBDA', i))}` : ''}
          = ${b.live.vec(n, (i) => snap.v('TAU', i), undefined, `lv ${C_TAU}`)}`,
        );
        if (nu > 0) {
          b.math(
            String.raw`\tau = B\,u + \tau_{\text{ext}},\quad u = ${b.live.vec(nu, (i) => snap.v('U', i))},\quad
            \tau_{\text{ext}} = ${b.live.vec(n, (i) => snap.v('TAU_EXT', i))}`,
          );
        }
      },
    },
    {
      id: 'l-energy',
      title: 'Energy',
      cite: 'MLS §4.2',
      build(b) {
        b.math(
          String.raw`T = ${b.live.num(() => snap.v('ENERGY', 0))}\,\mathrm{J},\quad
          V = ${b.live.num(() => snap.v('ENERGY', 1))}\,\mathrm{J},\quad
          E = T + V = ${b.live.num(() => snap.v('ENERGY', 2))}\,\mathrm{J}`,
        );
        b.text(
          'Because $\\dot M - 2C$ is skew-symmetric, the energy changes only through work done on the system:',
        );
        b.math(
          String.raw`\dot E = \dot q^{T}\tau - \dot q^{T}\operatorname{diag}(\beta)\,\dot q
          = ${b.live.num(() => snap.v('POWER'))} - ${b.live.num(() => snap.v('DISSIPATION'))}
          = ${b.live.num(() => snap.v('POWER') - snap.v('DISSIPATION'))}\,\mathrm{W}`,
        );
      },
    },
    {
      id: 'l-m',
      title: 'Inertia matrix',
      cite: 'MLS Lemma 4.2',
      build(b) {
        b.math(String.raw`M(q) = ${texMat(d.M)}`);
        b.math(String.raw`M(q) = ${b.live.mat(n, n, (i, j) => snap.m('M', i, j), undefined, `lv ${C_M}`)}`);
        b.math(
          String.raw`\det M = ${b.live.num(() => snap.v('DET_M'))},\quad
          \lambda(M) = \{${Array.from({ length: n }, (_, i) => b.live.num(() => snap.v('EIG_M', i))).join(',\\ ')}\}`,
        );
      },
    },
    {
      id: 'l-c',
      title: 'Coriolis matrix',
      cite: 'MLS (4.23)',
      build(b) {
        b.math(String.raw`C(q,\dot q) = ${texMat(d.C)}`);
        b.math(
          String.raw`C(q,\dot q) = ${b.live.mat(n, n, (i, j) => snap.m('C', i, j), undefined, `lv ${C_C}`)}`,
        );
        b.math(
          String.raw`\max_{ij}\left|(\dot M - 2C) + (\dot M - 2C)^{T}\right| = ${b.live.num(() => snap.v('SKEW'), sci)}`,
          { caption: 'passivity (MLS Lemma 4.2)' },
        );
      },
    },
    {
      id: 'l-n',
      title: 'Gravity & friction',
      build(b) {
        b.math(
          String.raw`N(q,\dot q) = ${col(d.N)} = ${b.live.vec(n, (i) => snap.v('N', i), undefined, `lv ${C_N}`)}`,
        );
      },
    },
    {
      id: 'l-frames',
      title: 'Frames',
      cite: 'MLS §2.3',
      open: false,
      build(b) {
        d.bodies.forEach((body, k) => {
          b.math(
            String.raw`g_{s${body.label}} = ${texMat(body.gWorld)} = ${b.live.mat(3, 3, (i, j) => {
              const [x, y, p] = snap.frame(k + 1);
              return hom(x, y, p, i, j);
            })}`,
          );
        });
      },
    },
  ];

  if (d.closures.length) {
    const m = d.phi.length;
    sections.push({
      id: 'l-loops',
      title: 'Loop closures',
      cite: 'MLS §6.1',
      build(b) {
        b.math(
          String.raw`\varphi(q) = ${col(d.phi)} = 0,\qquad \max_i |\varphi_i| = ${b.live.num(() => snap.v('CONSTRAINT'), sci)}\,\mathrm{m}`,
        );
        b.math(
          String.raw`\lambda = ${b.live.vec(m, (i) => snap.v('LAMBDA', i))},\qquad A^{T}\lambda = ${b.live.vec(n, (i) => snap.v('AT_LAMBDA', i))}`,
          { caption: 'constraint forces keeping the loop closed (pin reactions)' },
        );
      },
    });
  }

  const tool = d.tool;
  if (tool) {
    sections.push({
      id: 'l-tool',
      title: 'Tool & Jacobians',
      cite: 'MLS §3.4',
      open: false,
      build(b) {
        b.math(String.raw`p_{st} = ${col(tool.p)} = ${b.live.vec(2, (i) => snap.v('TOOL_P', i))}`);
        b.math(
          String.raw`\frac{\partial p_{st}}{\partial q} = ${b.live.mat(2, n, (i, j) => snap.m('JP', i, j))},\quad
          J^{s}_{st} = ${b.live.mat(3, n, (i, j) => snap.m('JS', i, j))}`,
        );
      },
    });
  }

  sections.push(controlSection({ d, snap, mode, param }));
  return sections;
}

function controlSection({ d, snap, mode }: LiveContext): SectionSpec {
  const n = d.n;
  const nu = d.nu;
  const full = nu === n;
  const under = nu > 0 && nu < n;
  return {
    id: 'l-control',
    title: 'Control',
    cite: 'MLS §4.5',
    build(b) {
      if (nu === 0) {
        b.text(
          'No joint is actuated: nothing can be controlled. Watch the passive dynamics (and the energy).',
        );
        return;
      }
      const e = (i: number) => snap.v('Q_REF', i) - snap.v('Q', i);
      if (d.closures.length) {
        b.text(
          'This mechanism contains closed loops. Joint-space PD (with or without gravity feed-forward) acts directly on the actuated joints; the model-based laws below (computed torque, LQR, swing-up) are written for the open-chain equations and would need the constrained model, so they are not offered here.',
        );
      }
      b.text(
        'Every law below is computed from the same symbolic model: $M$, $C$, $N$ and $\\partial V / \\partial q$ are evaluated at the current state.',
      );
      b.math(
        String.raw`q^{*} = ${b.live.vec(n, (i) => snap.v('Q_TARGET', i))},\quad
        q_d = ${b.live.vec(n, (i) => snap.v('Q_REF', i))},\quad
        e = q_d - q = ${b.live.vec(n, e)}`,
        { caption: 'target, smoothed reference (critically damped filter) and tracking error' },
      );

      b.math(
        String.raw`\textbf{PD:}\quad u = K_p\,e + K_v\,\dot e,\quad K_p = \omega_{pd}^2 \operatorname{diag} M(q^{*}),\ K_v = 2\omega_{pd} \operatorname{diag} M(q^{*})`,
        {
          active: () => mode() === MODE.PD,
        },
      );
      b.math(
        String.raw`K_p = \operatorname{diag}${b.live.vec(n, (i) => snap.v('KP', i))},\quad K_v = \operatorname{diag}${b.live.vec(n, (i) => snap.v('KD', i))}`,
        { active: () => mode() === MODE.PD || mode() === MODE.PD_GRAVITY },
      );
      b.text(
        'With gravity, plain PD settles where $K_p e = \\partial V/\\partial q$: a steady-state error. Feeding the gravity term forward removes it:',
      );
      b.math(
        String.raw`\textbf{PD + gravity:}\quad u = K_p\,e + K_v\,\dot e + \frac{\partial V}{\partial q}(q)`,
        {
          active: () => mode() === MODE.PD_GRAVITY,
        },
      );

      if (full) {
        b.text(
          'Computed torque inverts the whole model. Substituting it into $M\\ddot q + C\\dot q + N = \\tau$ and cancelling $M \\succ 0$ leaves linear, decoupled error dynamics that are the same in every configuration:',
        );
        b.math(
          String.raw`\textbf{Computed torque:}\quad \tau = M(q)\big(\ddot q_d + K_v\dot e + K_p e\big) + C(q,\dot q)\,\dot q + N(q,\dot q)`,
          { active: () => mode() === MODE.COMPUTED_TORQUE },
        );
        b.math(
          String.raw`\Rightarrow\ \ddot e + K_v\dot e + K_p e = 0,\quad K_p = \omega_c^2 I,\ K_v = 2\omega_c I,\ \omega_c = ${b.live.num(() => snap.v('CT_WN'))}\,\mathrm{rad/s}`,
          { active: () => mode() === MODE.COMPUTED_TORQUE },
        );
      }

      // LQR
      const lqrActive = () =>
        mode() === MODE.LQR || (mode() === MODE.SWING_UP && snap.v('PHASE') === PHASE.BALANCE);
      b.text(
        'Near an equilibrium $q^{*}$ the motion is approximately linear. With $x = (q - q^{*}, \\dot q)$, linearising $\\ddot q = M^{-1}(Bu - C\\dot q - N)$ gives ($C\\dot q$ is second order and drops out):',
      );
      b.math(
        String.raw`\dot x = A x + B_x (u - u^{*}),\quad A = \begin{bmatrix} 0 & I \\ -M^{-1}\frac{\partial^2 V}{\partial q^2} & -M^{-1}\operatorname{diag}\beta \end{bmatrix}_{q^{*}},\quad B_x = \begin{bmatrix} 0 \\ M^{-1}B \end{bmatrix}_{q^{*}}`,
        { active: lqrActive },
      );
      b.math(
        String.raw`A = ${b.live.mat(2 * n, 2 * n, (i, j) => snap.v('LIN_A', i * 2 * n + j))},\quad B_x = ${b.live.mat(2 * n, nu, (i, j) => snap.v('LIN_B', i * nu + j))}`,
        { active: lqrActive },
      );
      b.text(
        'The gain minimises $\\int_0^\\infty x^T Q x + u^T R u\\,dt$ (discrete Riccati equation at the 1 kHz control rate, solved by doubling). The feed-forward $u^{*}$ holds the equilibrium: $B u^{*} = \\partial V/\\partial q(q^{*})$.',
      );
      b.math(
        String.raw`\textbf{LQR:}\quad u = u^{*} - K x,\quad K = ${b.live.mat(nu, 2 * n, (i, j) => snap.v('LQR_K', i * 2 * n + j))},\quad u^{*} = ${b.live.vec(nu, (i) => snap.v('U_FF', i))}`,
        { active: lqrActive },
      );
      b.math(
        String.raw`\max_{\text{passive } i}\left|\frac{\partial V}{\partial q_i}(q^{*})\right| = ${b.live.num(() => snap.v('EQ_RESIDUAL'), sci)}`,
        {
          caption:
            'q* is an equilibrium only if this vanishes (the passive joints must be balanced by gravity alone)',
        },
      );

      if (under) {
        const swinging = () => mode() === MODE.SWING_UP && snap.v('PHASE') === PHASE.SWING;
        b.text(
          'Swing-up by energy shaping. The passive joints cannot be driven directly, but the actuated ones can pump energy into them. Freezing the actuated velocities, the energy of the passive motion relative to the target is',
        );
        b.math(
          String.raw`E_u = \tfrac12\,\dot q_u^{T} M_{uu}\,\dot q_u + V(q) - V(q^{*}) = ${b.live.num(() => snap.v('E_U'))}\,\mathrm{J}`,
          { active: swinging },
        );
        b.text(
          'An actuated acceleration $v = \\ddot q_a$ feeds the power $-\\dot q_u^T M_{ua} v$ into the passive coordinates. Choosing $v$ along $w = -M_{ua}^T \\dot q_u$ with the sign of $-E_u$ drives $E_u \\to 0$ (the target orbit); a PD term keeps the actuated joints near $q^{*}$:',
        );
        b.math(
          String.raw`v = k_E\,\frac{-E_u}{E_{\text{ref}}}\,\frac{w}{\|M_{ua}\|} - k_c\,(q_a - q_a^{*}) - k_v\,\dot q_a,\qquad E_{\text{ref}} = ${b.live.num(() => snap.v('E_REF'))}\,\mathrm{J}`,
          { active: swinging },
        );
        b.text(
          'Collocated partial feedback linearisation (Spong) turns $v$ into torques: the passive rows give $\\ddot q_u$, the actuated rows give $\\tau_a$ ($h = C\\dot q + N$):',
        );
        b.math(
          String.raw`\ddot q_u = -M_{uu}^{-1}\big(h_u + M_{ua}\,v\big),\qquad \tau_a = M_{aa}\,v + M_{au}\,\ddot q_u + h_a`,
          { active: swinging },
        );
        b.text(
          'Close to $q^{*}$ the LQR takes over (and hands back if the state leaves its region of attraction).',
        );
        b.math(
          String.raw`\text{phase: }${b.live.num(
            () => snap.v('PHASE'),
            (v) => (v === PHASE.SWING ? 'energy pumping' : v === PHASE.BALANCE ? 'LQR catch' : 'idle'),
          )}`,
          { active: () => mode() === MODE.SWING_UP },
        );
      }
    },
  };
}
