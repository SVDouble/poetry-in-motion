/**
 * The Equations tab: the derived equations of the primary simulation with their live numeric values.
 * (Each controller has its own page, see `control/`.)
 */
import type { Derivation } from '../model/derivation';
import type { Snapshot } from '../sim/layout';
import type { SectionSpec } from './panel';
import { texMat } from './tex';

export const C_M = 't-m';
export const C_C = 't-c';
export const C_N = 't-n';
export const C_TAU = 't-tau';

const cls = (c: string, s: string) => String.raw`\htmlClass{${c}}{${s}}`;
const col = (xs: string[]) => texMat(xs.map((x) => [x]));

export interface LiveContext {
  d: Derivation;
  snap: Snapshot;
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

export function liveSections({ d, snap }: LiveContext): SectionSpec[] {
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
        b.text(
          'Integrated since the last reset (with the same RK4 weights as the motion), every joule is accounted for. Friction is part of $N(q,\\dot q)$: with $\\beta > 0$ a passive mechanism loses energy — that is physics, not a numerical leak.',
        );
        const w = (i: number) => b.live.num(() => snap.v('WORK', i));
        b.math(
          String.raw`E - E_0 = \underbrace{${w(0)}}_{\text{inputs}} + \underbrace{${w(1)}}_{\text{push, kicks}} + \underbrace{${w(2)}}_{\text{end stops}} - \underbrace{${w(3)}}_{\text{friction}}${d.closures.length ? String.raw` + \underbrace{${w(4)}}_{\text{loops}}` : ''}`,
        );
        b.math(String.raw`\text{residual} = ${b.live.num(() => snap.v('BUDGET'))}\,\mathrm{J}`, {
          caption: 'integration error (or non-physical what-ifs: switch Coriolis off and watch it grow)',
        });
      },
    },
    {
      id: 'l-m',
      title: 'Inertia matrix',
      cite: 'MLS Lemma 4.2',
      build(b) {
        b.math(String.raw`M(q) = ${texMat(d.M)}`);
        b.math(String.raw`M(q) = ${b.live.mat(n, n, (i, j) => snap.m('M', i, j), undefined, `lv ${C_M}`)}`, {
          focus: 'all',
        });
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
          { focus: 'velocities' },
        );
        b.math(
          String.raw`\max_{ij}\left|(\dot M - 2C) + (\dot M - 2C)^{T}\right| = ${b.live.num(() => snap.v('SKEW'))}`,
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
          { focus: 'gravity' },
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
            { focus: `frame:${k}` },
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
          String.raw`\varphi(q) = ${col(d.phi)} = 0,\qquad \max_i |\varphi_i| = ${b.live.num(() => snap.v('CONSTRAINT'))}\,\mathrm{m}`,
        );
        b.math(
          String.raw`\lambda = ${b.live.vec(m, (i) => snap.v('LAMBDA', i))},\qquad A^{T}\lambda = ${b.live.vec(n, (i) => snap.v('AT_LAMBDA', i))}`,
          { caption: 'constraint forces keeping the loop closed (pin reactions)', focus: 'loop:0' },
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
        b.math(String.raw`p_{st} = ${col(tool.p)} = ${b.live.vec(2, (i) => snap.v('TOOL_P', i))}`, {
          focus: 'tool',
        });
        b.math(
          String.raw`\frac{\partial p_{st}}{\partial q} = ${b.live.mat(2, n, (i, j) => snap.m('JP', i, j))},\quad
          J^{s}_{st} = ${b.live.mat(3, n, (i, j) => snap.m('JS', i, j))}`,
        );
      },
    });
  }

  return sections;
}
