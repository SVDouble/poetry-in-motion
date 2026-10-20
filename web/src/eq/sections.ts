/**
 * Sections common to every system, built from the generated symbolic model (`src/generated`) and the
 * live snapshot. Notation: Murray, Li & Sastry, "A Mathematical Introduction to Robotic Manipulation".
 */
import type { SymbolicModel } from '../generated';
import type { Snapshot } from '../sim/layout';
import type { SectionSpec } from './panel';
import { sci, texMat } from './tex';

export interface SectionContext {
  model: SymbolicModel;
  snap: Snapshot;
  /** Symbol of the generalised coordinates: `q` or `\theta`. */
  q: string;
  /** Current value of a model parameter by id. */
  param: (id: string) => number;
}

export const C_M = 't-m';
export const C_C = 't-c';
export const C_N = 't-n';
export const C_TAU = 't-tau';

const cls = (c: string, s: string) => String.raw`\htmlClass{${c}}{${s}}`;

/** Planar homogeneous transform entries of (x, y, φ). */
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

export function commonSections(ctx: SectionContext): SectionSpec[] {
  const { model, snap, q } = ctx;
  const n = model.nq;
  const qd = String.raw`\dot{${q}}`;
  const qdd = String.raw`\ddot{${q}}`;
  const coords = texMat(model.q.map((s) => [s]));
  const Mq = String.raw`M(${q})`;
  const Cq = String.raw`C(${q},${qd})`;
  const Nq = String.raw`N(${q},${qd})`;

  const frameNames = ['s', ...model.frames.map((f) => f.name)];
  const pose = (name: string): [number, number, number] => {
    const k = frameNames.indexOf(name);
    return [snap.v('FRAMES', 3 * k), snap.v('FRAMES', 3 * k + 1), snap.v('FRAMES', 3 * k + 2)];
  };
  /** g_{parent,child} = g_{s,parent}⁻¹ g_{s,child} */
  const localPose = (parent: string, child: string): [number, number, number] => {
    const [px, py, pp] = pose(parent);
    const [cx, cy, cp] = pose(child);
    const dx = cx - px;
    const dy = cy - py;
    const c = Math.cos(pp);
    const s = Math.sin(pp);
    return [c * dx + s * dy, -s * dx + c * dy, cp - pp];
  };

  const sections: SectionSpec[] = [
    {
      id: 'state',
      title: 'Configuration & state',
      build(b) {
        b.text(
          `Generalised coordinates $${q}\\in Q$ fully describe the configuration; the state is $(${q},${qd})$.`,
        );
        b.math(
          String.raw`${q} = ${coords} = ${b.live.vec(n, (i) => snap.v('Q', i))},\quad
          ${qd} = ${b.live.vec(n, (i) => snap.v('QD', i))},\quad
          ${qdd} = ${b.live.vec(n, (i) => snap.v('QDD', i))}`,
        );
      },
    },
    {
      id: 'eom',
      title: 'Equations of motion',
      cite: 'MLS (4.24)',
      build(b) {
        b.math(
          String.raw`${cls(C_M, String.raw`${Mq}\,${qdd}`)} + ${cls(C_C, String.raw`${Cq}\,${qd}`)} + ${cls(C_N, Nq)} = ${cls(C_TAU, String.raw`\tau`)}`,
        );
        b.text(
          'Each row balances generalised forces: <span class="k t-m">inertial</span>, ' +
            '<span class="k t-c">Coriolis &amp; centrifugal</span>, <span class="k t-n">gravity &amp; friction</span> ' +
            'and <span class="k t-tau">applied</span>. Live values:',
        );
        b.math(
          String.raw`${b.live.vec(n, (i) => snap.v('MQDD', i), undefined, `lv ${C_M}`)}
          + ${b.live.vec(n, (i) => snap.v('CQD', i), undefined, `lv ${C_C}`)}
          + ${b.live.vec(n, (i) => snap.v('N', i), undefined, `lv ${C_N}`)}
          = ${b.live.vec(n, (i) => snap.v('TAU', i), undefined, `lv ${C_TAU}`)}`,
        );
        const nu = model.B[0]?.length ?? 1;
        b.math(
          String.raw`\tau = B\,u + \tau_{\text{ext}},\quad B = ${texMat(model.B)},\quad
          u = ${b.live.vec(nu, (i) => snap.v('U', i))},\quad
          \tau_{\text{ext}} = ${b.live.vec(n, (i) => snap.v('TAU_EXT', i))}`,
        );
      },
    },
    {
      id: 'lagrangian',
      title: 'Lagrangian & energy',
      cite: 'MLS §4.2',
      build(b) {
        b.math(String.raw`L(${q},${qd}) = T(${q},${qd}) - V(${q}),\qquad
          \frac{d}{dt}\frac{\partial L}{\partial ${qd}_i} - \frac{\partial L}{\partial ${q}_i} = \Upsilon_i`);
        b.math(
          model.T.length < 240
            ? String.raw`T = \tfrac12\,${qd}^{T} M(${q})\,${qd} = ${model.T}`
            : String.raw`T = \tfrac12\,${qd}^{T} M(${q})\,${qd} = \tfrac12\sum_{i,j} M_{ij}(${q})\,${qd}_i ${qd}_j`,
        );
        b.math(String.raw`V = ${model.V}`);
        b.math(
          String.raw`T = ${b.live.num(() => snap.v('ENERGY', 0))}\,\mathrm{J},\quad
          V = ${b.live.num(() => snap.v('ENERGY', 1))}\,\mathrm{J},\quad
          E = T + V = ${b.live.num(() => snap.v('ENERGY', 2))}\,\mathrm{J}`,
        );
        b.text(
          `Because $\\dot M - 2C$ is skew-symmetric, the energy changes only through work done on the system:`,
        );
        b.math(
          String.raw`\dot E = ${qd}^{T}\tau - ${qd}^{T}\operatorname{diag}(\beta)\,${qd}
          = ${b.live.num(() => snap.v('POWER'))} - ${b.live.num(() => snap.v('DISSIPATION'))}
          = ${b.live.num(() => snap.v('POWER') - snap.v('DISSIPATION'))}\,\mathrm{W}`,
        );
      },
    },
    {
      id: 'inertia',
      title: 'Inertia matrix',
      cite: 'MLS Lemma 4.2',
      build(b) {
        b.math(String.raw`${Mq} = \frac{\partial^2 T}{\partial ${qd}^2} = ${texMat(model.M)}`);
        b.math(String.raw`${Mq} = ${b.live.mat(n, n, (i, j) => snap.m('M', i, j), undefined, `lv ${C_M}`)}`);
        b.math(
          String.raw`M = M^{T} \succ 0:\quad \det M = ${b.live.num(() => snap.v('DET_M'))},\quad
          \lambda(M) = \{${b.live.num(() => snap.v('EIG_M', 0))},\ ${b.live.num(() => snap.v('EIG_M', 1))}\}`,
        );
      },
    },
    {
      id: 'coriolis',
      title: 'Coriolis matrix',
      cite: 'MLS (4.23)',
      build(b) {
        b.math(String.raw`C_{ij}(${q},${qd}) = \frac{1}{2}\sum_{k=1}^{n}\left(
          \frac{\partial M_{ij}}{\partial ${q}_k} + \frac{\partial M_{ik}}{\partial ${q}_j}
          - \frac{\partial M_{kj}}{\partial ${q}_i}\right)${qd}_k`);
        b.math(String.raw`${Cq} = ${texMat(model.C)}`);
        b.math(String.raw`${Cq} = ${b.live.mat(n, n, (i, j) => snap.m('C', i, j), undefined, `lv ${C_C}`)}`);
        b.text('Passivity: with this choice of $C$, $\\dot M - 2C$ is skew-symmetric (MLS Lemma 4.2).');
        b.math(
          String.raw`\dot M = ${b.live.mat(n, n, (i, j) => snap.m('MDOT', i, j))},\quad
          \max_{ij}\left|(\dot M - 2C) + (\dot M - 2C)^{T}\right| = ${b.live.num(() => snap.v('SKEW'), sci)}`,
        );
      },
    },
    {
      id: 'gravity',
      title: 'Gravity & friction',
      cite: 'MLS (4.24)',
      build(b) {
        b.math(String.raw`${Nq} = \frac{\partial V}{\partial ${q}} + \operatorname{diag}(\beta)\,${qd}
          = ${texMat(model.N)}`);
        b.math(String.raw`${Nq} = ${b.live.vec(n, (i) => snap.v('N', i), undefined, `lv ${C_N}`)},\quad
          \frac{\partial V}{\partial ${q}} = ${b.live.vec(n, (i) => snap.v('DV', i))}`);
      },
    },
    {
      id: 'frames',
      title: 'Coordinate frames',
      cite: 'MLS §2.3',
      build(b) {
        b.text(
          'Rigid motions in the plane are elements of $SE(2)$; $g_{ab}$ maps coordinates in frame $b$ to frame $a$, and they compose as $g_{ac} = g_{ab}\\,g_{bc}$.',
        );
        for (const f of model.frames) {
          b.math(
            String.raw`g_{${f.parent}${f.name}} = ${texMat(f.local)} = ${b.live.mat(3, 3, (i, j) => {
              const [x, y, p] = localPose(f.parent, f.name);
              return hom(x, y, p, i, j);
            })}`,
          );
          if (f.parent !== 's') {
            b.math(
              String.raw`g_{s${f.name}} = ${texMat(f.world)} = ${b.live.mat(3, 3, (i, j) => {
                const [x, y, p] = pose(f.name);
                return hom(x, y, p, i, j);
              })}`,
            );
          }
        }
      },
    },
  ];

  const tool = model.tool;
  if (tool) {
    const t = tool.frame;
    sections.push({
      id: 'kinematics',
      title: 'Forward kinematics & Jacobians',
      cite: 'MLS (3.3), §3.4',
      build(b) {
        b.text(
          'Product of exponentials: each revolute joint contributes a twist $\\xi_i = (-\\omega_i\\times q_i,\\ \\omega_i)$, written here as $(v_x, v_y, \\omega)$.',
        );
        const xi = model.q.map((_, k) => texMat(tool.twists.map((row) => [row[k] ?? '0'])));
        b.math(
          String.raw`g_{s${t}}(${q}) = ${model.q.map((qi, k) => String.raw`e^{\hat\xi_{${k + 1}}${qi}}`).join('')}\,g_{s${t}}(0),\quad
          g_{s${t}}(0) = ${texMat(tool.gst0)},\quad ${xi.map((x, k) => String.raw`\xi_{${k + 1}} = ${x}`).join(', ')}`,
        );
        b.math(
          String.raw`p_{s${t}}(${q}) = ${texMat(tool.p)} = ${b.live.vec(2, (i) => snap.v('TOOL_P', i))}`,
        );
        b.math(
          String.raw`\frac{\partial p_{s${t}}}{\partial ${q}} = ${texMat(tool.Jp)} = ${b.live.mat(2, n, (i, j) => snap.m('JP', i, j))}`,
        );
        b.math(
          String.raw`\dot p_{s${t}} = \frac{\partial p}{\partial ${q}}\,${qd} = ${b.live.vec(2, (i) => {
            let s = 0;
            for (let j = 0; j < n; j++) s += snap.m('JP', i, j) * snap.v('QD', j);
            return s;
          })}\ \mathrm{m/s}`,
        );
        b.text(
          "Spatial manipulator Jacobian: the columns are the joint twists $\\xi_i'$ in the current configuration, $V^s_{st} = J^s_{st}(\\theta)\\,\\dot\\theta$.",
        );
        b.math(
          String.raw`J^{s}_{s${t}}(${q}) = ${texMat(tool.Js)} = ${b.live.mat(3, n, (i, j) => snap.m('JS', i, j))}`,
        );
      },
    });
  }

  sections.push({
    id: 'notation',
    title: 'Notation & parameters',
    open: false,
    build(b) {
      const rows = model.params.map((p) => String.raw`${p.tex} &= ${b.live.num(() => ctx.param(p.id))}`);
      b.math(String.raw`\begin{aligned}${rows.join(String.raw`\\`)}\end{aligned}`);
      if (model.trig.length) {
        b.math(
          String.raw`\begin{aligned}${model.trig.map(([s, m]) => String.raw`${s} &= ${m}`).join(String.raw`\\`)}\end{aligned}`,
        );
      }
      b.text('SI units throughout: m, kg, s, rad, N, N·m, J.');
    },
  });

  return sections;
}
