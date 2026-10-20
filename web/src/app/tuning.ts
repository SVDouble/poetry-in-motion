/** The Simulate stage's tuning panel: joint targets, controller settings and the symbolic coefficients. */

import type { Derivation } from '../model/derivation';
import type { Mechanism } from '../model/description';
import type { SimClient } from '../sim/client';
import type { Flag } from '../sim/layout';
import { card, DEG, note, type SliderOpts, slider } from '../ui/widgets';

export interface Tuning {
  umax: number;
  push: number;
  pdWn: number;
  ctWn: number;
  refWn: number;
  lqrQx: number;
  lqrQv: number;
  lqrR: number;
  swing: number;
  coriolis: boolean;
  frames: boolean;
  trail: boolean;
}

export const defaultTuning = (): Tuning => ({
  umax: 80,
  push: 20,
  pdWn: 6,
  ctWn: 12,
  refWn: 4,
  lqrQx: 10,
  lqrQv: 1,
  lqrR: 1,
  swing: 12,
  coriolis: true,
  frames: true,
  trail: true,
});

/** Flags that mirror tuning values in the core. */
export const TUNING_FLAGS: [keyof Tuning, Flag][] = [
  ['umax', 'Umax'],
  ['pdWn', 'PdWn'],
  ['ctWn', 'CtWn'],
  ['refWn', 'RefWn'],
  ['lqrQx', 'LqrQx'],
  ['lqrQv', 'LqrQv'],
  ['lqrR', 'LqrR'],
  ['swing', 'SwingGain'],
];

type Range = Pick<SliderOpts, 'min' | 'max'> & Partial<Pick<SliderOpts, 'log' | 'unit'>>;

const CONTROLLER: [keyof Tuning, string, Range][] = [
  ['umax', String.raw`u_{\max}`, { min: 1, max: 300, log: true, unit: 'N·m | N' }],
  ['push', String.raw`u_{\text{key}}`, { min: 1, max: 100, log: true, unit: 'N·m | N' }],
  ['pdWn', String.raw`\omega_{pd}`, { min: 0.5, max: 30, log: true, unit: 'rad/s' }],
  ['ctWn', String.raw`\omega_{c}`, { min: 1, max: 40, log: true, unit: 'rad/s' }],
  ['refWn', String.raw`\omega_{\text{ref}}`, { min: 0.5, max: 20, log: true, unit: 'rad/s' }],
  ['lqrQx', 'Q_x', { min: 0.1, max: 1000, log: true }],
  ['lqrQv', String.raw`Q_{\dot x}`, { min: 0.01, max: 100, log: true }],
  ['lqrR', 'R', { min: 0.001, max: 100, log: true }],
  ['swing', 'k_E', { min: 0, max: 40 }],
];

export function tuningPanel(
  d: Derivation,
  mech: Mechanism,
  tuning: Tuning,
  params: number[],
  client: SimClient,
  onTarget: () => void,
): HTMLElement {
  const host = document.createElement('div');
  host.className = 'editor';
  const flagOf = new Map(TUNING_FLAGS);

  const targets = d.coords.map((c, i) => {
    const angular = c.kind === 'revolute';
    const b = mech.bodies[c.body]!;
    const range = angular ? 180 : Math.max(b.limit, 1);
    return slider({
      label: String.raw`${c.tex}^{*}`,
      tex: true,
      min: -range,
      max: range,
      step: angular ? 1 : 0.01,
      value: b.qTarget * (angular ? DEG : 1),
      unit: angular ? '°' : 'm',
      onInput: (v) => {
        b.qTarget = v / (angular ? DEG : 1);
        onTarget();
        client.send({ type: 'target', i, v: b.qTarget });
      },
    });
  });
  const targetsNote = d.closures.length
    ? [
        note(
          'With loops, q* is moved onto the constraint manifold: the actuated targets are kept, the rest follows.',
        ),
      ]
    : [];

  const controller = CONTROLLER.map(([key, label, range]) =>
    slider({
      label,
      tex: true,
      value: tuning[key] as number,
      ...range,
      onInput: (v) => {
        (tuning[key] as number) = v;
        const flag = flagOf.get(key);
        if (flag) client.flag(flag, v);
      },
    }),
  );

  const max = { m: 10, g: 20, beta: 2, l: 2, r: 2, I: 2 } as const;
  const coeffs = d.params.map((p) =>
    slider({
      label: p.tex,
      tex: true,
      min: p.kind === 'r' ? -1 : 0,
      max: max[p.kind],
      step: 0.001,
      value: params[p.index] ?? p.value,
      onInput: (v) => {
        params[p.index] = v;
        client.send({ type: 'param', i: p.index, v });
      },
    }),
  );

  host.append(
    card('Targets q*', ...targetsNote, ...targets),
    card('Controller', ...controller),
    card(
      'Coefficients',
      note(
        'The equations were derived with symbolic coefficients, so these only change numbers — no re-derivation. (Uniform-rod links keep r and I as set here until you rebuild.)',
        'prose',
      ),
      ...coeffs,
    ),
  );
  return host;
}
