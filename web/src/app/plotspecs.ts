/** The three rolling plots of the Simulate stage. */
import type { Derivation } from '../model/derivation';
import type { PlotSpec } from '../plots/plots';
import { css, palette } from '../scene/theme';
import type { Snapshot } from '../sim/layout';

const COLORS = [palette.accent, palette.input, palette.coriolis, palette.gravity, palette.inertial, 0xe8eefc];
const plain = (tex: string) => tex.replace(/\\theta_\{(\d+)\}/, 'θ$1').replace(/d_\{(\d+)\}/, 'd$1');

export function plotSpecs(d: Derivation, s: Snapshot): PlotSpec[] {
  // the most interesting row: the first passive one (underactuated systems), else the first
  const row = Math.max(
    0,
    d.coords.findIndex((c) => !c.actuated),
  );
  const balance = [
    { label: 'Mq̈', color: css(palette.inertial), get: () => s.v('MQDD', row) },
    { label: 'Cq̇', color: css(palette.coriolis), get: () => s.v('CQD', row) },
    { label: 'N', color: css(palette.gravity), get: () => s.v('N', row) },
    ...(d.closures.length
      ? [{ label: 'Aᵀλ', color: css(palette.accent), get: () => s.v('AT_LAMBDA', row) }]
      : []),
    { label: 'τ', color: css(palette.input), get: () => s.v('TAU', row), dash: [4, 4] },
  ];
  return [
    {
      title: 'Configuration',
      unit: 'rad, m',
      series: d.coords.map((c, i) => ({
        label: plain(c.tex),
        color: css(COLORS[i % COLORS.length]!),
        get: () => s.v('Q', i),
      })),
    },
    {
      title: 'Energy',
      unit: 'J',
      series: [
        { label: 'T', color: css(palette.inertial), get: () => s.v('ENERGY', 0) },
        { label: 'V', color: css(palette.gravity), get: () => s.v('ENERGY', 1) },
        { label: 'E', color: '#e8eefc', get: () => s.v('ENERGY', 2) },
      ],
    },
    { title: `Row ${row + 1} of the equations of motion`, unit: 'N·m | N', series: balance },
  ];
}
