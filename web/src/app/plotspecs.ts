/** The rolling plots of the Simulate stage: the primary's motion and balance, and every slot side by side. */
import { type ControlStore, SLOT_COLORS } from '../control/store';
import type { Derivation } from '../model/derivation';
import type { PlotSpec, SeriesSpec } from '../plots/plots';
import { css, palette } from '../scene/theme';
import type { SlotField, Snapshot } from '../sim/layout';

const COLORS = [palette.accent, palette.input, palette.coriolis, palette.gravity, palette.inertial, 0xe8eefc];
const plain = (tex: string) => tex.replace(/\\theta_\{(\d+)\}/, 'θ$1').replace(/d_\{(\d+)\}/, 'd$1');

export function plotSpecs(d: Derivation, s: Snapshot, store: ControlStore): PlotSpec[] {
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
  const perSlot = (field: (s: Snapshot) => SlotField): SeriesSpec[] =>
    store.slots.flatMap((kind, i) =>
      kind < 0
        ? []
        : [
            {
              label: store.spec(kind)?.name ?? '?',
              color: css(SLOT_COLORS[i]!),
              get: () => s.slots[i]!.v(field(s)),
              ...(i > 0 ? { dash: [5, 3] } : {}),
            },
          ],
    );
  const tool = d.tool !== null;
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
      title: 'Tracking error',
      unit: tool ? 'rad | m' : 'rad, m',
      series: perSlot((snap) => (snap.v('USE_TOOL') ? 'ERR_X' : 'ERR_Q')),
    },
    { title: 'Effort ‖u‖', unit: 'N·m | N', series: perSlot(() => 'EFFORT') },
    { title: `Row ${row + 1} of the equations of motion`, unit: 'N·m | N', series: balance },
  ];
}
