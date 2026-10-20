/** Curated plot layouts for the scope: what is worth looking at for this mechanism and these controllers. */
import { range } from '../control/page';
import type { ControlStore } from '../control/store';
import type { Derivation } from '../model/derivation';
import type { PlotDef, Preset } from './scope';

export function presets(d: Derivation, store: ControlStore, watch: PlotDef[], primaryName: string): Preset[] {
  const n = d.n;
  const running = store.slots.filter((k) => k >= 0).map((k) => store.spec(k)?.key ?? '');
  const perSlot = (what: string) => ({ ids: running.map((key) => `slot:${key}:${what}`) });
  // the most telling row of the equations: the first passive one (underactuated systems), else the first
  const row = Math.max(
    0,
    d.coords.findIndex((c) => !c.actuated),
  );
  const eomRow = (i: number) => ({
    ids: ['m', 'c', 'n', ...(d.closures.length ? ['a'] : []), 'tau'].map((t) => `eom:${i}:${t}`),
  });
  const list: Preset[] = [
    {
      key: 'overview',
      label: 'Overview',
      hint: 'configuration, and the error and effort of every running controller',
      plots: [{ ids: range(n, (i) => `q:${i}`) }, perSlot('err'), perSlot('effort')],
    },
  ];
  if (watch.length) {
    list.push({
      key: 'controller',
      label: primaryName,
      hint: `the signals that matter for ${primaryName}`,
      plots: watch,
    });
  }
  list.push(
    {
      key: 'energy',
      label: 'Energy',
      hint: 'kinetic, potential and total energy; the power balance Ė = q̇ᵀτ − q̇ᵀβq̇',
      plots: [{ ids: ['energy:T', 'energy:V', 'energy:E'] }, { ids: ['energy:P', 'energy:D', 'energy:dE'] }],
    },
    {
      key: 'eom',
      label: 'Equations',
      hint: 'each term of a row of M q̈ + C q̇ + N = τ',
      plots: n > 1 ? [eomRow(row), eomRow(row === 0 ? 1 : 0)] : [eomRow(0)],
    },
  );
  if (d.tool) {
    list.push({
      key: 'task',
      label: 'Task',
      hint: 'the tool against its reference, the push and the manipulability',
      plots: [
        { ids: ['tool:x', 'xref:x', 'tool:y', 'xref:y'] },
        { ids: ['taskerr', 'fext:n'] },
        { ids: ['w'] },
      ],
    });
  }
  return list;
}
