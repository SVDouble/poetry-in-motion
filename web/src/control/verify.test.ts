/**
 * Verifies every guided experiment on every preset mechanism where it is offered: the real experiment
 * runner plays it against the real wasm core, the signals it names are recorded, and the claim its "what to
 * notice" makes is checked numerically. Run `make wasm` first.
 */
import { beforeAll, describe, expect, it } from 'vitest';
import { defaultSettings } from '../app/settings';
import { encode, PRESETS } from '../model/description';
import { Signals } from '../scope/signals';
import type { SimClient } from '../sim/client';
import type { Snapshot } from '../sim/layout';
import { TestCore } from '../sim/testing';
import { LABS } from './lab';
import { type Experiment, pageContext } from './page';
import { type Port, runExperiment } from './runner';
import { ControlStore } from './store';

const SAMPLE_MS = 10;

interface Trace {
  t: number[];
  s(id: string): number[];
  /** Time of the first mark `name` (NaN if none). */
  mark(name: string): number;
  /** Mean of |id| over the last `window` seconds (or before `until`). */
  final(id: string, window?: number, until?: number): number;
  max(id: string, from?: number, to?: number): number;
  /** The input bound at the end of the setup. */
  umax: number;
}

interface Check {
  /** Extra signals the check needs beyond the experiment's plots. */
  ids?: (key: string) => string[];
  /** Observation time after the actions, s. */
  after?: number;
  /** null if the claim holds, else what failed. */
  verify(tr: Trace, key: string): string | null;
}

const err = (key: string) => `slot:${key}:err`;
const fmt = (v: number) => (Math.abs(v) < 1e-3 && v !== 0 ? v.toExponential(1) : v.toFixed(3));
const fail = (ok: boolean, msg: string) => (ok ? null : msg);
const all = (...r: (string | null)[]) => r.find((x) => x !== null) ?? null;

/** Settling: |id| ends below `tol`. */
const settles = (tr: Trace, id: string, tol: number) =>
  fail(tr.final(id) < tol, `${id} ends at ${fmt(tr.final(id))} (want < ${tol})`);
const falls = (tr: Trace, id: string, tol: number) =>
  fail(tr.final(id) > tol, `${id} ends at ${fmt(tr.final(id))} (want > ${tol}: should fail)`);

/** Value at the end of the push (mean over its last 0.3 s). */
const endOfPush = (tr: Trace, id: string) => tr.final(id, 0.3, tr.mark('release'));

const CHECKS: Record<string, Check> = {
  'Where does the energy go?': {
    verify: (tr) =>
      all(
        fail(tr.max('budget') < 1e-3, `budget residual ${fmt(tr.max('budget'))} J`),
        // with friction it does work; without, E stays put
        fail(
          tr.final('work:fric') > 0 || tr.max('energy:E') - tr.final('energy:E') < 1e-3,
          'E changed without friction',
        ),
      ),
  },
  'Switch Coriolis off': {
    verify: (tr) =>
      fail(tr.max('budget') > 1e-3, `residual stays at ${fmt(tr.max('budget'))} J (C q̇ had no effect)`),
  },
  'Exact compensation vs. sag': {
    verify: (tr) =>
      all(
        settles(tr, err('pd_gravity'), 5e-3),
        fail(tr.final(err('pd')) > 5 * tr.final(err('pd_gravity')) + 1e-3, 'PD does not sag'),
      ),
  },
  'The gravity sag': {
    verify: (tr) =>
      all(
        settles(tr, err('pd_gravity'), 5e-3),
        fail(tr.final(err('pd')) > 5 * tr.final(err('pd_gravity')) + 1e-3, 'PD does not sag'),
      ),
  },
  'Damping ratio': {
    ids: (key) => [err(key)],
    verify: (tr, key) => {
      // ζ = 0.15: the error rings — several local maxima after the kick
      const e = tr.s(err(key));
      const k0 = tr.t.findIndex((t) => t >= tr.mark('kick'));
      let peaks = 0;
      for (let k = k0 + 1; k < e.length - 1; k++)
        if (e[k]! > e[k - 1]! && e[k]! >= e[k + 1]! && e[k]! > 1e-3) peaks++;
      return fail(peaks >= 3, `only ${peaks} oscillation peaks`);
    },
  },
  'PD cannot balance': {
    verify: (tr) => all(falls(tr, err('pd'), 0.3), settles(tr, err('lqr'), 0.02)),
  },
  'Same error dynamics everywhere': {
    verify: (tr) => all(settles(tr, err('computed_torque'), 1e-3), settles(tr, err('pd_gravity'), 0.02)),
  },
  'Saturation breaks cancellation': {
    ids: () => ['slot:computed_torque:effort'],
    verify: (tr) => {
      const kick = tr.mark('kick');
      // unsaturated, ω_c = 12 rad/s settles within ~0.6 s; saturated it takes visibly longer
      const late = tr.final(err('computed_torque'), 0.1, kick + 0.8);
      return all(
        fail(tr.max('slot:computed_torque:effort', kick) >= 7.9, 'the actuators never saturated'),
        fail(late > 0.01, `already settled 0.8 s after the kick (${fmt(late)})`),
      );
    },
  },
  'Joint stiffness vs. task stiffness': {
    verify: (tr) => {
      // impedance renders F / K_d; computed torque deflects during the push and returns after it.
      // Each slot feels its own push; the shadow's (0.4 m at 200 N/m, minus 0.1 m give) is capped at 30 N.
      const [x, f] = [endOfPush(tr, err('impedance')), 30];
      return all(
        fail(Math.abs(300 * x - f) < 0.05 * f, `impedance K_d·|e| = ${fmt(300 * x)} N vs F = ${fmt(f)} N`),
        fail(endOfPush(tr, err('computed_torque')) > 1e-3, 'computed torque did not deflect'),
        settles(tr, err('computed_torque'), 2e-3),
      );
    },
  },
  'Watch the Lyapunov function': {
    ids: () => ['tele:passivity:lyap:0'],
    verify: (tr) => {
      const v = tr.s('tele:passivity:lyap:0');
      const k0 = tr.t.findIndex((t) => t >= tr.mark('kick') + 0.05);
      let rises = 0;
      for (let k = k0 + 1; k < v.length; k++) if (v[k]! > v[k - 1]! + 1e-6 * Math.max(1, v[k0]!)) rises++;
      return all(
        fail(v[k0]! > 1e-3, 'the kick did not raise V'),
        fail(rises === 0, `V rose in ${rises} samples`),
      );
    },
  },
  'Passivity vs. computed torque': {
    verify: (tr) => all(settles(tr, err('passivity'), 1e-3), settles(tr, err('computed_torque'), 1e-3)),
  },
  'Energy of the closed loop': {
    ids: () => ['tele:vmc:lyap:0', 'taskerr'],
    verify: (tr) => {
      const v = tr.s('tele:vmc:lyap:0');
      const k0 = tr.t.findIndex((t) => t >= tr.mark('release') + 0.05);
      let rises = 0;
      for (let k = k0 + 1; k < v.length; k++) if (v[k]! > v[k - 1]! + 1e-6) rises++;
      return all(
        fail(rises === 0, `V_L rose in ${rises} samples after the release`),
        settles(tr, 'taskerr', 0.01),
      );
    },
  },
  'Stiff spring, same damping ratio': {
    ids: () => ['taskerr', 'fext:n'],
    verify: (tr) => {
      const [x, f] = [endOfPush(tr, 'taskerr'), endOfPush(tr, 'fext:n')];
      return fail(Math.abs(1500 * x - f) < 0.15 * f, `K·Δx = ${fmt(1500 * x)} N vs F = ${fmt(f)} N`);
    },
  },
  'Posture in the null space': {
    ids: () => ['tele:vmc:leak:0', 'taskerr', ...Array.from({ length: 7 }, (_, i) => `q:${i}`)],
    verify: (tr) => {
      const h = tr.mark('hold');
      const k0 = tr.t.findIndex((t) => t >= h);
      // the joint that moves most: with the tool held the null space may leave the shoulder nearly still
      const moved = Math.max(
        ...Array.from({ length: 7 }, (_, i) => tr.s(`q:${i}`))
          .map((q) => Math.abs(q[q.length - 1]! - q[k0]!))
          .filter(Number.isFinite),
      );
      return all(
        fail(
          tr.max('tele:vmc:leak:0', h + 0.05) < 1e-5,
          `leakage up to ${fmt(tr.max('tele:vmc:leak:0', h + 0.05))}`,
        ),
        settles(tr, 'taskerr', 5e-3),
        fail(moved > 0.05, `the joints moved only ${fmt(moved)} rad`),
      );
    },
  },
  'The spring law': {
    ids: () => ['taskerr', 'fext:n'],
    verify: (tr) => {
      const [x, f] = [endOfPush(tr, 'taskerr'), endOfPush(tr, 'fext:n')];
      return fail(Math.abs(300 * x - f) < 0.05 * f, `K_d·|e| = ${fmt(300 * x)} N vs F = ${fmt(f)} N`);
    },
  },
  'Light virtual mass': {
    ids: () => ['taskerr'],
    verify: (tr) => settles(tr, 'taskerr', 0.01),
  },
  'Impedance vs. admittance': {
    verify: (tr) => {
      const [i, a] = [endOfPush(tr, err('impedance')), endOfPush(tr, err('admittance'))];
      return fail(a > i, `admittance deflects ${fmt(a)} vs impedance ${fmt(i)}`);
    },
  },
  'Two springs in series': {
    ids: () => [
      'tele:admittance:xv:0',
      'tele:admittance:xv:1',
      'tele:admittance:x:0',
      'tele:admittance:x:1',
      'xref:x',
      'xref:y',
      'fext:n',
    ],
    verify: (tr) => {
      const g = (id: string) => endOfPush(tr, id);
      const virt = Math.hypot(
        g('tele:admittance:xv:0') - g('xref:x'),
        g('tele:admittance:xv:1') - g('xref:y'),
      );
      const lag = Math.hypot(
        g('tele:admittance:x:0') - g('tele:admittance:xv:0'),
        g('tele:admittance:x:1') - g('tele:admittance:xv:1'),
      );
      const f = g('fext:n');
      return all(
        fail(
          Math.abs(100 * virt - f) < 0.05 * f,
          `K_d·|x_v − x_d| = ${fmt(100 * virt)} N vs F = ${fmt(f)} N`,
        ),
        fail(lag > 1e-3, `the tool does not lag (${fmt(lag)})`),
      );
    },
  },
  'A slow inner loop': {
    ids: () => ['tele:admittance:xv:0', 'tele:admittance:x:0', 'taskerr'],
    verify: (tr) => {
      const lag = Math.abs(endOfPush(tr, 'tele:admittance:x:0') - endOfPush(tr, 'tele:admittance:xv:0'));
      return all(fail(lag > 0.01, `lag only ${fmt(lag)} m`), settles(tr, 'taskerr', 0.02));
    },
  },
  'Reaching beyond the workspace': {
    ids: () => Array.from({ length: 7 }, (_, i) => `qd:${i}`),
    verify: (tr) => {
      const end = tr.t[tr.t.length - 1]!;
      const moving = Math.max(...Array.from({ length: 7 }, (_, i) => tr.max(`qd:${i}`, end - 1)));
      return fail(moving < 1e-2, `still moving at ${fmt(moving)} rad/s`);
    },
  },
  'A stack of tasks': {
    ids: (key) => ['taskerr', 'orienterr', 'elbowerr', `tele:${key}:leak:0`, `tele:${key}:leak_sec:0`],
    verify: (tr, key) => {
      const after = tr.mark('stack') + 0.2;
      return all(
        settles(tr, 'taskerr', 2e-3),
        settles(tr, 'orienterr', 2e-3),
        settles(tr, 'elbowerr', 5e-3),
        fail(
          tr.max(`tele:${key}:leak:0`, after) < 1e-6,
          `tool leakage ${fmt(tr.max(`tele:${key}:leak:0`, after))}`,
        ),
        fail(
          tr.max(`tele:${key}:leak_sec:0`, after) < 1e-6,
          `secondary leakage ${fmt(tr.max(`tele:${key}:leak_sec:0`, after))}`,
        ),
      );
    },
  },
  'Priority under conflict': {
    ids: () => ['taskerr', 'orienterr', 'elbowerr'],
    verify: (tr) =>
      all(
        settles(tr, 'taskerr', 2e-3),
        settles(tr, 'orienterr', 2e-3),
        fail(
          tr.final('elbowerr') > 0.05,
          `the elbow reached an impossible goal (${fmt(tr.final('elbowerr'))})`,
        ),
      ),
  },
  'Region of attraction': {
    verify: (tr) => settles(tr, err('lqr'), 0.01),
  },
  'Cheap control': {
    verify: (tr) => settles(tr, err('lqr'), 0.01),
  },
  'Clipping vs. planning': {
    ids: () => ['slot:lqr:effort', 'tele:mpc:active:0'],
    verify: (tr) =>
      all(
        fail(tr.max('slot:lqr:effort', tr.mark('limit')) >= 0.99 * tr.umax, 'LQR never hit the bound'),
        fail(tr.max('tele:mpc:active:0', tr.mark('limit')) >= 1, 'no MPC bound became active'),
      ),
  },
  'Active constraints': {
    ids: () => ['tele:mpc:active:0'],
    verify: (tr) =>
      all(fail(tr.max('tele:mpc:active:0') >= 1, 'no bound became active'), settles(tr, err('mpc'), 0.01)),
  },
  'A coarse model': {
    ids: () => ['slot:mpc:effort', 'slot:lqr:effort'],
    verify: (tr) => {
      // each re-plan holds its input for 50 ms: the MPC effort moves in steps (sampled every 10 ms), the
      // LQR's at the 1 kHz rate does not (samples on the input bound, held by the clip, do not count)
      const held = (id: string) => {
        const k0 = tr.t.findIndex((t) => t > tr.mark('kick'));
        const v = tr.s(id).slice(k0, k0 + 100);
        const top = 0.99 * tr.umax;
        let [same, free] = [0, 0];
        for (let k = 1; k < v.length; k++) {
          if (v[k]! >= top || v[k - 1]! >= top) continue;
          free++;
          if (v[k] === v[k - 1]) same++;
        }
        return same / Math.max(1, free);
      };
      const [mpc, lqr] = [held('slot:mpc:effort'), held('slot:lqr:effort')];
      return fail(mpc > 0.7 && lqr < 0.3, `held fraction: MPC ${fmt(mpc)}, LQR ${fmt(lqr)}`);
    },
  },
  'Pump, then catch': {
    after: 30,
    verify: (tr) => settles(tr, err('swing_up'), 0.05),
  },
  'Gentle pumping': {
    after: 45,
    verify: (tr) => settles(tr, err('swing_up'), 0.05),
  },
};

let tc: TestCore;
beforeAll(async () => {
  tc = await TestCore.load();
});

interface Result {
  preset: string;
  controller: string;
  title: string;
  failure: string | null;
}

async function play(
  snap: Snapshot,
  port: Port,
  kind: number,
  e: Experiment,
  check: Check,
  key: string,
  signals: Signals,
) {
  const ids = [
    ...new Set([...e.plots.flatMap((p) => p.ids), ...(check.ids?.(key) ?? []), 'budget', 'work:fric']),
  ];
  const sigs = ids.map((id) => [id, signals.resolve(id)] as const);
  const rec = new Map<string, number[]>(ids.map((id) => [id, []]));
  const t: number[] = [];
  const marks: [string, number][] = [];
  let clock = 0;
  const sample = () => {
    tc.snapshot(snap);
    t.push(clock);
    for (const [id, s] of sigs) rec.get(id)!.push(s ? s.get() : Number.NaN);
  };
  port.sleep = async (ms) => {
    tc.advance(
      ms,
      (seconds) => {
        clock += seconds;
        sample();
      },
      tc.layout.dt,
      SAMPLE_MS,
    );
  };
  port.mark = (name) => marks.push([name, clock]);
  sample();
  await runExperiment(port, kind, e, () => true);
  await port.sleep((check.after ?? 10) * 1000);
  const series = (id: string) => rec.get(id) ?? [];
  const tr: Trace = {
    t,
    umax: port.settings.umax,
    s: series,
    mark: (name) => marks.find(([n]) => n === name)?.[1] ?? Number.NaN,
    final(id, window = 0.5, until = clock) {
      // samples before a slot runs are NaN: skip them
      const v = series(id).filter((x, k) => t[k]! <= until && t[k]! > until - window && !Number.isNaN(x));
      return v.reduce((a, x) => a + Math.abs(x), 0) / Math.max(1, v.length);
    },
    max(id, from = 0, to = Number.POSITIVE_INFINITY) {
      return series(id).reduce(
        (m, x, k) => (t[k]! >= from && t[k]! <= to && !Number.isNaN(x) ? Math.max(m, Math.abs(x)) : m),
        0,
      );
    },
  };
  const failure = check.verify(tr, key);
  if (!failure) return null;
  // say what was observed: the extremes of the signals the check looks at
  const seen = (check.ids?.(key) ?? []).map((id) => `${id} ≤ ${fmt(tr.max(id))}`).join(', ');
  return seen
    ? `${failure} [${seen}; marks ${marks.map(([n, t]) => `${n}@${t.toFixed(2)}`).join(' ')}]`
    : failure;
}

describe('guided experiments', () => {
  it('make the claims they explain, on every mechanism that offers them', { timeout: 600_000 }, async () => {
    const results: Result[] = [];
    for (const preset of Object.keys(PRESETS)) {
      const d = tc.build(encode(PRESETS[preset]!()));
      const reg = tc.controllers();
      const client = { send: (m) => tc.send(m), flag: (f, v) => tc.flag(f, v) } as Pick<
        SimClient,
        'send' | 'flag'
      >;
      const store = new ControlStore(client as SimClient, tc.layout.slots);
      store.load(reg, 0);
      const snap = tc.snapshot();
      const signals = new Signals(d, snap, store);
      for (const spec of reg.specs) {
        if (reg.applicable[spec.id] !== null) continue;
        const lab = LABS[spec.key]?.(pageContext(d, snap, spec, (i) => store.param(spec.id, i)));
        for (const e of lab?.experiments ?? []) {
          const check = CHECKS[e.title];
          if (!check) {
            results.push({ preset, controller: spec.key, title: e.title, failure: 'no check written' });
            continue;
          }
          const settings = defaultSettings(tc.layout);
          const port: Port = {
            d,
            store,
            settings,
            send: (m) => tc.send(m),
            flag: (f, v) => tc.flag(f, v),
            snap: () => snap,
            sleep: async () => undefined,
            reset: () => tc.send({ type: 'reset' }),
            setUseTool: (on) => {
              settings.useTool = on;
              tc.flag('UseTool', on ? 1 : 0);
            },
            setPush: () => undefined,
          };
          const failure = await play(snap, port, spec.id, e, check, spec.key, signals);
          results.push({ preset, controller: spec.key, title: e.title, failure });
        }
      }
    }
    const failed = results.filter((r) => r.failure);
    process.stdout.write(
      `${results.length - failed.length}/${results.length} experiment runs hold\n${failed
        .map((r) => `✗ ${r.preset} · ${r.controller} · ${r.title} — ${r.failure}\n`)
        .join('')}`,
    );
    expect(failed.map((r) => `${r.preset} · ${r.controller} · ${r.title}: ${r.failure}`)).toEqual([]);
  });
});
