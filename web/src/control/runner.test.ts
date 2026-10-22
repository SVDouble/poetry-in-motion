import { beforeAll, describe, expect, it } from 'vitest';
import { defaultSettings } from '../app/settings';
import { encode, PRESETS } from '../model/description';
import type { SimClient } from '../sim/client';
import { TestCore } from '../sim/testing';
import type { Experiment } from './page';
import { type Port, runExperiment } from './runner';
import { ControlStore } from './store';

let tc: TestCore;
beforeAll(async () => {
  tc = await TestCore.load();
});
const push: Experiment = {
  title: 'push',
  goal: 'cancellation',
  actions: [{ do: 'push', dx: 0.4, ms: 100 }],
  plots: [],
  notice: '',
};
const flush = async () => {
  for (let i = 0; i < 8; i++) await Promise.resolve();
};
function harness() {
  const d = tc.build(encode(PRESETS['Two-link arm']!()));
  const store = new ControlStore({ send: (msg) => tc.send(msg) } as SimClient, tc.layout.slots);
  store.load(tc.controllers(), 0);
  const waits: { ms: number; done: () => void }[] = [];
  const effects: string[] = [];
  let point: [number, number] | null = null;
  const port: Port = {
    d,
    store,
    settings: defaultSettings(tc.layout),
    send: () => {
      effects.push('send');
    },
    flag: (flag, value) => {
      effects.push(`${flag}=${value}`);
    },
    snap: () => tc.snapshot(),
    sleep: (ms) => new Promise<void>((done) => waits.push({ ms, done })),
    reset: () => {
      effects.push('reset');
    },
    setUseTool: () => {
      effects.push('tool');
    },
    setPush: (at) => {
      point = at;
      effects.push(`push:${JSON.stringify(at)}`);
    },
    mark: (event) => {
      effects.push(`mark:${event}`);
    },
  };
  return { port, effects, waits, point: () => point };
}

describe('experiment cancellation', () => {
  it.each([0.0007, 0.0023])('waits for physical durations at timestep %s', async (dt) => {
    const core = await TestCore.load();
    const d = core.build(encode(PRESETS['Two-link arm']!()));
    expect(core.core.set_dt(dt)).toBe(0);
    const store = new ControlStore({ send: (msg) => core.send(msg) } as SimClient, core.layout.slots);
    store.load(core.controllers(), 0);
    const marks = new Map<string, number>();
    let clock = 0;
    const port: Port = {
      d,
      store,
      settings: defaultSettings(core.layout),
      send: (m) => core.send(m),
      flag: (f, v) => core.flag(f, v),
      snap: () => core.snapshot(),
      sleep: async (ms) => {
        core.advance(
          ms,
          (seconds) => {
            clock += seconds;
          },
          dt,
        );
      },
      reset: () => core.send({ type: 'reset' }),
      setUseTool: () => undefined,
      setPush: () => undefined,
      mark: (event) => {
        marks.set(event, core.snapshot().t);
      },
    };
    await runExperiment(
      port,
      0,
      {
        ...push,
        actions: [
          { do: 'wait', ms: 100 },
          { do: 'kick', v: 0.1 },
          { do: 'wait', ms: 50 },
        ],
      },
      () => true,
    );
    expect(marks.get('kick')!).toBeGreaterThanOrEqual(0.1 - 1e-10);
    expect(marks.get('kick')!).toBeLessThan(0.1 + dt);
    expect(marks.get('done')!).toBeGreaterThanOrEqual(0.15 - 1e-10);
    expect(marks.get('done')!).toBeLessThan(0.15 + 2 * dt);
    expect(clock).toBeCloseTo(core.snapshot().t, 12);
  });

  it('does not mutate state or mark completion after cancellation during setup', async () => {
    const h = harness();
    let alive = true;
    const running = runExperiment(h.port, 0, push, () => alive);
    expect(h.waits[0]?.ms).toBe(300);
    alive = false;
    h.effects.length = 0;
    h.waits[0]!.done();
    await running;
    expect(h.effects).toEqual([]);
  });

  it('cannot release a newer run’s push when its previous wait completes', async () => {
    const h = harness();
    let owner = 1;
    const first = runExperiment(h.port, 0, push, () => owner === 1);
    h.waits[0]!.done();
    await flush();
    expect(h.point()).not.toBeNull();
    expect(h.waits[1]?.ms).toBe(100);
    owner = 2;
    // The stage releases the old owner's resources synchronously on cancellation.
    h.port.setPush(null);
    h.port.flag('PushOn', 0);
    const second = runExperiment(h.port, 0, push, () => owner === 2);
    h.waits[2]!.done();
    await flush();
    const current = h.point();
    expect(current).not.toBeNull();
    h.effects.length = 0;
    h.waits[1]!.done();
    await first;
    expect(h.point()).toEqual(current);
    expect(h.effects).toEqual([]);
    h.waits[3]!.done();
    await second;
    expect(h.point()).toBeNull();
    expect(h.effects).toContain('mark:done');
  });
});
