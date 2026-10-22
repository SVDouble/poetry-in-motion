import { describe, expect, it } from 'vitest';
import { encode, PRESETS } from '../model/description';
import { parseLayout } from '../sim/layout';
import { TestCore } from '../sim/testing';
import { defaultSettings, pushSettings } from './settings';

describe('core settings metadata', () => {
  it.each(['Cart-pole', 'Seven-joint arm', 'Four-bar linkage'])(
    'preserves fresh simulation behavior when browser defaults are applied (%s)',
    async (name) => {
      const baseline = await TestCore.load();
      const configured = await TestCore.load();
      const desc = encode(PRESETS[name]!());
      baseline.build(desc);
      configured.build(desc);
      pushSettings(configured, defaultSettings(configured.layout));
      const kind = baseline.controllers().specs.find((s) => s.key === 'lqr')!.id;
      for (const tc of [baseline, configured]) {
        tc.core.set_controller(0, kind);
        tc.core.step(100);
      }
      const expected = baseline.snapshot();
      const actual = configured.snapshot();
      expect(actual.t).toBeCloseTo(100 * configured.layout.dt, 12);
      for (let i = 0; i < expected.data.length; i++) {
        expect(Number.isFinite(actual.data[i]), `snapshot[${i}]`).toBe(true);
        expect(actual.data[i], `snapshot[${i}]`).toBeCloseTo(expected.data[i]!, 10);
      }
    },
  );

  it('reports missing defaults and incompatible timestep/limit metadata', async () => {
    const tc = await TestCore.load();
    const defaults = { ...tc.layout.defaults };
    delete defaults.PushK;
    expect(() => defaultSettings({ defaults })).toThrow(/PushK.*rebuild/);
    defaults.PushK = Number.NaN;
    expect(() => defaultSettings({ defaults })).toThrow(/PushK.*rebuild/);
    const c = tc.core;
    const raw = JSON.parse(tc.text(c.layout_ptr(), c.layout_len()));
    for (const dt of [undefined, 0, -0.001, null])
      expect(() => parseLayout(JSON.stringify({ ...raw, dt }))).toThrow(/timestep.*rebuild/);
    expect(() => parseLayout(JSON.stringify({ ...raw, limits: {} }))).toThrow(/limit.*rebuild/);
    expect(() => parseLayout(JSON.stringify({ ...raw, defaults: undefined }))).toThrow(/defaults.*rebuild/);
  });
});
