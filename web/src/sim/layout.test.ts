import { beforeAll, expect, it } from 'vitest';
import { parseLayout } from './layout';
import { TestCore } from './testing';

type Metadata = {
  abiVersion: number;
  len: number;
  fields: Record<string, [number, number]>;
  slot: Record<string, [number, number]>;
  slots: number;
  limits: { maxBodies: number };
  defaults: Record<string, number>;
  flags: Record<string, number>;
};
let metadata: string;
beforeAll(async () => {
  const t = await TestCore.load();
  metadata = t.text(t.core.layout_ptr(), t.core.layout_len());
});

it('checks required widths, bounds, overlap, capacities, defaults and compatibility against real WASM', () => {
  expect(parseLayout(metadata).len).toBeGreaterThan(0);
  for (const corrupt of [
    (r: Metadata) => {
      r.abiVersion++;
    },
    (r: Metadata) => {
      r.len = -1;
    },
    (r: Metadata) => {
      r.fields.Q![0] = r.len;
    },
    (r: Metadata) => {
      r.fields.Q![1]--;
    },
    (r: Metadata) => {
      r.fields.Q![0] = r.fields.QD![0];
    },
    (r: Metadata) => {
      delete r.fields.TIME;
    },
    (r: Metadata) => {
      r.slot.TELEMETRY![1]++;
    },
    (r: Metadata) => {
      r.slots--;
    },
    (r: Metadata) => {
      r.limits.maxBodies++;
    },
    (r: Metadata) => {
      delete r.defaults.MotorSpeed;
    },
    (r: Metadata) => {
      r.flags.Lead = (r.flags.Lead ?? 0) + 1;
    },
  ]) {
    const raw = JSON.parse(metadata);
    corrupt(raw);
    expect(() => parseLayout(JSON.stringify(raw))).toThrow(/core contract/);
  }
});
