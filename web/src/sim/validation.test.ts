import { beforeAll, describe, expect, it } from 'vitest';
import { encode, PRESETS } from '../model/description';
import { TestCore } from './testing';

let tc: TestCore;
beforeAll(async () => {
  tc = await TestCore.load();
});
function write(desc: number[]): void {
  const ptr = tc.core.desc_buffer(desc.length);
  new Float64Array(tc.core.memory.buffer, ptr, desc.length).set(desc);
}

describe('WASM validation and prepared builds', () => {
  it('rejects malformed flat descriptions and retains the old installed world', () => {
    const desc = encode(PRESETS['Two-link arm']!());
    tc.build(desc);
    tc.core.step(100);
    const previous = tc.snapshot().data.slice();
    for (const [i, value] of [
      [0, Number.NaN],
      [1, 1.5],
      [1, 1e99],
      [3, 20],
      [4, -1],
      [6, 0],
      [7, 9],
      [13, 0.5],
      [14, -1],
      [14, Number.POSITIVE_INFINITY],
    ] as const) {
      const invalid = [...desc];
      invalid[i] = value;
      write(invalid);
      expect(tc.core.build(invalid.length), `${i}=${value}`).toBe(-1);
      expect(tc.snapshot().data).toEqual(previous);
    }
    write([...desc, 0]);
    expect(tc.core.build(desc.length + 1)).toBe(-1);
  });
  it('derives and discards candidates without replacing running state; only commit installs them', () => {
    tc.build(encode(PRESETS['Two-link arm']!()));
    tc.core.step(50);
    const before = tc.snapshot().data.slice();
    const other = encode(PRESETS['Cart-pole']!());
    write(other);
    expect(tc.core.prepare(other.length)).toBe(0);
    expect(tc.snapshot().data).toEqual(before);
    const invalid = [...other];
    invalid[14] = -1;
    write(invalid);
    expect(tc.core.prepare(invalid.length)).toBe(-1);
    expect(tc.core.commit_build()).toBe(-1);
    expect(tc.snapshot().data).toEqual(before);
    write(other);
    expect(tc.core.prepare(other.length)).toBe(0);
    tc.core.discard_build();
    expect(tc.core.commit_build()).toBe(-1);
    expect(tc.snapshot().data).toEqual(before);
    write(other);
    expect(tc.core.prepare(other.length)).toBe(0);
    expect(tc.core.commit_build()).toBe(0);
    expect(tc.snapshot().v('NU')).toBe(1);
    expect(tc.snapshot().t).toBe(0);
  });
  it('reports rejected live mutations without changing the snapshot', () => {
    tc.build(encode(PRESETS['Two-link arm']!()));
    tc.core.step(30);
    const before = tc.snapshot().data.slice();
    expect(tc.core.set_param(0, -1)).toBe(-1);
    expect(tc.core.set_param(0, Number.NaN)).toBe(-1);
    expect(tc.core.set_state(0, Number.NaN, 0)).toBe(-1);
    expect(tc.core.kick(0, Number.POSITIVE_INFINITY)).toBe(-1);
    expect(tc.core.set_dt(Number.NaN)).toBe(-1);
    expect(tc.snapshot().data).toEqual(before);
    expect(tc.text(tc.core.error_ptr(), tc.core.error_len())).toMatch(/invalid simulation input/);
    expect(tc.core.set_param(0, 2)).toBe(0);
  });
});
