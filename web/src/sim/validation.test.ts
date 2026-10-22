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

it.each([
  [2, true],
  [6, false],
] as const)('preserves a platform tool task with frame %i (orientation feasible: %s)', (tool, canTurn) => {
  const mech = PRESETS['3-RPR platform']!();
  // Use the moving platform's frame; the middle piston frame cannot turn independently of its tip.
  mech.tool = tool;
  tc.build(encode(mech));
  const controllers = tc.controllers();
  const kind = controllers.specs.find((s) => s.key === 'impedance')!.id;
  tc.core.set_controller(0, kind);
  const start = tc.snapshot();
  const point = [start.v('TOOL_P'), start.v('TOOL_P', 1)];
  const angle = start.frame(mech.bodies.length + 1)[2] + 0.03;
  tc.flag('TargetX', point[0]!);
  tc.flag('TargetY', point[1]!);
  tc.flag('UseTool', 1);
  tc.flag('OrientTarget', angle);
  tc.flag('Orient', 2);
  for (let i = 0; i < 50; i++) {
    tc.core.step(100);
    const s = tc.snapshot();
    expect(s.v('CONSTRAINT')).toBeLessThan(1e-6);
    expect(s.v('RECOVERY')).toBe(0);
    for (const field of ['leak', 'leak_sec']) {
      const [offset] = controllers.specs[kind]!.telemetry[field]!;
      expect(s.slots[0]!.tele(offset)).toBeLessThan(1e-7);
    }
  }
  const end = tc.snapshot();
  expect(Math.hypot(end.v('TOOL_P') - point[0]!, end.v('TOOL_P', 1) - point[1]!)).toBeLessThan(1e-3);
  const orientationError = Math.abs(end.frame(mech.bodies.length + 1)[2] - angle);
  if (canTurn) expect(orientationError).toBeLessThan(0.005);
  else expect(orientationError).toBeCloseTo(0.03, 6);
});
