/// <reference types="node" />
/** Exercises the real wasm core through the same ABI the worker uses. Run `make wasm` first. */
import { readFile } from 'node:fs/promises';
import { beforeAll, describe, expect, it } from 'vitest';
import type { Derivation } from '../model/derivation';
import { dof, encode, PRESETS, paramVector } from '../model/description';
import { type Layout, parseLayout, Snapshot } from './layout';

interface Core {
  memory: WebAssembly.Memory;
  desc_buffer(len: number): number;
  build(len: number): number;
  error_ptr(): number;
  error_len(): number;
  derivation_ptr(): number;
  derivation_len(): number;
  reset(): void;
  step(n: number): void;
  set_mode(m: number): void;
  set_state(i: number, q: number, qd: number): void;
  param_count(): number;
  out_ptr(): number;
  out_len(): number;
  layout_ptr(): number;
  layout_len(): number;
}

let core: Core;
let layout: Layout;

const text = (ptr: number, len: number) =>
  new TextDecoder().decode(new Uint8Array(core.memory.buffer, ptr, len).slice());

function build(desc: number[]): Derivation {
  const ptr = core.desc_buffer(desc.length);
  new Float64Array(core.memory.buffer, ptr, desc.length).set(desc);
  const ok = core.build(desc.length);
  if (ok !== 0) throw new Error(text(core.error_ptr(), core.error_len()));
  return JSON.parse(text(core.derivation_ptr(), core.derivation_len())) as Derivation;
}

function snapshot(): Snapshot {
  const ptr = core.out_ptr();
  const s = new Snapshot(layout);
  s.data.set(new Float64Array(core.memory.buffer, ptr, layout.len));
  return s;
}

beforeAll(async () => {
  const bytes = await readFile(new URL('../wasm/motion_core.wasm', import.meta.url));
  const { instance } = await WebAssembly.instantiate(bytes, {});
  core = instance.exports as unknown as Core;
  layout = parseLayout(text(core.layout_ptr(), core.layout_len()));
});

describe('wasm core', () => {
  it('exposes every field and flag the UI relies on', () => {
    expect(layout.len).toBe(core.out_len());
  });

  it.each(Object.keys(PRESETS))('derives the %s preset', (name) => {
    const mech = PRESETS[name]!();
    const d = build(encode(mech));
    expect(d.n).toBe(dof(mech));
    expect(d.nu).toBe(mech.bodies.filter((b) => b.actuated && b.joint !== 'fixed').length);
    expect(d.closures).toHaveLength(mech.closures.length);
    expect(d.phi).toHaveLength(mech.closures.reduce((n, c) => n + (c.kind === 'pin' ? 2 : 1), 0));
    expect(d.M).toHaveLength(d.n);
    expect(d.eom).toHaveLength(d.n);
    expect(core.param_count()).toBe(paramVector(mech).length);
  });

  it('derives the textbook cart-pole equations', () => {
    const d = build(encode(PRESETS['Cart-pole']!()));
    expect(d.M[0]?.[0]).toBe('m_{1} + m_{2}');
    expect(d.M[0]?.[1]).toBe('- m_{2} r_{2} c_{2}');
    expect(d.gamma).toEqual([{ i: 1, j: 2, k: 2, tex: 'm_{2} r_{2} s_{2}' }]);
  });

  it('rejects an invalid description', () => {
    expect(() => build([1, 0, 9.81, -1, 0])).toThrow(/version/);
    expect(() => build([3, 0, 9.81, -1, 0])).toThrow(/bodies/);
  });

  it('satisfies M q̈ + C q̇ + N = τ in the published snapshot', () => {
    build(encode(PRESETS['Two-link arm']!()));
    core.set_mode(3);
    core.step(250);
    const s = snapshot();
    for (let i = 0; i < 2; i++) {
      const lhs = s.v('MQDD', i) + s.v('CQD', i) + s.v('N', i);
      expect(lhs).toBeCloseTo(s.v('TAU', i), 9);
    }
    expect(s.v('SKEW')).toBeLessThan(1e-12);
  });

  it('balances the cart-pole with LQR', () => {
    build(encode(PRESETS['Cart-pole']!()));
    core.set_mode(4);
    core.set_state(1, 0.2, 0);
    core.step(6000);
    const s = snapshot();
    expect(Math.abs(s.v('Q', 1))).toBeLessThan(1e-3);
    expect(s.v('PHASE')).toBe(2);
  });
});
