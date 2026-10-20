/// <reference types="node" />
/** Exercises the real wasm core through the same ABI the worker uses. Run `make wasm` first. */
import { readFile } from 'node:fs/promises';
import { beforeAll, describe, expect, it } from 'vitest';
import { arm2, cartpole } from '../generated';
import { arm2System } from '../systems/arm2';
import { cartpoleSystem } from '../systems/cartpole';
import { type Layout, parseLayout, Snapshot } from './layout';

interface Core {
  memory: WebAssembly.Memory;
  init(system: number): number;
  reset(): void;
  step(n: number): void;
  set_mode(m: number): void;
  set_param(i: number, v: number): void;
  set_state(i: number, q: number, qd: number): void;
  param_count(): number;
  out_ptr(): number;
  out_len(): number;
  layout_ptr(): number;
  layout_len(): number;
}

let core: Core;
let layout: Layout;

function snapshot(nq: number): Snapshot {
  const ptr = core.out_ptr();
  const s = new Snapshot(layout, nq);
  s.data.set(new Float64Array(core.memory.buffer, ptr, layout.len));
  return s;
}

beforeAll(async () => {
  const bytes = await readFile(new URL('../wasm/meowfer_core.wasm', import.meta.url));
  const { instance } = await WebAssembly.instantiate(bytes, {});
  core = instance.exports as unknown as Core;
  const ptr = core.layout_ptr();
  const len = core.layout_len();
  layout = parseLayout(new TextDecoder().decode(new Uint8Array(core.memory.buffer, ptr, len)));
});

describe('wasm core', () => {
  it('exposes every field and flag the UI relies on', () => {
    expect(layout.len).toBe(core.out_len());
  });

  it('agrees with the generated parameter lists and the UI mapping', () => {
    for (const [sys, model] of [
      [cartpoleSystem, cartpole],
      [arm2System, arm2],
    ] as const) {
      core.init(sys.id);
      expect(core.param_count()).toBe(model.params.length);
      const ui = Object.fromEntries(sys.params.map((p) => [p.key, p.value]));
      expect(sys.modelParams(ui)).toHaveLength(model.params.length);
    }
  });

  it('satisfies M q̈ + C q̇ + N = τ in the published snapshot', () => {
    core.init(1);
    core.set_mode(3);
    core.step(250);
    const s = snapshot(2);
    for (let i = 0; i < 2; i++) {
      const lhs = s.v('MQDD', i) + s.v('CQD', i) + s.v('N', i);
      expect(lhs).toBeCloseTo(s.v('TAU', i), 9);
    }
    expect(s.v('SKEW')).toBeLessThan(1e-12);
  });

  it('balances the cart-pole with LQR', () => {
    core.init(0);
    core.set_mode(1);
    core.set_state(1, 0.2, 0);
    core.step(6000);
    const s = snapshot(2);
    expect(Math.abs(s.v('Q', 1))).toBeLessThan(1e-3);
    expect(s.v('PHASE')).toBe(2);
  });
});
