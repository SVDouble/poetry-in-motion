/// <reference types="node" />
/** Exercises the real wasm core through the same ABI the worker uses. Run `make wasm` first. */
import { readFile } from 'node:fs/promises';
import { beforeAll, describe, expect, it } from 'vitest';
import { LABS } from '../control/lab';
import { pageContext } from '../control/page';
import { PAGES } from '../control/pages';
import { ControlStore } from '../control/store';
import type { Derivation } from '../model/derivation';
import { dof, encode, PRESETS, paramVector } from '../model/description';
import { presets } from '../scope/presets';
import { Signals } from '../scope/signals';
import type { SimClient } from './client';
import { type Controllers, parseControllers } from './controllers';
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
  set_controller(slot: number, kind: number): void;
  set_ctrl_param(kind: number, i: number, v: number): void;
  controllers(): number;
  controllers_ptr(): number;
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

function controllers(): Controllers {
  const len = core.controllers();
  return parseControllers(text(core.controllers_ptr(), len));
}

/** Id of the controller with spec key `key`. */
const kind = (key: string) => controllers().specs.find((s) => s.key === key)!.id;

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
    core.set_controller(0, kind('computed_torque'));
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
    core.set_controller(0, kind('lqr'));
    core.set_state(1, 0.2, 0);
    core.step(6000);
    const s = snapshot();
    expect(Math.abs(s.v('Q', 1))).toBeLessThan(1e-3);
    expect(s.v('KIND')).toBe(kind('lqr'));
  });

  it('describes every controller and where it applies', () => {
    build(encode(PRESETS['Cart-pole']!()));
    const reg = controllers();
    expect(reg.specs.map((s) => s.id)).toEqual(reg.specs.map((_, i) => i));
    expect(reg.params.map((p) => p.length)).toEqual(reg.specs.map((s) => s.params.length));
    expect(reg.applicable[kind('swing_up')]).toBeNull();
    expect(reg.applicable[kind('computed_torque')]).toMatch(/actuator/);
    for (const s of reg.specs) {
      const used = Object.values(s.telemetry).reduce((n, [, len]) => n + len, 0);
      expect(used).toBeLessThanOrEqual(layout.slotLen - layout.slot.TELEMETRY);
    }
  });

  it('runs a shadow independently and reports it in its slot', () => {
    build(encode(PRESETS['Two-link arm']!()));
    core.set_controller(0, kind('pd'));
    core.set_controller(1, kind('pd_gravity'));
    core.set_controller(2, -1);
    core.step(5000);
    const s = snapshot();
    const [primary, shadow, off] = [s.slots[0]!, s.slots[1]!, s.slots[2]!];
    expect(primary.kind).toBe(kind('pd'));
    expect(shadow.kind).toBe(kind('pd_gravity'));
    expect(off.on).toBe(false);
    expect(shadow.v('ERR_Q')).toBeLessThan(primary.v('ERR_Q'));
    expect(primary.v('Q', 0)).toBe(s.v('Q', 0));
    core.set_controller(1, -1);
  });

  it.each(Object.keys(PRESETS))('names only signals and controls that exist (%s)', (name) => {
    const d = build(encode(PRESETS[name]!()));
    const reg = controllers();
    const store = new ControlStore({ send: () => undefined } as unknown as SimClient, layout.slots);
    store.load(reg, 0);
    const snap = snapshot();
    const signals = new Signals(d, snap, store);
    const missing = (ids: string[]) => ids.filter((id) => !signals.resolve(id));
    for (const spec of reg.specs) {
      expect(PAGES[spec.key], spec.key).toBeDefined();
      if (reg.applicable[spec.id] !== null) continue;
      const ctx = pageContext(d, snap, spec, (i) => reg.params[spec.id]![i]!);
      const lab = LABS[spec.key]!(ctx);
      expect(missing(lab.watch.flatMap((p) => p.ids)), `${spec.key} watch`).toEqual([]);
      for (const e of lab.experiments) {
        expect(missing(e.plots.flatMap((p) => p.ids)), `${spec.key}: ${e.title}`).toEqual([]);
        for (const a of e.actions) {
          if (a.do === 'param')
            expect(
              spec.params.map((p) => p.key),
              `${spec.key}: ${e.title}`,
            ).toContain(a.key);
          if (a.do === 'shadow') expect(reg.specs.map((s) => s.key)).toContain(a.key);
        }
      }
      for (const p of presets(d, store, lab.watch, spec.name)) {
        expect(missing(p.plots.flatMap((x) => x.ids)), `preset ${p.key}`).toEqual([]);
      }
    }
    for (const g of signals.groups()) expect(missing(g.ids)).toEqual([]);
  });
});
