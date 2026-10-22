/// <reference types="node" />
/**
 * The real wasm core for tests (Node): load it, build mechanisms, read snapshots, and dispatch the same
 * `ToWorker` messages the physics worker handles. Run `make wasm` first.
 */
import { readFile } from 'node:fs/promises';
import type { Derivation } from '../model/derivation';
import { type Controllers, parseControllers } from './controllers';
import type { Core } from './core';
import { type Flag, type Layout, parseLayout, Snapshot } from './layout';
import type { ToWorker } from './protocol';

export class TestCore {
  private constructor(
    readonly core: Core,
    readonly layout: Layout,
  ) {}

  static async load(): Promise<TestCore> {
    const bytes = await readFile(new URL('../wasm/motion_core.wasm', import.meta.url));
    const { instance } = await WebAssembly.instantiate(bytes, {});
    const core = instance.exports as unknown as Core;
    const text = (ptr: number, len: number) =>
      new TextDecoder().decode(new Uint8Array(core.memory.buffer, ptr, len).slice());
    return new TestCore(core, parseLayout(text(core.layout_ptr(), core.layout_len())));
  }

  /** Call into wasm first: a call may grow memory and detach older views. */
  text(ptr: number, len: number): string {
    return new TextDecoder().decode(new Uint8Array(this.core.memory.buffer, ptr, len).slice());
  }

  build(desc: number[]): Derivation {
    const c = this.core;
    const ptr = c.desc_buffer(desc.length);
    new Float64Array(c.memory.buffer, ptr, desc.length).set(desc);
    if (c.build(desc.length) !== 0) throw new Error(this.text(c.error_ptr(), c.error_len()));
    return JSON.parse(this.text(c.derivation_ptr(), c.derivation_len())) as Derivation;
  }

  controllers(): Controllers {
    const len = this.core.controllers();
    return parseControllers(this.text(this.core.controllers_ptr(), len));
  }

  /** Copy the current snapshot into `into` (or a new one). */
  snapshot(into = new Snapshot(this.layout)): Snapshot {
    this.core.publish();
    into.data.set(new Float64Array(this.core.memory.buffer, this.core.out_ptr(), this.layout.len));
    return into;
  }

  /** Advance a physical duration, sampling in batches and reporting actual simulated seconds.
   * A custom dt must already have been applied with core.set_dt; the final sample can overshoot by < dt. */
  advance(ms: number, sample: (seconds: number) => void, dt = this.layout.dt, sampleMs = 10): void {
    let left = Math.ceil(ms / (1000 * dt) - 1e-9);
    const batch = Math.max(1, Math.round(sampleMs / (1000 * dt)));
    while (left > 0) {
      const steps = Math.min(batch, left);
      this.core.step(steps);
      sample(steps * dt);
      left -= steps;
    }
  }

  flag(f: Flag, v: number): void {
    this.core.set_flag(this.layout.flag[f], v);
  }

  /** What the physics worker does with a message. */
  send(m: ToWorker): void {
    const c = this.core;
    switch (m.type) {
      case 'param':
        c.set_param(m.i, m.v);
        return;
      case 'controller':
        c.set_controller(m.slot, m.kind);
        return;
      case 'ctrlParam':
        c.set_ctrl_param(m.kind, m.i, m.v);
        return;
      case 'sync':
        c.sync_shadows();
        return;
      case 'input':
        c.set_input(m.i, m.v);
        return;
      case 'flag':
        c.set_flag(m.id, m.v);
        return;
      case 'target':
        c.set_target(m.i, m.v);
        return;
      case 'kick':
        c.kick(m.i, m.v);
        return;
      case 'state':
        c.set_state(m.i, m.q, m.qd);
        return;
      case 'reset':
        c.reset();
        return;
      case 'advance':
        c.step(m.steps);
        return;
      default: // build, run, speed: not used by tests through this path
    }
  }
}

export type { Core } from './core';
