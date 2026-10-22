import { beforeAll, describe, expect, it, vi } from 'vitest';
import { Revisions } from '../app/revision';
import { cloneMechanism, PRESETS } from '../model/description';
import { type Built, SimClient } from './client';
import type { Layout } from './layout';
import {
  DATA_OFFSET,
  type FromWorker,
  newerSequence,
  type Observation,
  STATS,
  STATS_OFFSET,
  type ToWorker,
} from './protocol';
import { TestCore } from './testing';

class FakeWorker extends EventTarget {
  sent: ToWorker[] = [];
  terminated = false;
  postMessage(msg: ToWorker): void {
    this.sent.push(msg);
  }
  terminate(): void {
    this.terminated = true;
  }
  receive(msg: FromWorker): void {
    this.dispatchEvent(new MessageEvent('message', { data: msg }));
  }
}

let layout: Layout;
let derivation: string;
let controllers: string;
beforeAll(async () => {
  const tc = await TestCore.load();
  layout = tc.layout;
  derivation = tc.text(tc.core.derivation_ptr(), tc.core.derivation_len());
  controllers = JSON.stringify(tc.controllers());
});

let sequence = 0;
function observation(generation: number, value: number): Observation {
  sequence += 2;
  return {
    sequence,
    generation,
    data: new Float64Array(layout.len).fill(value),
    stats: new Float64Array(STATS).fill(value),
  };
}
function reply(id: number, value = 1): Extract<FromWorker, { type: 'built'; ok: true }> {
  return { type: 'built', id, ok: true, ...observation(id, value), derivation, controllers, buildMs: 1 };
}
async function installed(worker: FakeWorker, client: SimClient, value = 1): Promise<Built> {
  const promise = client.build([]);
  const id = (worker.sent.at(-1) as Extract<ToWorker, { type: 'build' }>).id;
  worker.receive({ type: 'prepared', id });
  expect(worker.sent.at(-1)?.type).toBe('commitBuild');
  worker.receive(reply(id, value));
  const result = await promise;
  if (!result.ok) throw Error(result.error);
  client.install(result);
  return result;
}

describe('physics client transitions', () => {
  it('orders sequence counters across unsigned wrap and rejects old or duplicate samples', () => {
    expect(newerSequence(0, 0xfffffffe)).toBe(true);
    expect(newerSequence(2, 0)).toBe(true);
    expect(newerSequence(0xfffffffe, 2)).toBe(false);
    expect(newerSequence(2, 2)).toBe(false);
  });

  it('rejects a prepared result invalidated during the debounce window', async () => {
    const worker = new FakeWorker();
    const client = new SimClient(worker as unknown as Worker, layout, null);
    const revisions = new Revisions(cloneMechanism);
    const document = PRESETS['Two-link arm']!();
    const ticket = revisions.capture(document);
    const pending = client.build([], ticket.current);
    const id = (worker.sent.at(-1) as Extract<ToWorker, { type: 'build' }>).id;
    revisions.change(); // edit B, before B's build timer fires
    document.bodies.pop();
    worker.receive({ type: 'prepared', id });
    expect((await pending).ok).toBe(false);
    expect(ticket.document.bodies).toHaveLength(2);
    expect(worker.sent.at(-1)?.type).toBe('discardBuild');
    expect(worker.sent.some((m) => m.type === 'commitBuild')).toBe(false);
    expect(client.generation).toBe(0);
  });

  it('installs coherent initial observations and ignores obsolete message snapshots', async () => {
    const worker = new FakeWorker();
    const client = new SimClient(worker as unknown as Worker, layout, null);
    const first = await installed(worker, client);
    worker.receive({ type: 'snapshot', ...observation(first.generation + 1, 9) });
    expect(client.pull()).toBe(false);
    expect(client.snap.data[0]).toBe(1);
    const second = await installed(worker, client, 2);
    worker.receive({ type: 'snapshot', ...observation(first.generation, 7) });
    expect(client.pull()).toBe(false);
    expect(client.snap.data[0]).toBe(2);
    worker.receive({ type: 'snapshot', ...observation(second.generation, 3) });
    expect(client.pull()).toBe(true);
    expect(client.snap.data[0]).toBe(3);
    expect(client.stats.stepUs).toBe(3);
    const pending = client.build([]);
    const id = (worker.sent.at(-1) as Extract<ToWorker, { type: 'build' }>).id;
    worker.receive({ type: 'built', id, ok: false, error: 'Invalid mass' });
    expect(await pending).toEqual({ ok: false, error: 'Invalid mass' });
    expect(client.generation).toBe(second.generation);
    expect(client.snap.data[0]).toBe(3);
  });

  it('keeps visible data and stats unchanged through exhausted seqlock retries and wrong generations', async () => {
    const buffer = new SharedArrayBuffer(DATA_OFFSET + layout.len * 8);
    const seq = new Int32Array(buffer, 0, 2);
    const data = new Float64Array(buffer, DATA_OFFSET, layout.len);
    const stats = new Float64Array(buffer, STATS_OFFSET, STATS);
    const worker = new FakeWorker();
    const client = new SimClient(worker as unknown as Worker, layout, buffer);
    const initial = await installed(worker, client);
    Atomics.store(seq, 1, initial.generation);
    data.fill(1);
    stats.fill(1);
    Atomics.store(seq, 0, initial.sequence + 2);
    expect(client.pull()).toBe(true);
    data.fill(2);
    stats.fill(2);
    Atomics.store(seq, 0, initial.sequence + 4);
    const load = Atomics.load;
    let reads = 0;
    const spy = vi.spyOn(Atomics, 'load').mockImplementation((array, index) => {
      if (array.buffer === buffer && index === 0) {
        if (index === 0 && ++reads % 2 === 0) Atomics.add(seq, 0, 2);
      }
      return load(array, index);
    });
    try {
      expect(client.pull()).toBe(false);
      expect(client.snap.data[0]).toBe(1);
      expect(client.stats.stepUs).toBe(1);
    } finally {
      spy.mockRestore();
    }
    expect(client.pull()).toBe(true);
    expect(client.snap.data[0]).toBe(2);
    expect(client.stats.stepUs).toBe(2);
    Atomics.store(seq, 1, initial.generation + 1);
    data.fill(8);
    stats.fill(8);
    Atomics.add(seq, 0, 2);
    expect(client.pull()).toBe(false);
    expect(client.snap.data[0]).toBe(2);
    expect(client.stats.stepUs).toBe(2);
  });

  it('settles every request on worker failure and stops sending commands', async () => {
    const worker = new FakeWorker();
    const client = new SimClient(worker as unknown as Worker, layout, null);
    const builds = [client.build([]), client.build([])];
    const refresh = client.refresh();
    worker.receive({ type: 'error', message: 'WASM trap' });
    for (const result of await Promise.all(builds)) expect(result).toEqual({ ok: false, error: 'WASM trap' });
    expect(await refresh).toBe(false);
    expect(worker.terminated).toBe(true);
    const count = worker.sent.length;
    expect(client.send({ type: 'reset' })).toBe(false);
    expect((await client.build([])).ok).toBe(false);
    expect(worker.sent).toHaveLength(count);
  });

  it('does not roll back to an acknowledgment older than a frame already displayed', async () => {
    const worker = new FakeWorker();
    const client = new SimClient(worker as unknown as Worker, layout, null);
    const initial = await installed(worker, client);
    const waiting = client.refresh();
    const id = (worker.sent.at(-1) as Extract<ToWorker, { type: 'observe' }>).id;
    const acknowledgment = observation(initial.generation, 2);
    worker.receive({ type: 'snapshot', ...observation(initial.generation, 3) });
    expect(client.pull()).toBe(true);
    worker.receive({ type: 'observed', id, ...acknowledgment });
    expect(await waiting).toBe(true);
    expect(client.snap.data[0]).toBe(3);
    expect(client.stats.stepUs).toBe(3);
  });

  it('cancels an observation wait and ignores its late reply', async () => {
    const worker = new FakeWorker();
    const client = new SimClient(worker as unknown as Worker, layout, null);
    const abort = new AbortController();
    const waiting = client.refresh(abort.signal);
    const id = (worker.sent.at(-1) as Extract<ToWorker, { type: 'observe' }>).id;
    abort.abort();
    expect(await waiting).toBe(false);
    worker.receive({ type: 'observed', id, ...observation(0, 9) });
    expect(client.snap.data[0]).toBe(0);
  });
});
