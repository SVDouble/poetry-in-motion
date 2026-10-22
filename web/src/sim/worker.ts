/// <reference lib="webworker" />
/**
 * Physics worker: hosts the Rust/WASM core, derives mechanisms on request, runs the simulation on a fixed
 * 1 kHz clock in real time and publishes snapshots through a SharedArrayBuffer (seqlock), or by message
 * when cross-origin isolation is missing.
 */
import wasmUrl from '../wasm/motion_core.wasm?url';
import { parseLayout } from './layout';
import { DATA_OFFSET, type FromWorker, STATS, STATS_OFFSET, type ToWorker } from './protocol';

interface Core {
  memory: WebAssembly.Memory;
  desc_buffer(len: number): number;
  prepare(len: number): number;
  commit_build(): number;
  discard_build(): void;
  error_ptr(): number;
  error_len(): number;
  derivation_ptr(): number;
  derivation_len(): number;
  reset(): void;
  step(n: number): void;
  bench(n: number): void;
  publish(): void;
  set_param(i: number, v: number): number;
  set_controller(slot: number, kind: number): void;
  set_ctrl_param(kind: number, i: number, v: number): number;
  param_value(i: number): number;
  sync_shadows(): void;
  controllers(): number;
  controllers_ptr(): number;
  set_target(i: number, v: number): number;
  set_input(i: number, v: number): number;
  set_flag(id: number, v: number): number;
  set_state(i: number, q: number, qd: number): number;
  kick(i: number, v: number): number;
  out_ptr(): number;
  out_len(): number;
  layout_ptr(): number;
  layout_len(): number;
}

const scope = self as unknown as DedicatedWorkerGlobalScope;
const post = (msg: FromWorker, transfer: Transferable[] = []) => scope.postMessage(msg, transfer);

/** The core's fixed step (s), read from its layout. */
let dt: number;
const TICK_MS = 4;
const MAX_CATCH_UP_S = 0.05;
const FALLBACK_POST_MS = 16;
/** Time budget of one step-cost measurement: it blocks the clock, so it must stay well inside a tick (a
 * fixed 20 000 steps took 300 ms on a seven-joint arm and stalled the simulation every two seconds). */
const BENCH_MS = 2;
const BENCH_EVERY_MS = 2000;

let core: Core | null = null;
let running = false;
let generation = 0;
let sequence = 0;
const nextSequence = () => {
  sequence = (sequence + 2) >>> 0;
  return sequence;
};
let candidate = 0;
let speed = 1;
let acc = 0;
let last = performance.now();
let lastPost = 0;
const stats = new Float64Array(STATS);
let stepCost = 0;
let lastBench = -Infinity;
let benchSteps = 50;
let stepsThisSecond = 0;
let secondStart = performance.now();

let shared: { seq: Int32Array; stats: Float64Array; data: Float64Array } | null = null;

/** Read a UTF-8 string from wasm memory. Call into wasm *before* touching memory.buffer: a call may grow
 * memory and detach older buffers. */
function text(c: Core, ptr: number, len: number): string {
  return new TextDecoder().decode(new Uint8Array(c.memory.buffer, ptr, len).slice());
}

function view(c: Core): Float64Array {
  const ptr = c.out_ptr();
  const len = c.out_len();
  return new Float64Array(c.memory.buffer, ptr, len);
}

async function load(): Promise<Core> {
  const { instance } = await WebAssembly.instantiateStreaming(fetch(wasmUrl), {});
  return instance.exports as unknown as Core;
}

function publish(c: Core): void {
  const out = view(c);
  if (shared) {
    const seq = nextSequence();
    Atomics.store(shared.seq, 0, seq - 1);
    Atomics.store(shared.seq, 1, generation);
    shared.stats.set(stats);
    shared.data.set(out);
    Atomics.store(shared.seq, 0, seq);
  } else {
    const now = performance.now();
    if (now - lastPost < FALLBACK_POST_MS) return;
    lastPost = now;
    const data = out.slice();
    const s = stats.slice();
    post({ type: 'snapshot', sequence: nextSequence(), generation, data, stats: s }, [data.buffer, s.buffer]);
  }
}

function tick(): void {
  const c = core;
  if (!c) return;
  const now = performance.now();
  const elapsed = Math.min((now - last) / 1000, MAX_CATCH_UP_S);
  last = now;
  if (running) {
    acc += elapsed * speed;
    const n = Math.floor(acc / dt);
    acc -= n * dt;
    if (n > 0) {
      c.step(n);
      stepsThisSecond += n;
    } else {
      c.publish();
    }
  } else {
    c.publish();
  }
  if (now - secondStart >= 1000) {
    stats[1] = (stepsThisSecond * 1000) / (now - secondStart);
    stepsThisSecond = 0;
    secondStart = now;
  }
  if (now - lastBench > BENCH_EVERY_MS) {
    // a few real-time steps are too short to time with the coarse clock, so time a batch on a copy, sized
    // from the last measurement to take about BENCH_MS
    const t0 = performance.now();
    c.bench(benchSteps);
    const ms = performance.now() - t0;
    // smoothed: a 2 ms window still catches the odd GC pause or preemption
    if (ms > 0)
      stepCost = stepCost > 0 ? 0.8 * stepCost + 0.2 * ((ms * 1000) / benchSteps) : (ms * 1000) / benchSteps;
    benchSteps = Math.round(Math.min(20_000, Math.max(10, (BENCH_MS * 1000) / Math.max(stepCost, 0.05))));
    lastBench = now;
  }
  stats[0] = stepCost;
  stats[2] = running ? speed : 0;
  stats[3] = running ? 1 : 0;
  publish(c);
}

let buildMs = 0;
function build(c: Core, id: number, desc: number[]): void {
  const t0 = performance.now();
  const ptr = c.desc_buffer(desc.length);
  new Float64Array(c.memory.buffer, ptr, desc.length).set(desc);
  if (c.prepare(desc.length) === 0) {
    candidate = id;
    buildMs = performance.now() - t0;
    post({ type: 'prepared', id });
  } else {
    candidate = 0;
    post({ type: 'built', id, ok: false, error: text(c, c.error_ptr(), c.error_len()) });
  }
}

function commit(c: Core, id: number): void {
  if (candidate !== id) {
    post({ type: 'built', id, ok: false, error: 'Build superseded by a newer revision' });
    return;
  }
  candidate = 0;
  if (c.commit_build() !== 0) {
    post({ type: 'built', id, ok: false, error: text(c, c.error_ptr(), c.error_len()) });
    return;
  }
  generation = id;
  const derivation = text(c, c.derivation_ptr(), c.derivation_len());
  const len = c.controllers();
  const controllers = text(c, c.controllers_ptr(), len);
  const data = view(c).slice(),
    s = stats.slice();
  post(
    {
      type: 'built',
      id,
      ok: true,
      sequence: nextSequence(),
      generation,
      data,
      stats: s,
      derivation,
      controllers,
      buildMs,
    },
    [data.buffer, s.buffer],
  );
  acc = 0;
  last = performance.now();
  lastBench = -Infinity;
  benchSteps = 50;
  stepCost = 0;
}

type Handlers = { [K in ToWorker['type']]: (c: Core, msg: Extract<ToWorker, { type: K }>) => unknown };

const handlers: Handlers = {
  build: (c, m) => build(c, m.id, m.desc),
  commitBuild: (c, m) => commit(c, m.id),
  discardBuild: (c, m) => {
    if (candidate === m.id) {
      c.discard_build();
      candidate = 0;
    }
  },
  observe: (c, m) => {
    c.publish();
    const data = view(c).slice(),
      s = stats.slice();
    post({ type: 'observed', id: m.id, sequence: nextSequence(), generation, data, stats: s }, [
      data.buffer,
      s.buffer,
    ]);
  },
  param: (c, m) => c.set_param(m.i, m.v),
  controller: (c, m) => c.set_controller(m.slot, m.kind),
  ctrlParam: (c, m) => c.set_ctrl_param(m.kind, m.i, m.v),
  sync: (c) => c.sync_shadows(),
  input: (c, m) => c.set_input(m.i, m.v),
  flag: (c, m) => c.set_flag(m.id, m.v),
  target: (c, m) => c.set_target(m.i, m.v),
  kick: (c, m) => c.kick(m.i, m.v),
  state: (c, m) => c.set_state(m.i, m.q, m.qd),
  reset: (c) => c.reset(),
  advance: (c, m) => {
    c.step(Math.max(1, Math.min(1000, Math.round(m.steps))));
    c.publish();
  },
  run: (_c, m) => {
    running = m.running;
  },
  speed: (_c, m) => {
    speed = m.speed;
  },
};

function handle(c: Core, msg: ToWorker): void {
  if (
    !['build', 'commitBuild', 'discardBuild', 'observe', 'run', 'speed'].includes(msg.type) &&
    msg.generation !== generation
  )
    return;
  const index = (v: number, off = false) => Number.isInteger(v) && v >= (off ? -1 : 0) && v <= 0xffffffff;
  if (
    ('i' in msg && !index(msg.i)) ||
    ('slot' in msg && !index(msg.slot)) ||
    ('kind' in msg && !index(msg.kind, msg.type === 'controller')) ||
    (msg.type === 'flag' && !index(msg.id)) ||
    (msg.type === 'advance' && (!Number.isInteger(msg.steps) || msg.steps < 1)) ||
    (msg.type === 'speed' && (!Number.isFinite(msg.speed) || msg.speed <= 0))
  ) {
    post({ type: 'invalid', message: 'Invalid simulation command index or step/speed value' });
    return;
  }
  const status = (handlers[msg.type] as (c: Core, m: ToWorker) => unknown)(c, msg);
  if (typeof status === 'number' && status < 0) {
    const value = msg.type === 'param' ? c.param_value(msg.i) : Number.NaN;
    post({
      type: 'invalid',
      message: text(c, c.error_ptr(), c.error_len()),
      ...(msg.type === 'param' && Number.isFinite(value) ? { param: { i: msg.i, value } } : {}),
    });
  }
}

const queue: ToWorker[] = [];
scope.onmessage = (e: MessageEvent<ToWorker>) => {
  if (core) {
    try {
      handle(core, e.data);
    } catch (error) {
      running = false;
      post({ type: 'error', message: String(error) });
    }
  } else queue.push(e.data);
};

load()
  .then((c) => {
    const layout = text(c, c.layout_ptr(), c.layout_len());
    dt = parseLayout(layout).dt;
    core = c;
    let sab: SharedArrayBuffer | null = null;
    if (scope.crossOriginIsolated) {
      sab = new SharedArrayBuffer(DATA_OFFSET + c.out_len() * 8);
      shared = {
        seq: new Int32Array(sab, 0, 2),
        stats: new Float64Array(sab, STATS_OFFSET, STATS),
        data: new Float64Array(sab, DATA_OFFSET, c.out_len()),
      };
    }
    post({ type: 'ready', layout, shared: sab });
    for (const m of queue.splice(0)) handle(c, m);
    setInterval(() => {
      try {
        tick();
      } catch (error) {
        running = false;
        post({ type: 'error', message: String(error) });
      }
    }, TICK_MS);
  })
  .catch((err: unknown) => post({ type: 'error', message: String(err) }));
