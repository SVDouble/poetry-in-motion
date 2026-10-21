/// <reference lib="webworker" />
/**
 * Physics worker: hosts the Rust/WASM core, derives mechanisms on request, runs the simulation on a fixed
 * 1 kHz clock in real time and publishes snapshots through a SharedArrayBuffer (seqlock), or by message
 * when cross-origin isolation is missing.
 */
import wasmUrl from '../wasm/motion_core.wasm?url';
import { DATA_OFFSET, type FromWorker, STATS, STATS_OFFSET, type ToWorker } from './protocol';

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
  bench(n: number): void;
  publish(): void;
  set_dt(dt: number): void;
  set_param(i: number, v: number): void;
  set_controller(slot: number, kind: number): void;
  set_ctrl_param(kind: number, i: number, v: number): void;
  sync_shadows(): void;
  controllers(): number;
  controllers_ptr(): number;
  set_target(i: number, v: number): void;
  set_input(i: number, v: number): void;
  set_flag(id: number, v: number): void;
  set_state(i: number, q: number, qd: number): void;
  kick(i: number, v: number): void;
  out_ptr(): number;
  out_len(): number;
  layout_ptr(): number;
  layout_len(): number;
}

const scope = self as unknown as DedicatedWorkerGlobalScope;
const post = (msg: FromWorker, transfer: Transferable[] = []) => scope.postMessage(msg, transfer);

const DT = 1e-3;
const TICK_MS = 4;
const MAX_CATCH_UP_S = 0.05;
const FALLBACK_POST_MS = 16;
/** Time budget of one step-cost measurement: it blocks the clock, so it must stay well inside a tick (a
 * fixed 20 000 steps took 300 ms on a seven-joint arm and stalled the simulation every two seconds). */
const BENCH_MS = 2;
const BENCH_EVERY_MS = 2000;

let core: Core | null = null;
let running = false;
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
    Atomics.add(shared.seq, 0, 1);
    shared.stats.set(stats);
    shared.data.set(out);
    Atomics.add(shared.seq, 0, 1);
  } else {
    const now = performance.now();
    if (now - lastPost < FALLBACK_POST_MS) return;
    lastPost = now;
    const data = out.slice();
    const s = stats.slice();
    post({ type: 'snapshot', data, stats: s }, [data.buffer, s.buffer]);
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
    const n = Math.floor(acc / DT);
    acc -= n * DT;
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

function build(c: Core, id: number, desc: number[]): void {
  const t0 = performance.now();
  const ptr = c.desc_buffer(desc.length);
  new Float64Array(c.memory.buffer, ptr, desc.length).set(desc);
  if (c.build(desc.length) === 0) {
    c.set_dt(DT);
    const buildMs = performance.now() - t0;
    const derivation = text(c, c.derivation_ptr(), c.derivation_len());
    const len = c.controllers();
    const controllers = text(c, c.controllers_ptr(), len);
    post({ type: 'built', id, ok: true, derivation, controllers, buildMs });
    lastBench = -Infinity;
    benchSteps = 50;
    stepCost = 0;
  } else {
    post({ type: 'built', id, ok: false, error: text(c, c.error_ptr(), c.error_len()) });
  }
}

type Handlers = { [K in ToWorker['type']]: (c: Core, msg: Extract<ToWorker, { type: K }>) => void };

const handlers: Handlers = {
  build: (c, m) => build(c, m.id, m.desc),
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
  run: (_c, m) => {
    running = m.running;
  },
  speed: (_c, m) => {
    speed = m.speed;
  },
};

function handle(c: Core, msg: ToWorker): void {
  (handlers[msg.type] as (c: Core, m: ToWorker) => void)(c, msg);
}

const queue: ToWorker[] = [];
scope.onmessage = (e: MessageEvent<ToWorker>) => {
  if (core) handle(core, e.data);
  else queue.push(e.data);
};

load()
  .then((c) => {
    core = c;
    c.set_dt(DT);
    const layout = text(c, c.layout_ptr(), c.layout_len());
    let sab: SharedArrayBuffer | null = null;
    if (scope.crossOriginIsolated) {
      sab = new SharedArrayBuffer(DATA_OFFSET + c.out_len() * 8);
      shared = {
        seq: new Int32Array(sab, 0, 1),
        stats: new Float64Array(sab, STATS_OFFSET, STATS),
        data: new Float64Array(sab, DATA_OFFSET, c.out_len()),
      };
    }
    post({ type: 'ready', layout, shared: sab });
    for (const m of queue.splice(0)) handle(c, m);
    setInterval(tick, TICK_MS);
  })
  .catch((err: unknown) => post({ type: 'error', message: String(err) }));
