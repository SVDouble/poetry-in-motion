/// <reference lib="webworker" />
/**
 * Physics worker: hosts the Rust/WASM core, runs it on a fixed 1 kHz clock in real time and publishes
 * snapshots through a SharedArrayBuffer (seqlock), or by message when cross-origin isolation is missing.
 */
import wasmUrl from '../wasm/meowfer_core.wasm?url';
import { DATA_OFFSET, type FromWorker, STATS, STATS_OFFSET, type ToWorker } from './protocol';

interface Core {
  memory: WebAssembly.Memory;
  init(system: number): number;
  reset(): void;
  step(n: number): void;
  bench(n: number): void;
  publish(): void;
  set_dt(dt: number): void;
  set_param(i: number, v: number): void;
  set_mode(m: number): void;
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

let core: Core | null = null;
let running = true;
let speed = 1;
let acc = 0;
let last = performance.now();
let lastPost = 0;
const stats = new Float64Array(STATS);
let stepCost = 0;
let lastBench = -Infinity;
const BENCH_STEPS = 20_000;
const BENCH_EVERY_MS = 2000;
let stepsThisSecond = 0;
let secondStart = performance.now();

let shared: { seq: Int32Array; stats: Float64Array; data: Float64Array } | null = null;

function view(c: Core): Float64Array {
  // Call into wasm *before* touching memory.buffer: a call may grow memory and detach older buffers.
  const ptr = c.out_ptr();
  const len = c.out_len();
  return new Float64Array(c.memory.buffer, ptr, len);
}

async function load(): Promise<Core> {
  const { instance } = await WebAssembly.instantiateStreaming(fetch(wasmUrl), {});
  return instance.exports as unknown as Core;
}

function publish(c: Core): void {
  const out = view(c); // re-created each time: memory growth detaches old views
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
    // a few real time steps are too short to time with the coarse clock, so time a batch on a copy
    const t0 = performance.now();
    c.bench(BENCH_STEPS);
    stepCost = ((performance.now() - t0) * 1000) / BENCH_STEPS;
    lastBench = now;
  }
  stats[0] = stepCost;
  stats[2] = running ? speed : 0;
  stats[3] = running ? 1 : 0;
  publish(c);
}

function handle(c: Core, msg: ToWorker): void {
  switch (msg.type) {
    case 'init': {
      c.init(msg.system);
      c.set_dt(DT);
      msg.params.forEach((v, i) => {
        c.set_param(i, v);
      });
      for (const [id, v] of msg.flags) c.set_flag(id, v);
      c.reset();
      c.set_mode(msg.mode);
      c.publish();
      break;
    }
    case 'param':
      c.set_param(msg.i, msg.v);
      break;
    case 'mode':
      c.set_mode(msg.mode);
      break;
    case 'input':
      c.set_input(msg.i, msg.v);
      break;
    case 'flag':
      c.set_flag(msg.id, msg.v);
      break;
    case 'kick':
      c.kick(msg.i, msg.v);
      break;
    case 'state':
      c.set_state(msg.i, msg.q, msg.qd);
      break;
    case 'reset':
      c.reset();
      break;
    case 'run':
      running = msg.running;
      break;
    case 'speed':
      speed = msg.speed;
      break;
  }
}

const queue: ToWorker[] = [];
scope.onmessage = (e: MessageEvent<ToWorker>) => {
  if (core) handle(core, e.data);
  else queue.push(e.data);
};

load()
  .then((c) => {
    core = c;
    const ptr = c.layout_ptr();
    const len = c.layout_len();
    const bytes = new Uint8Array(c.memory.buffer, ptr, len);
    const layout = new TextDecoder().decode(bytes);
    let sab: SharedArrayBuffer | null = null;
    if (scope.crossOriginIsolated) {
      sab = new SharedArrayBuffer(DATA_OFFSET + c.out_len() * 8);
      shared = {
        seq: new Int32Array(sab, 0, 1),
        stats: new Float64Array(sab, STATS_OFFSET, STATS),
        data: new Float64Array(sab, DATA_OFFSET, c.out_len()),
      };
    }
    for (const m of queue.splice(0)) handle(c, m);
    post({ type: 'ready', layout, shared: sab });
    setInterval(tick, TICK_MS);
  })
  .catch((err: unknown) => post({ type: 'error', message: String(err) }));
