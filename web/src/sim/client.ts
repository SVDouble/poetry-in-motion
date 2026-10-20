import { type Flag, type Layout, parseLayout, Snapshot } from './layout';
import { DATA_OFFSET, type FromWorker, STATS, STATS_OFFSET, type ToWorker } from './protocol';

export interface SimStats {
  /** Wall-clock cost of one RK4 step including control, µs. */
  stepUs: number;
  stepsPerSecond: number;
  realtime: number;
  running: boolean;
  shared: boolean;
}

/** UI-side handle to the physics worker. `pull()` copies the newest consistent snapshot. */
export class SimClient {
  readonly stats: SimStats = { stepUs: 0, stepsPerSecond: 0, realtime: 1, running: true, shared: false };
  private snap: Snapshot | null = null;
  private shared: { seq: Int32Array; stats: Float64Array; data: Float64Array } | null = null;
  private latest: { data: Float64Array; stats: Float64Array } | null = null;
  private lastSeq = -1;

  private constructor(
    private readonly worker: Worker,
    readonly layout: Layout,
    sab: SharedArrayBuffer | null,
  ) {
    if (sab) {
      this.shared = {
        seq: new Int32Array(sab, 0, 1),
        stats: new Float64Array(sab, STATS_OFFSET, STATS),
        data: new Float64Array(sab, DATA_OFFSET, layout.len),
      };
      this.stats.shared = true;
    }
    worker.addEventListener('message', (e: MessageEvent<FromWorker>) => {
      if (e.data.type === 'snapshot') this.latest = { data: e.data.data, stats: e.data.stats };
    });
  }

  static start(): Promise<SimClient> {
    const worker = new Worker(new URL('./worker.ts', import.meta.url), { type: 'module' });
    return new Promise((resolve, reject) => {
      const onMessage = (e: MessageEvent<FromWorker>) => {
        if (e.data.type === 'ready') {
          worker.removeEventListener('message', onMessage);
          resolve(new SimClient(worker, parseLayout(e.data.layout), e.data.shared));
        } else if (e.data.type === 'error') {
          reject(new Error(e.data.message));
        }
      };
      worker.addEventListener('message', onMessage);
      worker.addEventListener('error', (e) => reject(new Error(e.message)));
    });
  }

  send(msg: ToWorker): void {
    this.worker.postMessage(msg);
  }

  flag(f: Flag, v: number): void {
    this.send({ type: 'flag', id: this.layout.flag[f], v });
  }

  /** (Re)initialise the worker with a system; returns a fresh snapshot reader. */
  init(system: number, nq: number, params: number[], mode: number, flags: [Flag, number][]): Snapshot {
    this.send({ type: 'init', system, params, mode, flags: flags.map(([f, v]) => [this.layout.flag[f], v]) });
    this.snap = new Snapshot(this.layout, nq);
    this.lastSeq = -1;
    return this.snap;
  }

  /** Copy the newest snapshot into the reader. Returns true if it changed. */
  pull(): boolean {
    const snap = this.snap;
    if (!snap) return false;
    if (this.shared) {
      const { seq, data, stats } = this.shared;
      for (let attempt = 0; attempt < 4; attempt++) {
        const s0 = Atomics.load(seq, 0);
        if (s0 & 1) continue; // writer active
        if (s0 === this.lastSeq) return false;
        snap.data.set(data);
        this.readStats(stats);
        if (Atomics.load(seq, 0) === s0) {
          this.lastSeq = s0;
          return true;
        }
      }
      return false;
    }
    if (!this.latest) return false;
    snap.data.set(this.latest.data);
    this.readStats(this.latest.stats);
    this.latest = null;
    return true;
  }

  private readStats(s: Float64Array): void {
    this.stats.stepUs = s[0] ?? 0;
    this.stats.stepsPerSecond = s[1] ?? 0;
    this.stats.realtime = s[2] ?? 0;
    this.stats.running = (s[3] ?? 0) !== 0;
  }
}
