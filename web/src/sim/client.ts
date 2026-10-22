import type { Derivation } from '../model/derivation';
import { type Controllers, parseControllers } from './controllers';
import { type Flag, type Layout, parseLayout, Snapshot } from './layout';
import { MAX_PROFILE_TICKS, type TickProfile } from './profile';
import {
  type CheckpointFrame,
  type FromWorker,
  newerSequence,
  type Observation,
  type RuntimeState,
  STATS,
  type ToWorker,
  TRANSPORT_VERSION,
} from './protocol';
import { SharedReader } from './shared';
import type { TaskGoal } from './tasks';

export interface SimStats {
  stepUs: number;
  stepsPerSecond: number;
  realtime: number;
  running: boolean;
  shared: boolean;
}

export type Built = Observation & {
  ok: true;
  derivation: Derivation;
  controllers: Controllers;
  ms: number;
  reused: boolean;
  targets: number[];
};
export type BuildResult = Built | { ok: false; error: string };

/** UI handle to one worker. A build is installed explicitly after the document revision is checked. */
export class SimClient {
  readonly stats: SimStats = { stepUs: 0, stepsPerSecond: 0, realtime: 1, running: false, shared: false };
  readonly snap: Snapshot;
  generation = 0;
  failure: string | null = null;
  private readonly shared: SharedReader | null;
  private latest: Observation | null = null;
  private nextId = 1;
  private sequence: number | null = null;
  private id(): number {
    let id: number;
    do {
      id = this.nextId;
      this.nextId = id === 0x7fffffff ? 1 : id + 1;
    } while (
      id === this.generation ||
      this.pending.has(id) ||
      this.observations.has(id) ||
      this.restores.has(id) ||
      this.profiles.has(id)
    );
    return id;
  }
  private readonly pending = new Map<number, { done: (r: BuildResult) => void; current: () => boolean }>();
  private readonly observations = new Map<number, (o: Observation | null) => void>();
  private readonly rejected = new Set<(param: { i: number; value: number }) => void>();
  private readonly errors = new Set<(message: string) => void>();

  tasks: TaskGoal[] = [];
  /** Requested joint targets, distinct from the projected equilibrium in Q_TARGET. */
  targets: number[] = [];
  private readonly checkpoints = new Set<(frame: CheckpointFrame) => void>();
  private readonly taskListeners = new Set<() => void>();
  private readonly restores = new Map<number, (state: RuntimeState | null) => void>();
  private readonly profiles = new Map<number, (report: TickProfile | null) => void>();

  onCheckpoint(fn: (frame: CheckpointFrame) => void): () => void {
    this.checkpoints.add(fn);
    return () => this.checkpoints.delete(fn);
  }
  onTasks(fn: () => void): () => void {
    this.taskListeners.add(fn);
    return () => this.taskListeners.delete(fn);
  }
  setTasks(tasks: TaskGoal[]): void {
    this.tasks = tasks.map((t) => ({ ...t, y: t.kind === 'orientation' ? 0 : t.y }));
    this.send({ type: 'tasks', tasks: this.tasks });
  }
  restore(checkpoint: number): Promise<RuntimeState | null> {
    if (this.failure || !Number.isInteger(checkpoint) || checkpoint < 1 || checkpoint > 0xffffffff)
      return Promise.resolve(null);
    const id = this.id();
    return new Promise((resolve) => {
      this.restores.set(id, resolve);
      this.send({ type: 'restore', id, checkpoint });
    });
  }
  profile(ticks = 250): Promise<TickProfile | null> {
    if (
      this.failure ||
      this.profiles.size ||
      !Number.isInteger(ticks) ||
      ticks < 1 ||
      ticks > MAX_PROFILE_TICKS
    )
      return Promise.resolve(null);
    const id = this.id();
    return new Promise((resolve) => {
      this.profiles.set(id, resolve);
      this.send({ type: 'profile', id, ticks });
    });
  }

  constructor(
    private readonly worker: Worker,
    readonly layout: Layout,
    sab: SharedArrayBuffer | null,
  ) {
    this.snap = new Snapshot(layout);
    this.shared = sab ? new SharedReader(sab, layout.len) : null;
    this.stats.shared = !!sab;
    worker.addEventListener('message', (e: MessageEvent<FromWorker>) => {
      if (this.failure) return;
      const msg = e.data;
      try {
        if ('sequence' in msg) this.validate(msg);
        if (msg.type === 'checkpoint') {
          if (msg.generation === this.generation && msg.data.length === this.layout.len)
            for (const fn of this.checkpoints) fn(msg);
        } else if (msg.type === 'tasksChanged') {
          if (msg.generation === this.generation) {
            const changed = JSON.stringify(this.tasks) !== JSON.stringify(msg.tasks);
            this.tasks = msg.tasks;
            if (changed) for (const fn of this.taskListeners) fn();
          }
        } else if (msg.type === 'profiled') {
          this.profiles.get(msg.id)?.(msg.report);
          this.profiles.delete(msg.id);
        } else if (msg.type === 'restoreFailed') {
          this.restores.get(msg.id)?.(null);
          this.restores.delete(msg.id);
          this.report(msg.error);
        } else if (msg.type === 'restored') {
          const done = this.restores.get(msg.id);
          this.restores.delete(msg.id);
          if (msg.generation === this.generation) {
            // A newer periodic observation may arrive before this acknowledgment is handled.
            // It supersedes the sample, but the restore and its configuration still succeeded.
            this.accept(msg);
            this.tasks = msg.runtime.tasks;
            this.targets = msg.runtime.targets;
            done?.(msg.runtime);
          } else done?.(null);
        } else if (msg.type === 'prepared') {
          const request = this.pending.get(msg.id);
          if (request?.current()) this.send({ type: 'commitBuild', id: msg.id });
          else {
            this.pending.delete(msg.id);
            this.send({ type: 'discardBuild', id: msg.id });
            request?.done({ ok: false, error: 'Build superseded by a newer revision' });
          }
        } else if (msg.type === 'snapshot') {
          if (msg.generation === this.generation) this.latest = msg;
        } else if (msg.type === 'observed') {
          const done = this.observations.get(msg.id);
          this.observations.delete(msg.id);
          done?.(msg);
        } else if (msg.type === 'built') {
          const done = this.pending.get(msg.id)?.done;
          // Parse before removing the request, so a malformed reply fails it too.
          const result: BuildResult = msg.ok
            ? {
                ...msg,
                derivation: JSON.parse(msg.derivation) as Derivation,
                controllers: parseControllers(msg.controllers),
                ms: msg.buildMs,
                reused: msg.reused,
                targets: msg.targets,
              }
            : { ok: false, error: msg.error };
          this.pending.delete(msg.id);
          done?.(result);
        } else if (msg.type === 'error') this.fail(msg.message);
        else if (msg.type === 'invalid') {
          if (msg.param) for (const fn of this.rejected) fn(msg.param);
          this.report(msg.message);
        }
      } catch (error) {
        this.fail(String(error));
      }
    });
    worker.addEventListener('error', (e) => this.fail(e.message || 'Physics worker failed'));
    worker.addEventListener('messageerror', () => this.fail('Could not read physics worker response'));
  }

  static start(): Promise<SimClient> {
    const worker = new Worker(new URL('./worker.ts', import.meta.url), { type: 'module' });
    return new Promise((resolve, reject) => {
      const failed = (message: string) => {
        worker.terminate();
        reject(new Error(message));
      };
      const onError = (e: ErrorEvent) => failed(e.message);
      const onMessage = (e: MessageEvent<FromWorker>) => {
        if (e.data.type === 'ready') {
          try {
            if (e.data.version !== TRANSPORT_VERSION)
              throw new Error('Incompatible physics worker transport');
            const layout = parseLayout(e.data.layout);
            worker.removeEventListener('message', onMessage);
            worker.removeEventListener('error', onError);
            resolve(new SimClient(worker, layout, e.data.shared));
          } catch (error) {
            failed(String(error));
          }
        } else if (e.data.type === 'error') failed(e.data.message);
      };
      worker.addEventListener('message', onMessage);
      worker.addEventListener('error', onError);
    });
  }

  onParamRejected(fn: (param: { i: number; value: number }) => void): () => void {
    this.rejected.add(fn);
    return () => this.rejected.delete(fn);
  }

  onError(fn: (message: string) => void): () => void {
    this.errors.add(fn);
    return () => this.errors.delete(fn);
  }

  private report(message: string): void {
    for (const fn of this.errors) fn(message);
  }

  private fail(message: string): void {
    if (this.failure) return;
    this.failure = message;
    this.worker.terminate();
    for (const { done } of this.pending.values()) done({ ok: false, error: message });
    this.pending.clear();
    for (const done of this.observations.values()) done(null);
    this.observations.clear();
    for (const done of this.restores.values()) done(null);
    this.restores.clear();
    for (const done of this.profiles.values()) done(null);
    this.profiles.clear();
    this.report(message);
  }

  send(msg: ToWorker): boolean {
    if (this.failure) return false;
    try {
      this.worker.postMessage({ ...msg, generation: this.generation });
      if (
        msg.type === 'target' &&
        Number.isInteger(msg.i) &&
        msg.i >= 0 &&
        msg.i < this.targets.length &&
        Number.isFinite(msg.v)
      )
        this.targets[msg.i] = msg.v;
      return true;
    } catch (error) {
      this.fail(String(error));
      return false;
    }
  }

  flag(f: Flag, v: number): void {
    this.send({ type: 'flag', id: this.layout.flag[f], v });
  }

  build(desc: number[], current: () => boolean = () => true): Promise<BuildResult> {
    if (this.failure) return Promise.resolve({ ok: false, error: this.failure });
    const id = this.id();
    return new Promise((resolve) => {
      this.pending.set(id, { done: resolve, current });
      this.send({ type: 'build', id, desc });
    });
  }

  install(result: Built): void {
    if (this.failure) return;
    this.validate(result);
    for (const done of this.restores.values()) done(null);
    this.restores.clear();
    this.generation = result.generation;
    this.tasks = [];
    this.targets = [...result.targets];
    this.sequence = null;
    this.latest = null;
    this.shared?.reset();
    this.accept(result);
  }

  /** Acknowledge previous commands and obtain a fresh observation, including while paused. */
  refresh(signal?: AbortSignal): Promise<boolean> {
    if (signal?.aborted || this.failure) return Promise.resolve(false);
    const generation = this.generation;
    const id = this.id();
    return new Promise((resolve) => {
      const abort = () => {
        this.observations.delete(id);
        resolve(false);
      };
      signal?.addEventListener('abort', abort, { once: true });
      this.observations.set(id, (o) => {
        signal?.removeEventListener('abort', abort);
        if (!o || o.generation !== generation || generation !== this.generation) resolve(false);
        else {
          this.accept(o);
          resolve(true);
        }
      });
      this.send({ type: 'observe', id });
    });
  }

  pull(): boolean {
    if (this.failure) return false;
    if (this.shared) {
      if (!this.shared.read(this.generation)) return false;
      return this.accept(this.shared);
    }
    const o = this.latest;
    this.latest = null;
    if (!o || o.generation !== this.generation) return false;
    return this.accept(o);
  }

  private validate(o: Observation): void {
    if (
      o.data.length !== this.layout.len ||
      o.stats.length !== STATS ||
      !Number.isInteger(o.generation) ||
      o.generation < 0 ||
      o.generation > 0x7fffffff ||
      !Number.isInteger(o.sequence) ||
      o.sequence < 0 ||
      o.sequence > 0xffffffff ||
      (o.sequence & 1) !== 0
    )
      throw new Error('Invalid observation layout or generation');
  }

  private accept(o: Pick<Observation, 'sequence' | 'data' | 'stats'>): boolean {
    if (this.sequence !== null && !newerSequence(o.sequence, this.sequence)) return false;
    this.sequence = o.sequence;
    this.snap.data.set(o.data);
    this.readStats(o.stats);
    return true;
  }

  private readStats(s: Float64Array): void {
    this.stats.stepUs = s[0] ?? 0;
    this.stats.stepsPerSecond = s[1] ?? 0;
    this.stats.realtime = s[2] ?? 0;
    this.stats.running = (s[3] ?? 0) !== 0;
  }
}
