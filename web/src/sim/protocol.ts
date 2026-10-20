/** Messages between the UI thread and the physics worker. */

export type ToWorker =
  | { type: 'build'; id: number; desc: number[] }
  | { type: 'param'; i: number; v: number }
  | { type: 'controller'; slot: number; kind: number }
  | { type: 'ctrlParam'; kind: number; i: number; v: number }
  | { type: 'sync' }
  | { type: 'input'; i: number; v: number }
  | { type: 'flag'; id: number; v: number }
  | { type: 'target'; i: number; v: number }
  | { type: 'kick'; i: number; v: number }
  | { type: 'state'; i: number; q: number; qd: number }
  | { type: 'reset' }
  | { type: 'run'; running: boolean }
  | { type: 'speed'; speed: number };

export type FromWorker =
  | { type: 'ready'; layout: string; shared: SharedArrayBuffer | null }
  | { type: 'built'; id: number; ok: true; derivation: string; controllers: string; buildMs: number }
  | { type: 'built'; id: number; ok: false; error: string }
  | { type: 'snapshot'; data: Float64Array<ArrayBuffer>; stats: Float64Array<ArrayBuffer> }
  | { type: 'error'; message: string };

/**
 * Shared buffer layout: [seq: i32, pad: i32][stats: f64 × STATS][snapshot: f64 × len]
 * `seq` is a seqlock — odd while the worker is writing.
 */
export const STATS = 4; // [µs per step, steps per second, real-time factor, running]
export const STATS_OFFSET = 8;
export const DATA_OFFSET = STATS_OFFSET + STATS * 8;
