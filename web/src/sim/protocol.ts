/** Messages between the UI thread and the physics worker. */

export type ToWorker =
  | { type: 'init'; system: number; params: number[]; mode: number; flags: [number, number][] }
  | { type: 'param'; i: number; v: number }
  | { type: 'mode'; mode: number }
  | { type: 'input'; i: number; v: number }
  | { type: 'flag'; id: number; v: number }
  | { type: 'kick'; i: number; v: number }
  | { type: 'state'; i: number; q: number; qd: number }
  | { type: 'reset' }
  | { type: 'run'; running: boolean }
  | { type: 'speed'; speed: number };

export type FromWorker =
  | { type: 'ready'; layout: string; shared: SharedArrayBuffer | null }
  | { type: 'snapshot'; data: Float64Array<ArrayBuffer>; stats: Float64Array<ArrayBuffer> }
  | { type: 'error'; message: string };

/**
 * Shared buffer layout: [seq: i32, pad: i32][stats: f64 × STATS][snapshot: f64 × len]
 * `seq` is a seqlock — odd while the worker is writing.
 */
export const STATS = 4; // [µs per step, steps per second, real-time factor, running]
export const STATS_OFFSET = 8;
export const DATA_OFFSET = STATS_OFFSET + STATS * 8;
