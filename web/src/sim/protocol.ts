/** Incompatible worker messages or shared-memory headers increment this independently of the core ABI. */
export const TRANSPORT_VERSION = 1;

/** Messages between the UI thread and the physics worker. */
export type ToWorker = (
  | { type: 'build'; id: number; desc: number[] }
  | { type: 'observe'; id: number }
  | { type: 'commitBuild'; id: number }
  | { type: 'discardBuild'; id: number }
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
  | { type: 'advance'; steps: number }
  | { type: 'run'; running: boolean }
  | { type: 'speed'; speed: number }
) & { generation?: number };

export interface Observation {
  sequence: number;
  generation: number;
  data: Float64Array<ArrayBuffer>;
  stats: Float64Array<ArrayBuffer>;
}

export type FromWorker =
  | { type: 'ready'; version: number; layout: string; shared: SharedArrayBuffer | null }
  | { type: 'prepared'; id: number }
  | ({
      type: 'built';
      id: number;
      ok: true;
      derivation: string;
      controllers: string;
      buildMs: number;
    } & Observation)
  | { type: 'built'; id: number; ok: false; error: string }
  | ({ type: 'snapshot' } & Observation)
  | ({ type: 'observed'; id: number } & Observation)
  | { type: 'invalid'; message: string; param?: { i: number; value: number } }
  | { type: 'error'; message: string };

/** [seq: i32, installed generation: i32][stats: f64 × STATS][snapshot: f64 × len].
 * Generation, stats and data are all covered by the sequence check. */
export const STATS = 4; // [µs per step, steps per second, requested speed, running]
export const STATS_OFFSET = 8;
export const DATA_OFFSET = STATS_OFFSET + STATS * 8;

/** Observations use the seqlock's unsigned 32-bit sequence, including explicit acknowledgments. */
export const newerSequence = (next: number, previous: number) => {
  const delta = (next - previous) >>> 0;
  return delta !== 0 && delta < 0x80000000;
};
