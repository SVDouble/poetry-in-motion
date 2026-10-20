/**
 * Typed view of the snapshot layout. The authoritative layout lives in Rust (`core/src/layout.rs`) and
 * arrives as JSON; `parseLayout` validates that every field this UI relies on exists.
 */

export const FIELDS = [
  'TIME',
  'STEPS',
  'NQ',
  'NU',
  'MODE',
  'PHASE',
  'Q',
  'QD',
  'QDD',
  'U',
  'TAU',
  'TAU_EXT',
  'M',
  'C',
  'N',
  'DV',
  'MQDD',
  'CQD',
  'MDOT',
  'ENERGY',
  'POWER',
  'DISSIPATION',
  'SKEW',
  'DET_M',
  'EIG_M',
  'FRAMES',
  'COMS',
  'TOOL_P',
  'JP',
  'JS',
  'LIN_A',
  'LIN_B',
  'LQR_K',
  'U_FF',
  'LQR_OK',
  'EQ_RESIDUAL',
  'E_U',
  'E_REF',
  'TARGET',
  'IK',
  'REACHABLE',
  'Q_TARGET',
  'Q_REF',
  'QD_REF',
  'KP',
  'KD',
  'UMAX',
  'CT_WN',
  'NC',
  'CONSTRAINT',
  'LAMBDA',
  'AT_LAMBDA',
] as const;

export const FLAGS = [
  'Coriolis',
  'Umax',
  'TargetX',
  'TargetY',
  'Ik',
  'RefWn',
  'CtWn',
  'PdWn',
  'LqrQx',
  'LqrQv',
  'LqrR',
  'SwingGain',
] as const;

export type Field = (typeof FIELDS)[number];
export type Flag = (typeof FLAGS)[number];

export interface Layout {
  len: number;
  maxn: number;
  off: Record<Field, number>;
  flag: Record<Flag, number>;
}

interface RawLayout {
  len: number;
  maxn: number;
  fields: Record<string, [number, number]>;
  flags: Record<string, number>;
}

export function parseLayout(json: string): Layout {
  const raw = JSON.parse(json) as RawLayout;
  const off = {} as Record<Field, number>;
  for (const f of FIELDS) {
    const entry = raw.fields[f];
    if (!entry) throw new Error(`snapshot layout is missing field ${f} — rebuild the wasm core`);
    off[f] = entry[0];
  }
  const flag = {} as Record<Flag, number>;
  for (const f of FLAGS) {
    const id = raw.flags[f];
    if (id === undefined) throw new Error(`snapshot layout is missing flag ${f} — rebuild the wasm core`);
    flag[f] = id;
  }
  return { len: raw.len, maxn: raw.maxn, off, flag };
}

/** Read helpers over the most recent snapshot. */
export class Snapshot {
  data: Float64Array;

  constructor(readonly layout: Layout) {
    this.data = new Float64Array(layout.len);
  }

  get n(): number {
    return this.v('NQ');
  }

  get nu(): number {
    return this.v('NU');
  }

  get t(): number {
    return this.v('TIME');
  }

  /** Scalar or vector element `field[i]`. */
  v(f: Field, i = 0): number {
    return this.data[this.layout.off[f] + i] ?? Number.NaN;
  }

  /** Matrix element (row-major, given column count; defaults to n). */
  m(f: Field, i: number, j: number, cols = this.n): number {
    return this.data[this.layout.off[f] + i * cols + j] ?? Number.NaN;
  }

  /** Pose (x, y, φ) of frame k: 0 = s, 1 … n = bodies, n + 1 = tool. */
  frame(k: number): [number, number, number] {
    const o = this.layout.off.FRAMES + 3 * k;
    return [this.data[o] ?? 0, this.data[o + 1] ?? 0, this.data[o + 2] ?? 0];
  }
}
