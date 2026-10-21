/**
 * Typed view of the snapshot layout. The authoritative layout lives in Rust (`core/src/layout.rs`) and
 * arrives as JSON; `parseLayout` validates that every field this UI relies on exists.
 */

export const FIELDS = [
  'TIME',
  'STEPS',
  'NQ',
  'NU',
  'KIND',
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
  'WORK',
  'E0',
  'BUDGET',
  'EIG_M',
  'FRAMES',
  'COMS',
  'TOOL_P',
  'JP',
  'JS',
  'LIN_A',
  'LIN_B',
  'U_FF',
  'EQ_RESIDUAL',
  'DOF',
  'CT_OK',
  'TARGET',
  'USE_TOOL',
  'REACHABLE',
  'Q_TARGET',
  'Q_REF',
  'QD_REF',
  'X_REF',
  'FEXT',
  'MANIPULABILITY',
  'UMAX',
  'GOALS',
  'NC',
  'NCON',
  'CONSTRAINT',
  'LAMBDA',
  'AT_LAMBDA',
  'VCOM',
  'ACOM',
  'OMEGA',
  'ALPHA',
  'ELEMENT_F',
  'JCOM',
  'SLOT',
] as const;

/** Fields of one slot block (primary and shadows). */
export const SLOT_FIELDS = [
  'KIND',
  'PHASE',
  'ERR_Q',
  'ERR_X',
  'EFFORT',
  'ENERGY',
  'Q',
  'U',
  'FRAMES',
  'TOOL',
  'TELEMETRY',
] as const;

export const FLAGS = [
  'Coriolis',
  'Umax',
  'TargetX',
  'TargetY',
  'UseTool',
  'RefWn',
  'PushX',
  'PushY',
  'PushOn',
  'PushK',
  'Posture',
  'Manipulability',
  'Orient',
  'OrientTarget',
  'Elbow',
  'ElbowX',
  'ElbowY',
  'ElbowBody',
  'NullDamping',
  'DlsEps',
  'DlsMax',
  'DlsLowerEps',
  'DlsLowerMax',
  'FadeLo',
  'FadeHi',
  'ReachMargin',
  'IkStep',
  'IkDamping',
  'PushMax',
  'StopK',
  'StopC',
  'StopDepth',
  'Baumgarte',
  'MotorSpeed',
  'Lead',
] as const;

export type Field = (typeof FIELDS)[number];
export type SlotField = (typeof SLOT_FIELDS)[number];
export type Flag = (typeof FLAGS)[number];

export interface Layout {
  len: number;
  maxn: number;
  /** Simulation slots: the primary (0) and the shadows. */
  slots: number;
  slotLen: number;
  off: Record<Field, number>;
  slot: Record<SlotField, number>;
  flag: Record<Flag, number>;
}

interface RawLayout {
  len: number;
  maxn: number;
  slots: number;
  slotLen: number;
  fields: Record<string, [number, number]>;
  slot: Record<string, [number, number]>;
  flags: Record<string, number>;
}

function pick<K extends string>(
  keys: readonly K[],
  raw: Record<string, unknown>,
  what: string,
  at: (v: never) => number,
) {
  const out = {} as Record<K, number>;
  for (const k of keys) {
    const v = raw[k];
    if (v === undefined) throw new Error(`snapshot layout is missing ${what} ${k} — rebuild the wasm core`);
    out[k] = at(v as never);
  }
  return out;
}

export function parseLayout(json: string): Layout {
  const raw = JSON.parse(json) as RawLayout;
  const first = (e: [number, number]) => e[0];
  return {
    len: raw.len,
    maxn: raw.maxn,
    slots: raw.slots,
    slotLen: raw.slotLen,
    off: pick(FIELDS, raw.fields, 'field', first),
    slot: pick(SLOT_FIELDS, raw.slot, 'slot field', first),
    flag: pick(FLAGS, raw.flags, 'flag', (v: number) => v),
  };
}

/** One slot's block of the snapshot: its controller, errors, pose and telemetry. */
export class SlotView {
  constructor(
    private readonly snap: Snapshot,
    readonly index: number,
  ) {}

  private get base(): number {
    const l = this.snap.layout;
    return l.off.SLOT + this.index * l.slotLen;
  }

  /** Controller kind, or −1 when the slot is off. */
  get kind(): number {
    return this.v('KIND');
  }

  get on(): boolean {
    return this.kind >= 0;
  }

  v(f: SlotField, i = 0): number {
    return this.snap.data[this.base + this.snap.layout.slot[f] + i] ?? Number.NaN;
  }

  /** Telemetry value at offset `at` (see the controller's spec). */
  tele(at: number): number {
    return this.v('TELEMETRY', at);
  }

  frame(k: number): [number, number, number] {
    const o = this.base + this.snap.layout.slot.FRAMES + 3 * k;
    const d = this.snap.data;
    return [d[o] ?? 0, d[o + 1] ?? 0, d[o + 2] ?? 0];
  }
}

/** Read helpers over the most recent snapshot. */
export class Snapshot {
  data: Float64Array;

  readonly slots: SlotView[];

  constructor(readonly layout: Layout) {
    this.data = new Float64Array(layout.len);
    this.slots = Array.from({ length: layout.slots }, (_, i) => new SlotView(this, i));
  }

  /** The slot running controller `kind`, if any (the primary first). */
  slotOf(kind: number): SlotView | undefined {
    return this.slots.find((s) => s.kind === kind);
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
