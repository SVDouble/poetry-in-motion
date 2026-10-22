/**
 * Typed view of the snapshot layout. The authoritative layout lives in Rust (`core/src/layout.rs`) and
 * arrives as JSON; `parseLayout` validates that every field this UI relies on exists.
 */

import {
  ABI_VERSION,
  DEFAULT_FLAGS,
  FIELDS,
  FIELDS_LENGTHS,
  FLAGS,
  LIMITS,
  SLOT_FIELDS,
  SLOT_FIELDS_LENGTHS,
} from '../generated/contracts';

export { FIELDS, FLAGS, SLOT_FIELDS } from '../generated/contracts';

export type Field = (typeof FIELDS)[number];
export type SlotField = (typeof SLOT_FIELDS)[number];
export type Flag = (typeof FLAGS)[number];

/** The limits of a mechanism description, as the core defines them. */
export interface Limits {
  version: number;
  header: number;
  perBody: number;
  perClosure: number;
  perElement: number;
  paramsPerBody: number;
  maxBodies: number;
  maxDof: number;
  maxClosures: number;
  maxElements: number;
}

export interface Layout {
  len: number;
  maxn: number;
  /** The fixed step of the simulation (s). */
  dt: number;
  /** What a fresh simulation starts with, for the flags that mirror a setting. */
  defaults: Partial<Record<Flag, number>>;
  limits: Limits;
  /** Simulation slots: the primary (0) and the shadows. */
  slots: number;
  slotLen: number;
  off: Record<Field, number>;
  slot: Record<SlotField, number>;
  flag: Record<Flag, number>;
}

interface RawLayout {
  abiVersion: number;
  len: number;
  dt: number;
  defaults: Partial<Record<Flag, number>>;
  limits: Limits;
  maxn: number;
  slots: number;
  slotLen: number;
  fields: Record<string, [number, number]>;
  slot: Record<string, [number, number]>;
  flags: Record<string, number>;
}

function invalid(what: string): never {
  throw new Error(`Invalid core contract: ${what} — rebuild the wasm core`);
}

export function parseLayout(json: string): Layout {
  const raw = JSON.parse(json) as RawLayout;
  if (!raw || raw.abiVersion !== ABI_VERSION) invalid('incompatible ABI version');
  for (const key of ['len', 'maxn', 'slots', 'slotLen'] as const) {
    if (!Number.isSafeInteger(raw[key]) || raw[key] <= 0) invalid(key);
  }
  if (!Number.isFinite(raw.dt) || raw.dt <= 0) invalid('timestep');
  if (!raw.defaults || DEFAULT_FLAGS.some((key) => !Number.isFinite(raw.defaults[key])))
    invalid('required defaults');
  if (Object.values(raw.defaults).some((v) => !Number.isFinite(v))) invalid('defaults');
  for (const key of Object.keys(LIMITS) as (keyof Limits)[]) {
    if (raw.limits?.[key] !== LIMITS[key]) invalid(`mechanism limit ${key}`);
  }
  if (raw.maxn !== raw.limits.maxDof) invalid('coordinate capacity');
  const fields = <K extends string>(
    keys: readonly K[],
    widths: Record<K, number>,
    entries: Record<string, [number, number]>,
    len: number,
  ): Record<K, number> => {
    const result = {} as Record<K, number>;
    const occupied = new Set<number>();
    for (const key of keys) {
      const entry = entries?.[key];
      if (!Array.isArray(entry) || entry.length !== 2) invalid(`field ${key}`);
      const [offset, width] = entry;
      if (!Number.isSafeInteger(offset) || offset < 0 || width !== widths[key] || offset + width > len)
        invalid(`bounds or length of ${key}`);
      for (let i = offset; i < offset + width; i++) {
        if (occupied.has(i)) invalid(`overlapping field ${key}`);
        occupied.add(i);
      }
      result[key] = offset;
    }
    if (occupied.size !== len) invalid('unmapped snapshot values');
    return result;
  };
  const off = fields(FIELDS, FIELDS_LENGTHS, raw.fields, raw.len);
  const slot = fields(SLOT_FIELDS, SLOT_FIELDS_LENGTHS, raw.slot, raw.slotLen);
  if (FIELDS_LENGTHS.SLOT !== raw.slots * raw.slotLen) invalid('slot capacity');
  const flag = {} as Record<Flag, number>;
  for (const [id, key] of FLAGS.entries()) {
    if (raw.flags?.[key] !== id) invalid(`flag ${key}`);
    flag[key] = id;
  }
  return {
    len: raw.len,
    maxn: raw.maxn,
    dt: raw.dt,
    defaults: raw.defaults,
    limits: raw.limits,
    slots: raw.slots,
    slotLen: raw.slotLen,
    off,
    slot,
    flag,
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
