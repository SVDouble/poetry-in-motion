/**
 * Which controller runs in which simulation slot, and every controller's parameters. The single source of
 * truth on the UI side: widgets bound to a parameter stay in sync wherever they are shown, and views that
 * depend on the slot assignment re-render on `onSlots`.
 */
import type { SimClient } from '../sim/client';
import type { ControllerSpec, Controllers } from '../sim/controllers';

export { SLOT_COLORS } from '../scene/theme';

export const OFF = -1;

export class ControlStore {
  reg: Controllers = { specs: [], applicable: [], params: [] };
  /** Controller kind per slot (OFF when a shadow slot is free). */
  readonly slots: number[];
  private readonly setters = new Map<string, Set<(v: number) => void>>();
  private readonly listeners = new Set<() => void>();
  /** The user picked the primary (keep it across rebuilds while it applies). */
  private chosen = false;

  constructor(
    private readonly client: SimClient,
    slots: number,
  ) {
    this.slots = Array.from({ length: slots }, () => OFF);
    this.slots[0] = 0;
  }

  get primary(): number {
    return this.slots[0] ?? 0;
  }

  spec(kind: number): ControllerSpec | undefined {
    return this.reg.specs[kind];
  }

  /** Why `kind` cannot run on this mechanism, or null. */
  reason(kind: number): string | null {
    return kind >= 0 && kind < this.reg.applicable.length ? this.reg.applicable[kind]! : 'unknown controller';
  }

  usable(): ControllerSpec[] {
    return this.reg.specs.filter((s) => this.reason(s.id) === null);
  }

  /** Slot running `kind`, or −1. */
  slotOf(kind: number): number {
    return this.slots.indexOf(kind);
  }

  /** A new mechanism: keep the controllers that still apply; the primary falls back to `fallback` unless
   * the user chose it. */
  load(reg: Controllers, fallback: number): void {
    this.reg = reg;
    const keep = (k: number, i: number) => k !== OFF && this.reason(k) === null && (i > 0 || this.chosen);
    this.slots.forEach((k, i) => {
      this.slots[i] = keep(k, i) ? k : i === 0 ? fallback : OFF;
      this.client.send({ type: 'controller', slot: i, kind: this.slots[i]! });
    });
    this.notify();
  }

  /** Adopt a restored core state without sending assignments that would reset its controllers. */
  restore(reg: Controllers, slots: number[]): void {
    this.reg = reg;
    this.slots.splice(0, this.slots.length, ...slots);
    this.chosen = true;
    this.notify();
  }

  setPrimary(kind: number): void {
    if (this.reason(kind) !== null || kind === this.primary) return;
    this.chosen = true;
    const shadow = this.slotOf(kind);
    if (shadow > 0) this.assign(shadow, OFF);
    this.assign(0, kind);
    this.notify();
  }

  /** Run `kind` as a shadow, or stop its shadow. */
  toggleShadow(kind: number): void {
    if (this.reason(kind) !== null || kind === this.primary) return;
    const at = this.slotOf(kind);
    if (at > 0) this.assign(at, OFF);
    else {
      const free = this.slots.indexOf(OFF);
      if (free < 0) return;
      this.assign(free, kind);
    }
    this.notify();
  }

  byKey(key: string): number {
    return this.reg.specs.find((s) => s.key === key)?.id ?? OFF;
  }

  clearShadows(): void {
    this.slots.forEach((k, i) => {
      if (i > 0 && k !== OFF) this.assign(i, OFF);
    });
    this.notify();
  }

  /** Back to the defaults of every parameter of `kind`. */
  resetParams(kind: number): void {
    this.spec(kind)?.params.forEach((p, i) => {
      this.setParam(kind, i, p.default);
    });
  }

  /** Restart every shadow from the primary's current state. */
  sync(): void {
    this.client.send({ type: 'sync' });
  }

  param(kind: number, i: number): number {
    return this.reg.params[kind]?.[i] ?? Number.NaN;
  }

  setParam(kind: number, i: number, v: number, from?: (v: number) => void): void {
    const p = this.reg.params[kind];
    if (!p) return;
    p[i] = v;
    this.client.send({ type: 'ctrlParam', kind, i, v });
    for (const set of this.setters.get(`${kind}:${i}`) ?? []) if (set !== from) set(v);
  }

  /** Keep a widget showing parameter i of `kind` in sync. Returns the unbind function. */
  bind(kind: number, i: number, set: (v: number) => void): () => void {
    const key = `${kind}:${i}`;
    const group = this.setters.get(key) ?? new Set();
    this.setters.set(key, group);
    group.add(set);
    return () => group.delete(set);
  }

  onSlots(fn: () => void): () => void {
    this.listeners.add(fn);
    return () => this.listeners.delete(fn);
  }

  private assign(slot: number, kind: number): void {
    this.slots[slot] = kind;
    this.client.send({ type: 'controller', slot, kind });
  }

  private notify(): void {
    for (const fn of this.listeners) fn();
  }
}
