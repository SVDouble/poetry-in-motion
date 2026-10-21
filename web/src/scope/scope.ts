/**
 * The scope: rolling plots of any signals (see `signals.ts`). One recorder samples every watched signal
 * once; plots are views onto it, so adding or removing a signal keeps the history of the others. Each plot
 * shows its signals as chips with live values; presets switch between curated layouts.
 */
import uPlot from 'uplot';
import 'uplot/dist/uPlot.min.css';
import { fix } from '../eq/tex';

/** Smallest vertical span of a plot: round-off noise (1e-15 … 1e-6) must not be magnified into curves. */
const MIN_SPAN = 1e-3;

/** y-range of a plot: the data with 5 % margin, but never narrower than MIN_SPAN (relative for large values). */
function yRange(_u: uPlot, lo: number | null, hi: number | null): uPlot.Range.MinMax {
  if (lo === null || hi === null || !Number.isFinite(lo) || !Number.isFinite(hi)) return [-1, 1];
  const mid = (lo + hi) / 2;
  const half = Math.max((hi - lo) * 0.55, (MIN_SPAN / 2) * Math.max(1, Math.abs(mid)));
  return [mid - half, mid + half];
}

import { css, palette } from '../scene/theme';
import { closePopover, popover } from '../ui/popover';
import { button, el } from '../ui/widgets';
import type { Signal, Signals } from './signals';

export interface PlotDef {
  ids: string[];
}

export interface Preset {
  key: string;
  label: string;
  plots: PlotDef[];
  hint?: string;
}

const WINDOW_S = 10;
const CAPACITY = 600;
const MAX_PLOTS = 4;
const COLORS = [
  palette.accent,
  palette.input,
  palette.coriolis,
  palette.gravity,
  palette.inertial,
  palette.ink,
  0x6b7a1f,
  0x9a2f7a,
];

/** Time series of every watched signal, in one shared time base. */
class Recorder {
  readonly t = new Float64Array(CAPACITY);
  n = 0;
  readonly cols = new Map<string, { sig: Signal; data: Float64Array }>();

  watch(sigs: Signal[]): void {
    const keep = new Set(sigs.map((s) => s.id));
    for (const id of this.cols.keys()) if (!keep.has(id)) this.cols.delete(id);
    for (const sig of sigs) {
      const col = this.cols.get(sig.id);
      if (col) col.sig = sig;
      else this.cols.set(sig.id, { sig, data: new Float64Array(CAPACITY).fill(Number.NaN) });
    }
  }

  sample(time: number): boolean {
    const last = this.n ? this.t[this.n - 1]! : Number.NaN;
    if (time === last) return false;
    if (time < last) this.n = 0; // time went backwards (reset)
    if (this.n === CAPACITY) {
      this.t.copyWithin(0, 1);
      for (const c of this.cols.values()) c.data.copyWithin(0, 1);
      this.n--;
    }
    this.t[this.n] = time;
    for (const c of this.cols.values()) c.data[this.n] = c.sig.get();
    this.n++;
    return true;
  }

  series(id: string): Float64Array {
    return this.cols.get(id)?.data.subarray(0, this.n) ?? new Float64Array(this.n).fill(Number.NaN);
  }
}

class PlotView {
  readonly el: HTMLElement;
  private readonly plot: uPlot;
  private readonly values: HTMLElement[] = [];

  constructor(
    readonly sigs: Signal[],
    actions: { remove: (id: string) => void; add: (anchor: HTMLElement) => void; drop: () => void },
  ) {
    this.el = el('div', 'scope-plot');
    const chips = el('div', 'scope-chips');
    sigs.forEach((s, k) => {
      const color = css(s.color ?? COLORS[k % COLORS.length]!);
      const value = el('span', 'v');
      this.values.push(value);
      const x = button('×', () => actions.remove(s.id), 'x', 'remove');
      const chip = el('span', 'scope-chip', el('i', '', ''), el('span', 'l', s.label), value, x);
      (chip.firstChild as HTMLElement).style.background = s.dash
        ? `repeating-linear-gradient(90deg, ${color} 0 3px, transparent 3px 5px)`
        : color;
      chip.title = s.unit ? `${s.label} [${s.unit}]` : s.label;
      chips.append(chip);
    });
    const add = button('+', () => add.isConnected && actions.add(add), 'scope-add', 'add a signal');
    const drop = button('✕', actions.drop, 'scope-drop', 'remove this plot');
    chips.append(add, drop);
    const canvas = el('div', 'scope-canvas');
    this.el.append(chips, canvas);
    const axis: uPlot.Axis = {
      stroke: '#5b606b',
      grid: { stroke: '#e4e0d5', width: 1 },
      ticks: { show: false },
      font: '11px KaTeX_Main, "Latin Modern Roman", serif',
      size: 26,
    };
    this.plot = new uPlot(
      {
        width: 300,
        height: 120,
        pxAlign: 0,
        cursor: { show: true, drag: { x: false, y: false } },
        legend: { show: false },
        scales: { x: { time: false }, y: { range: yRange } },
        axes: [
          { ...axis, values: (_u, v) => v.map((t) => `${t.toFixed(0)}s`) },
          { ...axis, size: 46, values: (_u, v) => v.map((x) => fix(x)) },
        ],
        series: [
          {},
          ...sigs.map((s, k) => ({
            label: s.label,
            stroke: css(s.color ?? COLORS[k % COLORS.length]!),
            width: 1.5,
            points: { show: false },
            ...(s.dash ? { dash: s.dash } : {}),
          })),
        ],
      },
      [[], ...sigs.map(() => [])],
      canvas,
    );
  }

  draw(rec: Recorder): void {
    // uPlot treats null as a gap but NaN poisons its auto-range: missing samples (a signal watched only
    // from now on, a slot that is not running) become null
    const gaps = (col: Float64Array) => Array.from(col, (v) => (Number.isNaN(v) ? null : v));
    const data = [
      rec.t.subarray(0, rec.n),
      ...this.sigs.map((s) => gaps(rec.series(s.id))),
    ] as unknown as uPlot.AlignedData;
    const t1 = rec.n ? rec.t[rec.n - 1]! : 0;
    this.plot.batch(() => {
      this.plot.setData(data); // rescales y to the data; x is fixed to the rolling window below
      this.plot.setScale('x', { min: Math.max(0, t1 - WINDOW_S), max: Math.max(WINDOW_S, t1) });
    });
    this.sigs.forEach((s, k) => {
      const text = fix(s.get());
      const v = this.values[k]!;
      if (v.textContent !== text) v.textContent = text;
    });
  }

  /** Height of the legend chips above the canvas. */
  get header(): number {
    return (this.el.firstElementChild as HTMLElement).offsetHeight;
  }

  resize(w: number, h: number): void {
    this.plot.setSize({ width: w, height: h });
  }

  destroy(): void {
    this.plot.destroy();
    this.el.remove();
  }
}

export class Scope {
  private readonly rec = new Recorder();
  private readonly bar: HTMLElement;
  private readonly grid: HTMLElement;
  private views: PlotView[] = [];
  private defs: PlotDef[] = [];
  private presets: Preset[] = [];
  private active = '';
  /** Label of a layout that is not a preset (an experiment, a controller's watch list, a user edit). */
  private label = '';
  private signals: Signals | null = null;
  private readonly ro: ResizeObserver;

  constructor(host: HTMLElement) {
    this.bar = el('div', 'scope-bar');
    this.grid = el('div', 'scope-grid');
    host.replaceChildren(this.bar, this.grid);
    this.ro = new ResizeObserver(() => this.layout());
    this.ro.observe(this.grid);
  }

  /** New mechanism or slot assignment: new presets; keeps the current layout if it still applies. */
  mount(signals: Signals, presets: Preset[], prefer?: string): void {
    this.signals = signals;
    this.presets = presets;
    const key = prefer ?? this.active;
    const preset = presets.find((p) => p.key === key);
    if (preset) this.show(preset.key, preset.plots);
    else if (this.label && this.defs.length) this.show(key, this.defs, this.label);
    else this.show(presets[0]!.key, presets[0]!.plots);
  }

  /** Show a preset by key, or explicit plots under a label (experiments). */
  show(key: string, plots: PlotDef[], label?: string): void {
    this.active = key;
    this.label = label ?? '';
    this.defs = plots.map((p) => ({ ids: [...p.ids] })).slice(0, MAX_PLOTS);
    this.renderBar();
    this.renderPlots();
  }

  sample(t: number): void {
    this.rec.sample(t);
  }

  draw(): void {
    for (const v of this.views) v.draw(this.rec);
  }

  clear(): void {
    for (const v of this.views) v.destroy();
    this.views = [];
    this.rec.n = 0;
  }

  private renderBar(): void {
    const tabs = el('div', 'segmented small scope-presets');
    const entries = [...this.presets.map((p) => [p.key, p.label, p.hint] as const)];
    if (this.label) entries.push([this.active, this.label, undefined]);
    for (const [key, text, hint] of entries) {
      const b = button(text, () => {
        const p = this.presets.find((x) => x.key === key);
        if (p) this.show(p.key, p.plots);
      });
      b.className = key === this.active ? 'on' : '';
      if (hint) b.title = hint;
      tabs.append(b);
    }
    const add = button('+ plot', () => this.edit(() => this.defs.push({ ids: [] })), 'btn small');
    add.disabled = this.defs.length >= MAX_PLOTS;
    this.bar.replaceChildren(el('span', 'scope-title', 'Plots'), tabs, el('span', 'spacer'), add);
  }

  /** Apply a user edit: the layout becomes "Custom". */
  private edit(change: () => void): void {
    change();
    this.active = 'custom';
    this.label = 'Custom';
    this.renderBar();
    this.renderPlots();
  }

  private renderPlots(): void {
    const signals = this.signals;
    for (const v of this.views) v.destroy();
    this.views = [];
    if (!signals) return;
    const resolved = this.defs.map((d) =>
      d.ids.map((id) => signals.resolve(id)).filter((s) => s !== undefined),
    );
    this.rec.watch(resolved.flat());
    resolved.forEach((sigs, k) => {
      const def = this.defs[k]!;
      const view = new PlotView(sigs, {
        remove: (id) => this.edit(() => def.ids.splice(def.ids.indexOf(id), 1)),
        add: (anchor) => this.pick(anchor, def),
        drop: () => this.edit(() => this.defs.splice(k, 1)),
      });
      this.grid.append(view.el);
      this.views.push(view);
    });
    this.layout();
    this.draw();
  }

  private pick(anchor: HTMLElement, def: PlotDef): void {
    const signals = this.signals;
    if (!signals) return;
    const host = el('div', 'picker');
    for (const g of signals.groups()) {
      const items = el('div', 'picker-items');
      for (const id of g.ids) {
        const s = signals.resolve(id)!;
        const on = def.ids.includes(id);
        const b = button(
          s.label,
          () => {
            closePopover();
            this.edit(() => (on ? def.ids.splice(def.ids.indexOf(id), 1) : def.ids.push(id)));
          },
          `picker-item${on ? ' on' : ''}`,
        );
        if (s.unit) b.title = s.unit;
        items.append(b);
      }
      host.append(el('div', 'picker-group', el('h5', '', g.name), items));
    }
    popover(anchor, host);
  }

  private layout(): void {
    const n = this.views.length;
    if (!n) return;
    const w = this.grid.clientWidth;
    const h = this.grid.clientHeight;
    const cols = Math.max(1, Math.min(n, Math.floor(w / 260)));
    const gap = 10;
    const pw = Math.floor((w - gap * (cols - 1)) / cols) - 1;
    const rows = Math.ceil(n / cols);
    this.grid.style.gridTemplateColumns = `repeat(${cols}, minmax(0, 1fr))`;
    // one row fills the strip: each canvas gets what its own legend leaves (several rows scroll)
    for (const v of this.views) v.resize(pw, Math.max(80, rows === 1 ? h - v.header - 4 : 150));
  }
}
