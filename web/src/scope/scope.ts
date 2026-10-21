/**
 * The scope: rolling plots of any signals (see `signals.ts`). One recorder samples every watched signal
 * once; plots are views onto it, so adding or removing a signal keeps the history of the others. Each plot
 * shows its signals as chips with live values; presets switch between curated layouts.
 */
import uPlot from 'uplot';
import 'uplot/dist/uPlot.min.css';
import { fix, symbolTex, tex } from '../eq/tex';

/** Smallest vertical span of a plot: round-off noise (1e-15 … 1e-6) must not be magnified into curves. */
const MIN_SPAN = 1e-3;

/** y-range of a plot: the data with 5 % margin, but never narrower than MIN_SPAN (relative for large values). */
function yRange(_u: uPlot, lo: number | null, hi: number | null): uPlot.Range.MinMax {
  if (lo === null || hi === null || !Number.isFinite(lo) || !Number.isFinite(hi)) return [-1, 1];
  const mid = (lo + hi) / 2;
  const half = Math.max((hi - lo) * 0.55, (MIN_SPAN / 2) * Math.max(1, Math.abs(mid)));
  return [mid - half, mid + half];
}

import { ink, OLIVE, PLUM, palette } from '../scene/theme';

/** A colour of the current theme (CSS custom property). */
const cssVar = (name: string) => getComputedStyle(document.documentElement).getPropertyValue(name).trim();

import { closePopover, popover } from '../ui/popover';
import { button, el } from '../ui/widgets';
import type { Signal, Signals } from './signals';

export interface PlotDef {
  ids: string[];
}

/** An event of an experiment (a kick, a push…) at simulated time `t`. */
export interface Mark {
  t: number;
  label: string;
}

/** Tick labels with as many decimals as the tick spacing needs, and a true minus sign. */
function ticks(v: number[]): string[] {
  const step = v.length > 1 ? Math.abs(v[1]! - v[0]!) : 1;
  const dec = Math.max(0, Math.min(4, Math.ceil(-Math.log10(step) - 1e-9)));
  return v.map((x) => {
    const s = (Math.abs(x) < step * 1e-6 ? 0 : x).toFixed(dec);
    return s.startsWith('-') ? `−${s.slice(1)}` : s;
  });
}

/** A signal name typeset: symbols with subscripts (θ₁, u₂, e θ₁) in KaTeX, words as text. */
function chipLabel(label: string): HTMLElement {
  const out = el('span', 'l');
  label.split(' ').forEach((word, k) => {
    if (k) out.append(' ');
    const src = symbolTex(word);
    if (!src) {
      out.append(word);
      return;
    }
    const span = el('span', 'm');
    tex(span, src);
    out.append(span);
  });
  return out;
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
  OLIVE,
  PLUM,
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
    // time went back (a reset, or resuming from a recorded moment): what came after it did not happen
    if (time < last) this.n = this.index(time);
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

  /** Number of samples taken before `time` (binary search: the times increase). */
  index(time: number): number {
    let [lo, hi] = [0, this.n];
    while (lo < hi) {
      const mid = (lo + hi) >> 1;
      if (this.t[mid]! < time) lo = mid + 1;
      else hi = mid;
    }
    return lo;
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
    marks: () => readonly Mark[],
    private readonly cursor: () => number | null,
  ) {
    this.el = el('div', 'scope-plot');
    const chips = el('div', 'scope-chips');
    sigs.forEach((s, k) => {
      const color = ink(s.color ?? COLORS[k % COLORS.length]!);
      const value = el('span', 'v');
      this.values.push(value);
      const x = button('×', () => actions.remove(s.id), 'x', 'remove');
      const chip = el('span', 'scope-chip', el('i', '', ''), chipLabel(s.label), value, x);
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
    const font = '11px KaTeX_Main, "Latin Modern Roman", serif';
    const axis: uPlot.Axis = {
      stroke: cssVar('--muted'),
      grid: { stroke: cssVar('--line'), width: 1 },
      ticks: { show: false },
      font,
      size: 26,
    };
    // the unit on the y axis when every signal of the plot shares it
    const units = [...new Set(sigs.map((s) => s.unit))];
    const unit = units.length === 1 && units[0] ? units[0] : '';
    const accent = cssVar('--accent');
    const ink0 = cssVar('--ink');
    // experiment events as dashed vertical lines with their names; the moment shown while rewinding as a
    // solid one
    const drawMarks = (u: uPlot) => {
      const list = marks();
      const at = cursor();
      const { ctx } = u;
      const [top, bottom] = [u.bbox.top, u.bbox.top + u.bbox.height];
      const lo = u.scales.x!.min ?? 0;
      const hi = u.scales.x!.max ?? 0;
      if (at !== null && at >= lo && at <= hi) {
        const x = u.valToPos(at, 'x', true);
        ctx.save();
        ctx.strokeStyle = ink0;
        ctx.lineWidth = 1.2 * devicePixelRatio;
        ctx.beginPath();
        ctx.moveTo(x, top);
        ctx.lineTo(x, bottom);
        ctx.stroke();
        ctx.restore();
      }
      if (!list.length) return;
      ctx.save();
      ctx.strokeStyle = accent;
      ctx.fillStyle = accent;
      ctx.lineWidth = 1 * devicePixelRatio;
      ctx.setLineDash([4 * devicePixelRatio, 3 * devicePixelRatio]);
      ctx.font = `italic ${11 * devicePixelRatio}px KaTeX_Main, serif`;
      for (const m of list) {
        if (m.t < lo || m.t > hi) continue;
        const x = u.valToPos(m.t, 'x', true);
        ctx.beginPath();
        ctx.moveTo(x, top);
        ctx.lineTo(x, bottom);
        ctx.stroke();
        ctx.fillText(m.label, x + 3 * devicePixelRatio, top + 11 * devicePixelRatio);
      }
      ctx.restore();
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
          { ...axis, values: (_u, v) => v.map((t) => `${t.toFixed(0)} s`) },
          {
            ...axis,
            size: 46,
            values: (_u, v) => ticks(v),
            ...(unit ? { label: unit, labelSize: 14, labelFont: `italic ${font}`, labelGap: 0 } : {}),
          },
        ],
        hooks: { draw: [drawMarks] },
        series: [
          {},
          ...sigs.map((s, k) => ({
            label: s.label,
            stroke: ink(s.color ?? COLORS[k % COLORS.length]!),
            width: 1.8,
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
    // the values at the moment shown (rewinding), else now
    const at = this.cursor();
    const k0 = at === null || !rec.n ? -1 : Math.min(rec.n - 1, rec.index(at));
    this.sigs.forEach((s, k) => {
      const text = fix(k0 < 0 ? s.get() : rec.series(s.id)[k0]!);
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
  /** Events of the running experiment, drawn on every plot. */
  private marks: Mark[] = [];
  /** The recorded moment being shown (rewinding), or null: now. */
  cursor: number | null = null;
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
    const last = this.rec.n ? this.rec.t[this.rec.n - 1]! : 0;
    if (t < last) this.marks = this.marks.filter((m) => m.t <= t); // time went back: forget what came after
    this.rec.sample(t);
  }

  /** Mark an event at simulated time `t` on every plot. */
  mark(label: string, t: number): void {
    this.marks.push({ t, label });
  }

  clearMarks(): void {
    this.marks = [];
  }

  draw(): void {
    for (const v of this.views) v.draw(this.rec);
  }

  clear(): void {
    for (const v of this.views) v.destroy();
    this.views = [];
    this.rec.n = 0;
    this.marks = [];
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
      const view = new PlotView(
        sigs,
        {
          remove: (id) => this.edit(() => def.ids.splice(def.ids.indexOf(id), 1)),
          add: (anchor) => this.pick(anchor, def),
          drop: () => this.edit(() => this.defs.splice(k, 1)),
        },
        () => this.marks,
        () => this.cursor,
      );
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
