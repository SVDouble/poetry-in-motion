/** Rolling time-series plots (uPlot) fed from the snapshot once per sample. */
import uPlot from 'uplot';
import 'uplot/dist/uPlot.min.css';

export interface SeriesSpec {
  label: string;
  color: string;
  get: () => number;
  dash?: number[];
}

export interface PlotSpec {
  title: string;
  unit: string;
  series: SeriesSpec[];
}

const WINDOW_S = 10;
const CAPACITY = 600;

class RollingPlot {
  readonly el: HTMLElement;
  private readonly plot: uPlot;
  private readonly cols: Float64Array[];
  private n = 0;

  constructor(
    private readonly spec: PlotSpec,
    host: HTMLElement,
  ) {
    this.el = document.createElement('div');
    this.el.className = 'plot';
    host.append(this.el);
    this.cols = Array.from({ length: spec.series.length + 1 }, () => new Float64Array(CAPACITY));
    const axis: uPlot.Axis = {
      stroke: 'rgba(200,210,235,0.55)',
      grid: { stroke: 'rgba(170,182,211,0.08)', width: 1 },
      ticks: { show: false },
      font: '11px Inter, system-ui, sans-serif',
      size: 34,
    };
    this.plot = new uPlot(
      {
        title: `${spec.title}${spec.unit ? ` [${spec.unit}]` : ''}`,
        width: 300,
        height: 150,
        pxAlign: 0,
        cursor: { show: true, drag: { x: false, y: false } },
        legend: { show: true, live: false },
        scales: { x: { time: false } },
        axes: [
          { ...axis, values: (_u, v) => v.map((t) => `${t.toFixed(0)}s`) },
          { ...axis, size: 44 },
        ],
        series: [
          {},
          ...spec.series.map((s) => ({
            label: s.label,
            stroke: s.color,
            width: 1.6,
            points: { show: false },
            ...(s.dash ? { dash: s.dash } : {}),
          })),
        ],
      },
      [[], ...spec.series.map(() => [])],
      this.el,
    );
  }

  sample(t: number): void {
    if (this.n > 0 && t < (this.cols[0]?.[this.n - 1] ?? 0)) this.n = 0; // time went backwards: reset
    if (this.n === CAPACITY) {
      for (const c of this.cols) c.copyWithin(0, 1);
      this.n--;
    }
    this.cols[0]![this.n] = t;
    this.spec.series.forEach((s, k) => {
      this.cols[k + 1]![this.n] = s.get();
    });
    this.n++;
  }

  draw(): void {
    const data = this.cols.map((c) => c.subarray(0, this.n)) as unknown as uPlot.AlignedData;
    const t1 = this.cols[0]![Math.max(0, this.n - 1)] ?? 0;
    this.plot.batch(() => {
      this.plot.setData(data, false);
      this.plot.setScale('x', { min: Math.max(0, t1 - WINDOW_S), max: Math.max(WINDOW_S, t1) });
    });
  }

  resize(w: number, h: number): void {
    this.plot.setSize({ width: w, height: h });
  }

  destroy(): void {
    this.plot.destroy();
    this.el.remove();
  }
}

export class Plots {
  private plots: RollingPlot[] = [];
  private lastT = -1;
  private readonly ro: ResizeObserver;

  constructor(private readonly host: HTMLElement) {
    this.ro = new ResizeObserver(() => this.layout());
    this.ro.observe(host);
  }

  mount(specs: PlotSpec[]): void {
    for (const p of this.plots) p.destroy();
    this.plots = specs.map((s) => new RollingPlot(s, this.host));
    this.lastT = -1;
    this.layout();
  }

  /** Sample the snapshot at simulation time `t` (skips duplicates while paused). */
  sample(t: number): void {
    if (t === this.lastT) return;
    this.lastT = t;
    for (const p of this.plots) p.sample(t);
  }

  draw(): void {
    for (const p of this.plots) p.draw();
  }

  private layout(): void {
    const n = this.plots.length;
    if (!n) return;
    const cs = getComputedStyle(this.host);
    const w = this.host.clientWidth - Number.parseFloat(cs.paddingLeft) - Number.parseFloat(cs.paddingRight);
    const h = this.host.clientHeight - Number.parseFloat(cs.paddingTop) - Number.parseFloat(cs.paddingBottom);
    const cols = w > 900 ? n : w > 560 ? Math.min(2, n) : 1;
    const gap = 10;
    const pw = Math.floor((w - gap * (cols - 1)) / cols) - 1; // 1 px slack: sub-pixel widths must not wrap
    const ph = cols === n ? h : 150;
    for (const p of this.plots) p.resize(pw, Math.max(110, ph - 44));
  }
}
