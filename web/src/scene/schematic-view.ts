import type { Derivation } from '../model/derivation';
import type { Mechanism } from '../model/description';
import { inks } from './drawing';
import type { Bounds } from './geometry';
import { CanvasPen, LabelLayer } from './pens';
import { Figure, type SchematicFrame } from './schematic';
import type { Mapping } from './stage';
import { ink } from './theme';

export class SchematicView {
  readonly host = document.createElement('div');
  readonly fig: Figure;
  private readonly back = document.createElement('canvas');
  private readonly front = document.createElement('canvas');
  private readonly labels = new LabelLayer();
  private readonly backPen: CanvasPen;
  private readonly frontPen: CanvasPen;
  private staticKey = '';
  private frameKey = '';
  /** Force the next frame to be drawn (selection or toggles changed). */
  dirty = true;

  constructor(mech: Mechanism, d: Derivation, bounds: Bounds) {
    this.fig = new Figure(mech, d, bounds);
    this.host.className = 'schematic';
    this.host.append(this.back, this.front, this.labels.el);
    this.backPen = new CanvasPen(this.back.getContext('2d')!, null, inks(ink));
    this.frontPen = new CanvasPen(this.front.getContext('2d')!, this.labels, inks(ink));
  }

  /** The theme changed: redraw everything in its inks. */
  restyle(): void {
    this.backPen.inks = this.frontPen.inks = inks(ink);
    this.staticKey = '';
    this.dirty = true;
  }

  select(i: number): void {
    if (i !== this.fig.selected) this.dirty = true;
    this.fig.selected = i;
  }

  selectElement(k: number): void {
    if (k !== this.fig.selectedElement) this.dirty = true;
    this.fig.selectedElement = k;
  }

  resetTrail(): void {
    this.fig.resetTrail();
    this.dirty = true;
  }

  /** Draw a frame if anything on it changed. */
  draw(m: Mapping, f: SchematicFrame): void {
    const dpr = Math.min(window.devicePixelRatio || 1, 2);
    const sKey = `${m.k.toFixed(4)},${m.x0.toFixed(2)},${m.y0.toFixed(2)},${m.w},${m.h},${dpr},${f.annotate}`;
    if (sKey !== this.staticKey) {
      this.staticKey = sKey;
      for (const c of [this.back, this.front]) {
        c.width = Math.round(m.w * dpr);
        c.height = Math.round(m.h * dpr);
      }
      // gravity: an arrow in the bottom-right corner, clear of the overlays (in world coordinates)
      this.fig.screenG = [(m.w - 34 - m.x0) / m.k, (m.y0 - (m.h - 84)) / m.k];
      this.fig.screenKey = [(18 - m.x0) / m.k, (m.y0 - (m.h - 70)) / m.k]; // above the timeline
      this.prepare(this.backPen, m, dpr);
      this.fig.paintStatic(this.backPen, f.annotate);
      this.dirty = true;
    }
    const s = f.snap;
    const [tx, ty] = s.frame(this.fig.mech.bodies.length + 1);
    if (f.running && this.fig.mech.tool >= 0) {
      this.fig.track(tx, ty);
      f.ghosts.forEach((slot, k) => {
        if (slot.on) this.fig.track(slot.v('TOOL'), slot.v('TOOL', 1), k + 1);
      });
    }
    // parameters (lengths, centres of mass) can change while paused
    let sum = 0;
    for (let i = 0; i < f.params.length; i++) sum += f.params[i]! * (i + 1);
    const fKey = `${s.t},${tx},${ty},${s.v('TARGET')},${s.v('TARGET', 1)},${s.v('USE_TOOL')},${f.push},${f.ghosts.length},${f.running},${f.trail},${f.frames},${f.dims},${f.velocities},${f.forces},${f.freeBody},${f.focus ? JSON.stringify(f.focus) : ''},${sum}`;
    if (!this.dirty && fKey === this.frameKey) return;
    this.frameKey = fKey;
    this.dirty = false;
    this.prepare(this.frontPen, m, dpr);
    this.labels.start();
    this.fig.paintDynamic(this.frontPen, f);
    this.labels.finish();
  }

  private prepare(pen: CanvasPen, m: Mapping, dpr: number): void {
    const c = pen.ctx;
    c.setTransform(dpr, 0, 0, dpr, 0, 0);
    c.clearRect(0, 0, m.w, m.h);
    c.lineCap = 'round';
    c.lineJoin = 'round';
    pen.map(m.k, m.x0, m.y0);
  }
}
