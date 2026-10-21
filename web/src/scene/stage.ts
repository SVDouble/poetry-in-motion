/**
 * The viewport: a plane of motion seen straight on, framed to a region of the world. It maps between world
 * coordinates (m, y up) and CSS pixels; the schematic draws with that mapping, the builder places its
 * handles with it, and the pointer is read back through it.
 */
export interface Mapping {
  /** Pixels per metre. */
  k: number;
  /** Screen position of the world origin: X = x0 + k x, Y = y0 − k y. */
  x0: number;
  y0: number;
  /** Viewport size (CSS pixels). */
  w: number;
  h: number;
}

export class Stage {
  private view = { cx: 0, cy: 0, width: 4, height: 4 };
  private map: Mapping = { k: 1, x0: 0, y0: 0, w: 1, h: 1 };

  constructor(private readonly container: HTMLElement) {
    new ResizeObserver(() => this.resize()).observe(container);
    this.resize();
  }

  /** Frame (contain) the `width × height` region (m) centred at (cx, cy). */
  setView(cx: number, cy: number, width: number, height: number): void {
    this.view = { cx, cy, width, height };
    this.resize();
  }

  resize(): void {
    const w = Math.max(1, this.container.clientWidth);
    const h = Math.max(1, this.container.clientHeight);
    const { cx, cy, width, height } = this.view;
    const k = Math.min(w / Math.max(width, 1e-6), h / Math.max(height, 1e-6));
    this.map = { k, x0: w / 2 - cx * k, y0: h / 2 + cy * k, w, h };
  }

  mapping(): Mapping {
    return this.map;
  }

  /** Zoom by `factor` (> 1: closer) keeping the world point under the screen point (sx, sy) in place. */
  zoomAt(factor: number, sx: number, sy: number): void {
    const [wx, wy] = [(sx - this.map.x0) / this.map.k, (this.map.y0 - sy) / this.map.k];
    const f = Math.min(8, Math.max(1 / 8, factor));
    const v = this.view;
    const width = Math.min(200, Math.max(0.05, v.width / f));
    const height = Math.min(200, Math.max(0.05, v.height / f));
    // the point under the pointer stays: c' = w − (w − c) / f
    const s = width / v.width;
    this.view = { cx: wx - (wx - v.cx) * s, cy: wy - (wy - v.cy) * s, width, height };
    this.resize();
  }

  /** Move the view by (dx, dy) screen pixels (the drawing follows the pointer). */
  panBy(dx: number, dy: number): void {
    const k = this.map.k;
    this.view = { ...this.view, cx: this.view.cx - dx / k, cy: this.view.cy + dy / k };
    this.resize();
  }

  /** Pointer position → world coordinates. */
  toWorld(clientX: number, clientY: number): [number, number] {
    const r = this.container.getBoundingClientRect();
    const { k, x0, y0 } = this.map;
    return [(clientX - r.left - x0) / k, (y0 - (clientY - r.top)) / k];
  }

  /** World point → CSS pixels relative to the viewport. */
  toScreen(x: number, y: number): [number, number] {
    const { k, x0, y0 } = this.map;
    return [x0 + k * x, y0 - k * y];
  }

  /** Metres per CSS pixel (for snapping and hit radii). */
  get metresPerPixel(): number {
    return 1 / this.map.k;
  }
}
