/**
 * Textbook view: the mechanism as a flat black-on-white schematic, in the symbols of engineering mechanics
 * texts: bars, open hinge circles, pin supports on hatched ground, carts on hatched rails, the quartered
 * centre-of-mass symbol, joint angles as arcs from dash-dotted reference lines, and a zigzag spring for the
 * push.
 *
 * Everything is drawn through a {@link Pen}: on screen into a 2D canvas (the 3D renderer is idle meanwhile),
 * for export into SVG or TikZ, so a figure of the current pose can be pasted into notes. Parts that never
 * move (ground, rails, gravity) live on a second canvas redrawn only when the view changes, and a frame is
 * only drawn when something on it changed.
 */
import { tex } from '../eq/tex';
import type { Derivation } from '../model/derivation';
import { actuatorIndex, coordIndex, type Mechanism, normalise } from '../model/description';
import type { SlotView, Snapshot } from '../sim/layout';
import { BLOCK, type Bounds, compose, type Pose } from './geometry';
import type { Mapping } from './stage';
import { css, ink, palette, SLOT_COLORS } from './theme';

/** Stroke and fill of one path. Widths and dashes are in symbol units (CSS pixels on screen). */
interface Ink {
  stroke?: string;
  fill?: string;
  width?: number;
  dash?: readonly number[];
  alpha?: number;
}

/** Drawing in world coordinates (m, y up); symbol sizes are multiples of `u` (m per CSS pixel). */
interface Pen {
  readonly u: number;
  begin(): void;
  move(x: number, y: number): void;
  line(x: number, y: number): void;
  /** Arc from angle a0 to a1 (rad, counter-clockwise when a1 > a0), joined to the current point. */
  arc(x: number, y: number, r: number, a0: number, a1: number): void;
  close(): void;
  draw(ink: Ink): void;
  /** A TeX label at (x, y) shifted by (dx, dy) pixels (y down), centred. */
  label(id: string, x: number, y: number, src: string, dx?: number, dy?: number): void;
}

/** The inks of one picture. */
interface Inks {
  ink: string;
  paper: string;
  muted: string;
  accent: string;
  input: string;
  warn: string;
  /** Coordinate frames. */
  frame: string;
  /** Shadow controllers, in their slot inks. */
  ghosts: string[];
}

const inks = (draw: (c: number) => string): Inks => ({
  ink: draw(palette.ink),
  paper: draw(palette.paper),
  muted: draw(palette.muted),
  accent: draw(palette.accent),
  input: draw(palette.input),
  warn: draw(palette.warn),
  frame: draw(palette.frame),
  ghosts: SLOT_COLORS.slice(1).map(draw),
});

/** Files are for paper: always ink on white. */
const PRINT = inks(css);

/** The inks being painted with (set for each picture; painting is synchronous). */
let C: Inks = PRINT;

const LINK_W = 3.8;
const THIN = 1.1;
const JOINT_R = 5;
const HATCH = 7;
const DASHDOT = [10, 3, 2, 3] as const;
const DASH = [5, 4] as const;

// ------------------------------------------------------------------------------------------- pens

class CanvasPen implements Pen {
  u = 1;
  private k = 1;
  private x0 = 0;
  private y0 = 0;

  constructor(
    readonly ctx: CanvasRenderingContext2D,
    private readonly labels: LabelLayer | null,
  ) {}

  /** Screen mapping: X = x0 + k x, Y = y0 − k y (CSS pixels). */
  map(k: number, x0: number, y0: number): void {
    this.k = k;
    this.x0 = x0;
    this.y0 = y0;
    this.u = 1 / k;
  }

  begin(): void {
    this.ctx.beginPath();
  }
  move(x: number, y: number): void {
    this.ctx.moveTo(this.x0 + this.k * x, this.y0 - this.k * y);
  }
  line(x: number, y: number): void {
    this.ctx.lineTo(this.x0 + this.k * x, this.y0 - this.k * y);
  }
  arc(x: number, y: number, r: number, a0: number, a1: number): void {
    this.ctx.arc(this.x0 + this.k * x, this.y0 - this.k * y, this.k * r, -a0, -a1, a1 > a0);
  }
  close(): void {
    this.ctx.closePath();
  }
  draw(ink: Ink): void {
    const c = this.ctx;
    c.globalAlpha = ink.alpha ?? 1;
    if (ink.fill) {
      c.fillStyle = ink.fill;
      c.fill();
    }
    if (ink.stroke) {
      c.strokeStyle = ink.stroke;
      c.lineWidth = ink.width ?? THIN;
      c.setLineDash(ink.dash ?? []);
      c.stroke();
    }
  }
  label(id: string, x: number, y: number, src: string, dx = 0, dy = 0): void {
    this.labels?.place(id, this.x0 + this.k * x + dx, this.y0 - this.k * y + dy, src);
  }
}

const n2 = (v: number) => (Math.abs(v) < 5e-4 ? '0' : v.toFixed(3).replace(/\.?0+$/, ''));

/** SVG in screen pixels: what is on screen, as a standalone file cropped to the drawing. */
class SvgPen implements Pen {
  readonly u: number;
  private d = '';
  readonly out: string[] = [];
  /** Extent of everything drawn (pixels): min x, min y, max x, max y. */
  readonly box = [Infinity, Infinity, -Infinity, -Infinity];

  constructor(
    private readonly k: number,
    private readonly x0: number,
    private readonly y0: number,
  ) {
    this.u = 1 / k;
  }
  /** "X Y" in pixels, growing the extent by `pad` around the point. */
  private at(x: number, y: number, pad = 2): string {
    const X = this.x0 + this.k * x;
    const Y = this.y0 - this.k * y;
    const b = this.box;
    b[0] = Math.min(b[0]!, X - pad);
    b[1] = Math.min(b[1]!, Y - pad);
    b[2] = Math.max(b[2]!, X + pad);
    b[3] = Math.max(b[3]!, Y + pad);
    return `${n2(X)} ${n2(Y)}`;
  }
  begin(): void {
    this.d = '';
  }
  move(x: number, y: number): void {
    this.d += `M${this.at(x, y)}`;
  }
  line(x: number, y: number): void {
    this.d += `L${this.at(x, y)}`;
  }
  arc(x: number, y: number, r: number, a0: number, a1: number): void {
    const steps = Math.max(2, Math.ceil(Math.abs(a1 - a0) / 0.12));
    for (let s = 0; s <= steps; s++) {
      const a = a0 + ((a1 - a0) * s) / steps;
      const px = x + r * Math.cos(a);
      const py = y + r * Math.sin(a);
      this.d += `${s === 0 && !this.d ? 'M' : 'L'}${this.at(px, py)}`;
    }
  }
  close(): void {
    this.d += 'Z';
  }
  draw(ink: Ink): void {
    const a = [`d="${this.d}"`, `fill="${ink.fill ?? 'none'}"`];
    if (ink.stroke) {
      a.push(`stroke="${ink.stroke}"`, `stroke-width="${n2(ink.width ?? THIN)}"`);
      a.push('stroke-linecap="round"', 'stroke-linejoin="round"');
      if (ink.dash) a.push(`stroke-dasharray="${ink.dash.join(' ')}"`);
    }
    if (ink.alpha !== undefined && ink.alpha < 1) a.push(`opacity="${n2(ink.alpha)}"`);
    this.out.push(`<path ${a.join(' ')}/>`);
  }
  label(_id: string, x: number, y: number, src: string, dx = 0, dy = 0): void {
    this.at(x + dx * this.u, y - dy * this.u, 14);
    const X = n2(this.x0 + this.k * x + dx);
    const Y = n2(this.y0 - this.k * y + dy + 5);
    this.out.push(
      `<text x="${X}" y="${Y}" text-anchor="middle" font-family="'Latin Modern Math','STIX Two Math','Cambria Math',serif" font-size="15" font-style="italic" fill="${C.ink}">${texToSvg(src)}</text>`,
    );
  }
}

/** TikZ in centimetres (`scale` cm per metre); widths keep their proportion to the drawing. */
class TikzPen implements Pen {
  readonly u: number;
  private d = '';
  private started = false;
  readonly out: string[] = [];
  private readonly colors = new Map<string, string>();

  constructor(
    u: number,
    private readonly scale: number,
  ) {
    this.u = u;
  }
  private P = (x: number, y: number) => `(${n2(x * this.scale)},${n2(y * this.scale)})`;
  /** Pixels → points at this scale. */
  private pt = (px: number) => n2(px * this.u * this.scale * 28.4528);
  private color(c: string): string {
    if (c === C.ink) return 'black';
    if (c === C.paper || c === '#fff') return 'white';
    let name = this.colors.get(c);
    if (!name) {
      name = `c${this.colors.size + 1}`;
      this.colors.set(c, name);
    }
    return name;
  }
  begin(): void {
    this.d = '';
    this.started = false;
  }
  move(x: number, y: number): void {
    this.d += ` ${this.P(x, y)}`;
    this.started = true;
  }
  line(x: number, y: number): void {
    this.d += ` -- ${this.P(x, y)}`;
  }
  arc(x: number, y: number, r: number, a0: number, a1: number): void {
    const start = this.P(x + r * Math.cos(a0), y + r * Math.sin(a0));
    this.d += this.started ? ` -- ${start}` : ` ${start}`;
    this.started = true;
    const deg = (a: number) => n2((a * 180) / Math.PI);
    this.d += ` arc[start angle=${deg(a0)}, end angle=${deg(a1)}, radius=${n2(r * this.scale)}]`;
  }
  close(): void {
    this.d += ' -- cycle';
  }
  draw(ink: Ink): void {
    const o: string[] = [];
    if (ink.stroke) {
      o.push(`draw=${this.color(ink.stroke)}`, `line width=${this.pt(ink.width ?? THIN)}pt`);
      if (ink.dash) {
        const on = ink.dash.map((v, i) => `${i % 2 ? 'off' : 'on'} ${this.pt(v)}pt`).join(' ');
        o.push(`dash pattern=${on}`);
      }
    }
    if (ink.fill) o.push(`fill=${this.color(ink.fill)}`);
    if (ink.alpha !== undefined && ink.alpha < 1) o.push(`opacity=${n2(ink.alpha)}`);
    this.out.push(`  \\path[${o.join(', ')}]${this.d};`);
  }
  label(_id: string, x: number, y: number, src: string, dx = 0, dy = 0): void {
    this.out.push(`  \\node at ${this.P(x + dx * this.u, y - dy * this.u)} {$${src}$};`);
  }
  preamble(): string[] {
    return [...this.colors].map(
      ([hex, name]) => `\\definecolor{${name}}{HTML}{${hex.slice(1).toUpperCase()}}`,
    );
  }
}

const GREEK: Record<string, string> = {
  alpha: 'α',
  beta: 'β',
  gamma: 'γ',
  delta: 'δ',
  theta: 'θ',
  vartheta: 'ϑ',
  varphi: 'φ',
  phi: 'ϕ',
  psi: 'ψ',
  tau: 'τ',
  lambda: 'λ',
  mu: 'μ',
  omega: 'ω',
  ell: 'ℓ',
};

/** Simple TeX (letters, Greek, sub- and superscripts) as SVG text with tspans. */
function texToSvg(src: string): string {
  const esc = (t: string) => t.replace(/&/g, '&amp;').replace(/</g, '&lt;');
  let out = '';
  let k = 0;
  /** One atom: a group {…}, a command, or a character. */
  const atom = (): string => {
    const c = src[k++] ?? '';
    if (c === '{') {
      let depth = 1;
      let inner = '';
      while (k < src.length && depth > 0) {
        const d = src[k++]!;
        depth += d === '{' ? 1 : d === '}' ? -1 : 0;
        if (depth > 0) inner += d;
      }
      return texToSvg(inner);
    }
    if (c === '\\') {
      const word = /^[a-zA-Z]+/.exec(src.slice(k))?.[0] ?? '';
      k += word.length;
      return esc(GREEK[word] ?? word);
    }
    return c === '}' ? '' : esc(c);
  };
  while (k < src.length) {
    const c = src[k]!;
    if (c === '_' || c === '^') {
      k++;
      const shift = c === '_' ? 'sub' : 'super';
      out += `<tspan baseline-shift="${shift}" font-size="11" font-style="normal">${atom()}</tspan>`;
    } else out += atom();
  }
  return out;
}

// ------------------------------------------------------------------------------------------- labels

/** KaTeX labels over the canvas: rendered once, then only moved (and only when they moved). */
class LabelLayer {
  readonly el = document.createElement('div');
  private readonly items = new Map<
    string,
    { el: HTMLElement; src: string; x: number; y: number; on: boolean }
  >();
  private used = new Set<string>();

  constructor() {
    this.el.className = 'schematic-labels';
  }

  start(): void {
    this.used.clear();
  }

  place(id: string, x: number, y: number, src: string): void {
    let it = this.items.get(id);
    if (!it) {
      const el = document.createElement('div');
      el.className = 'slabel';
      tex(el, src);
      this.el.append(el);
      it = { el, src, x: Number.NaN, y: Number.NaN, on: true };
      this.items.set(id, it);
    } else if (it.src !== src) {
      tex(it.el, src);
      it.src = src;
    }
    this.used.add(id);
    if (!it.on) {
      it.el.style.display = '';
      it.on = true;
    }
    // (a new label starts at NaN, which compares false: test for "not close" instead)
    if (!(Math.abs(x - it.x) <= 0.3 && Math.abs(y - it.y) <= 0.3)) {
      it.x = x;
      it.y = y;
      it.el.style.transform = `translate3d(${x.toFixed(1)}px, ${y.toFixed(1)}px, 0) translate(-50%, -50%)`;
    }
  }

  finish(): void {
    for (const [id, it] of this.items) {
      if (it.on && !this.used.has(id)) {
        it.el.style.display = 'none';
        it.on = false;
      }
    }
  }
}

// ------------------------------------------------------------------------------------------- symbols

/** A line on the ground with hatching on its left side (seen from (x0, y0) towards (x1, y1)). */
function hatched(pen: Pen, x0: number, y0: number, x1: number, y1: number): void {
  const len = Math.hypot(x1 - x0, y1 - y0);
  if (len < 1e-9) return;
  const [tx, ty] = [(x1 - x0) / len, (y1 - y0) / len];
  const [nx, ny] = [-ty, tx];
  const step = HATCH * pen.u;
  const h = HATCH * pen.u;
  pen.begin();
  pen.move(x0, y0);
  pen.line(x1, y1);
  pen.draw({ stroke: C.ink, width: 1.4 });
  pen.begin();
  for (let s = step * 0.5; s <= len; s += step) {
    const bx = x0 + tx * s;
    const by = y0 + ty * s;
    pen.move(bx, by);
    pen.line(bx + (nx - tx) * h * 0.75, by + (ny - ty) * h * 0.75);
  }
  pen.draw({ stroke: C.ink, width: 0.8 });
}

/** Pin support: a triangle from the hinge at (x, y) towards direction `dir`, standing on hatched ground. */
function pinSupport(pen: Pen, x: number, y: number, dir: number): void {
  const u = pen.u;
  const [cx, cy] = [Math.cos(dir), Math.sin(dir)];
  const [nx, ny] = [-cy, cx];
  const h = 15 * u;
  const w = 9 * u;
  pen.begin();
  pen.move(x, y);
  pen.line(x + cx * h + nx * w, y + cy * h + ny * w);
  pen.line(x + cx * h - nx * w, y + cy * h - ny * w);
  pen.close();
  pen.draw({ stroke: C.ink, fill: C.paper, width: 1.4 });
  const b = 15 * u;
  hatched(pen, x + cx * h - nx * b, y + cy * h - ny * b, x + cx * h + nx * b, y + cy * h + ny * b);
}

function circle(pen: Pen, x: number, y: number, r: number, ink: Ink): void {
  pen.begin();
  pen.arc(x, y, r, 0, 2 * Math.PI);
  pen.close();
  pen.draw(ink);
}

/** A hinge: an open circle; a driven one (a motor in the joint) gets a second, concentric ring. */
function hinge(pen: Pen, x: number, y: number, ink = C.ink, driven = false): void {
  if (driven) circle(pen, x, y, (JOINT_R + 3.2) * pen.u, { stroke: ink, fill: C.paper, width: 1 });
  circle(pen, x, y, JOINT_R * pen.u, { stroke: ink, fill: C.paper, width: 1.5 });
}

/** The centre-of-mass symbol: a circle with two opposite quadrants filled. */
function comSymbol(pen: Pen, x: number, y: number, p: number): void {
  const r = 5 * pen.u;
  circle(pen, x, y, r, { stroke: C.ink, fill: C.paper, width: 1 });
  for (const a of [p, p + Math.PI]) {
    pen.begin();
    pen.move(x, y);
    pen.arc(x, y, r, a, a + Math.PI / 2);
    pen.close();
    pen.draw({ fill: C.ink });
  }
}

/** Arrow head at (x, y) pointing along `dir`. */
function head(pen: Pen, x: number, y: number, dir: number, ink: string, size = 7): void {
  const s = size * pen.u;
  const [c, sn] = [Math.cos(dir), Math.sin(dir)];
  pen.begin();
  pen.move(x, y);
  pen.line(x - c * s - sn * s * 0.38, y - sn * s + c * s * 0.38);
  pen.line(x - c * s + sn * s * 0.38, y - sn * s - c * s * 0.38);
  pen.close();
  pen.draw({ fill: ink });
}

function arrow(pen: Pen, x0: number, y0: number, x1: number, y1: number, ink: string, width = 1.2): void {
  const dir = Math.atan2(y1 - y0, x1 - x0);
  const back = 5 * pen.u;
  pen.begin();
  pen.move(x0, y0);
  pen.line(x1 - Math.cos(dir) * back, y1 - Math.sin(dir) * back);
  pen.draw({ stroke: ink, width });
  head(pen, x1, y1, dir, ink);
}

/** An arc arrow around (x, y) from angle a0 to a1, head at a1. */
function arcArrow(pen: Pen, x: number, y: number, r: number, a0: number, a1: number, ink: string): void {
  if (Math.abs(a1 - a0) * r < 6 * pen.u) return;
  const s = Math.sign(a1 - a0);
  const trim = (6 * pen.u) / r;
  pen.begin();
  pen.arc(x, y, r, a0, a1 - s * trim);
  pen.draw({ stroke: ink, width: 1.2 });
  head(pen, x + r * Math.cos(a1), y + r * Math.sin(a1), a1 + (s * Math.PI) / 2, ink);
}

/** A zigzag spring from (x0, y0) to (x1, y1) with straight ends. */
function spring(pen: Pen, x0: number, y0: number, x1: number, y1: number, ink: string): void {
  const len = Math.hypot(x1 - x0, y1 - y0);
  if (len < 1e-6) return;
  const [tx, ty] = [(x1 - x0) / len, (y1 - y0) / len];
  const [nx, ny] = [-ty, tx];
  const end = Math.min(len * 0.15, 10 * pen.u);
  const body = len - 2 * end;
  const coils = Math.max(4, Math.min(14, Math.round(body / (9 * pen.u))));
  const amp = 6 * pen.u;
  pen.begin();
  pen.move(x0, y0);
  pen.line(x0 + tx * end, y0 + ty * end);
  for (let k = 0; k < coils; k++) {
    const s = end + (body * (k + 0.5)) / coils;
    const side = k % 2 ? -1 : 1;
    pen.line(x0 + tx * s + nx * amp * side, y0 + ty * s + ny * amp * side);
  }
  pen.line(x1 - tx * end, y1 - ty * end);
  pen.line(x1, y1);
  pen.draw({ stroke: ink, width: 1.2 });
}

const wrap = (a: number) => Math.atan2(Math.sin(a), Math.cos(a));

// ------------------------------------------------------------------------------------------- view

export interface SchematicFrame {
  snap: Snapshot;
  params: number[];
  running: boolean;
  /** Pointer the tool is pushed towards, or null. */
  push: [number, number] | null;
  /** Shadow slots to draw (empty when hidden). */
  ghosts: readonly SlotView[];
  /** Joint coordinates, masses and the gravity symbol. */
  annotate: boolean;
  /** The coordinate frames {s}, {1} … {n} and {t}. */
  frames: boolean;
  /** The tool's path (of every slot shown). */
  trail: boolean;
}

const TRAIL = 240;

/** A ring buffer of recent tool positions. */
class Path {
  readonly xy = new Float64Array(2 * TRAIL);
  len = 0;
  head = 0;

  reset(): void {
    this.len = 0;
    this.head = 0;
  }

  /** Points closer than 1 mm to the last one are skipped. */
  push(x: number, y: number): void {
    if (this.len > 0) {
      const last = (this.head - 1 + TRAIL) % TRAIL;
      const dx = x - this.xy[2 * last]!;
      const dy = y - this.xy[2 * last + 1]!;
      if (dx * dx + dy * dy < 1e-6) return;
    }
    this.xy[2 * this.head] = x;
    this.xy[2 * this.head + 1] = y;
    this.head = (this.head + 1) % TRAIL;
    this.len = Math.min(TRAIL, this.len + 1);
  }

  draw(pen: Pen, ink: string, dash: readonly number[]): void {
    if (this.len < 2) return;
    pen.begin();
    const first = (this.head - this.len + TRAIL) % TRAIL;
    for (let k = 0; k < this.len; k++) {
      const j = (first + k) % TRAIL;
      if (k === 0) pen.move(this.xy[2 * j]!, this.xy[2 * j + 1]!);
      else pen.line(this.xy[2 * j]!, this.xy[2 * j + 1]!);
    }
    pen.draw({ stroke: ink, width: 1, dash });
  }
}

/** A coordinate frame: x and y axes as arrows from (x, y), turned by p. */
function triad(pen: Pen, x: number, y: number, p: number, size = 26): void {
  const s = size * pen.u;
  arrow(pen, x, y, x + Math.cos(p) * s, y + Math.sin(p) * s, C.frame, 1);
  arrow(pen, x, y, x - Math.sin(p) * s * 0.8, y + Math.cos(p) * s * 0.8, C.frame, 1);
}

/** The picture itself, independent of the screen: painting, the tool's path, and the exports. */
export class Figure {
  private readonly coord: number[];
  private readonly actuator: number[];
  /** Tool paths: the primary, then the shadow slots. */
  private readonly paths = SLOT_COLORS.map(() => new Path());
  /** Where an export puts the gravity arrow (null: the corner of the reachable region). */
  private gAt: [number, number] | null = null;
  /** Highlighted body (−1: none). */
  selected = -1;
  /** Label boxes of the picture being painted (world x0, y0, x1, y1, …): a label that would overlap is left out. */
  private readonly boxes: number[] = [];

  constructor(
    readonly mech: Mechanism,
    private readonly d: Derivation,
    private readonly bounds: Bounds,
  ) {
    this.coord = coordIndex(mech);
    this.actuator = actuatorIndex(mech);
  }

  resetTrail(): void {
    for (const p of this.paths) p.reset();
  }

  private length(i: number, params: number[]): number {
    return params[5 * i + 1] ?? this.mech.bodies[i]?.l ?? 0;
  }

  private jointPose(i: number, snap: Snapshot, params: number[]): Pose {
    const b = this.mech.bodies[i]!;
    if (b.parent < 0) return [b.mountX, b.mountY, b.mountAngle];
    return compose(snap.frame(b.parent + 1), [b.mountS * this.length(b.parent, params), 0, b.mountAngle]);
  }

  private port(i: number, s: number, snap: Snapshot, params: number[]): [number, number] {
    const [x, y, p] = snap.frame(i + 1);
    const l = s * this.length(i, params);
    return [x + Math.cos(p) * l, y + Math.sin(p) * l];
  }

  /** Record the tool position of slot k (0: the primary). */
  track(x: number, y: number, k = 0): void {
    this.paths[k]?.push(x, y);
  }

  /** The current picture as a standalone SVG (screen pixels). */
  svg(m: Mapping, f: SchematicFrame): string {
    const pen = new SvgPen(m.k, m.x0, m.y0);
    this.paintExport(pen, f);
    const [x0, y0, x1, y1] = pen.box.map((v, i) => (i < 2 ? Math.floor(v - 10) : Math.ceil(v + 10)));
    const [w, h] = [x1! - x0!, y1! - y0!];
    return [
      `<svg xmlns="http://www.w3.org/2000/svg" width="${w}" height="${h}" viewBox="${x0} ${y0} ${w} ${h}">`,
      `<rect x="${x0}" y="${y0}" width="${w}" height="${h}" fill="#ffffff"/>`,
      ...pen.out,
      '</svg>',
    ].join('\n');
  }

  /** The current picture as a TikZ figure about 12 cm wide. */
  tikz(m: Mapping, f: SchematicFrame): string {
    const scale = 12 / Math.max(this.bounds.width, 1e-3);
    const pen = new TikzPen(1 / m.k, scale);
    this.paintExport(pen, f);
    return [
      `% ${this.mech.name} at t = ${f.snap.t.toFixed(2)} s — exported from Poetry in Motion`,
      '% needs \\usepackage{tikz}',
      ...pen.preamble(),
      '\\begin{tikzpicture}[line cap=round, line join=round]',
      ...pen.out,
      '\\end{tikzpicture}',
    ].join('\n');
  }

  /** The whole picture for a file: no shadow controllers, and gravity drawn beside the mechanism as it
   * stands (on screen it sits in a corner of the region the mechanism can reach). */
  private paintExport(pen: Pen, f: SchematicFrame): void {
    const screen = C;
    C = PRINT;
    let [maxX, maxY] = [-Infinity, -Infinity];
    for (let i = 0; i < this.mech.bodies.length; i++) {
      const [x, y, p] = f.snap.frame(i + 1);
      const l = this.length(i, f.params);
      maxX = Math.max(maxX, x, x + Math.cos(p) * l);
      maxY = Math.max(maxY, y, y + Math.sin(p) * l);
    }
    this.gAt = Number.isFinite(maxX) ? [maxX + 0.35, maxY] : null;
    this.paintStatic(pen, f.annotate);
    this.paintDynamic(pen, { ...f, ghosts: [] });
    this.gAt = null;
    C = screen;
  }

  // ----------------------------------------------------------------------------------- static layer

  /** Ground supports, rails on the ground, ground pins and slots, gravity. */
  paintStatic(pen: Pen, annotate: boolean): void {
    const u = pen.u;
    this.mech.bodies.forEach((raw) => {
      if (raw.parent >= 0) return;
      const b = normalise(raw);
      if (b.joint === 'prismatic') {
        const a = b.mountAngle + b.axisAngle;
        const [c, s] = [Math.cos(a), Math.sin(a)];
        const half = (b.limit > 0 ? b.limit : 2) + (b.shape === 'block' ? BLOCK.w / 2 : 0.05);
        // the rail under the wheels (or through the slider), hatched below
        const off = b.shape === 'block' ? -(BLOCK.h / 2 + 0.06) : -7 * u;
        const [ox, oy] = [b.mountX - s * off, b.mountY + c * off];
        hatched(pen, ox + c * half, oy + s * half, ox - c * half, oy - s * half);
        if (b.limit > 0) {
          for (const sgn of [-1, 1]) {
            const ex = ox + c * sgn * half;
            const ey = oy + s * sgn * half;
            pen.begin();
            pen.move(ex, ey);
            pen.line(ex - s * 16 * u, ey + c * 16 * u);
            pen.draw({ stroke: C.ink, width: 2.6 });
          }
        }
      } else if (b.joint === 'revolute') {
        pinSupport(pen, b.mountX, b.mountY, b.mountAngle + Math.PI);
      } else {
        // welded to the ground: a hatched wall behind the root
        const dir = b.mountAngle + Math.PI;
        const [c, s] = [Math.cos(dir), Math.sin(dir)];
        const w = 16 * u;
        hatched(pen, b.mountX - s * w, b.mountY + c * w, b.mountX + s * w, b.mountY - c * w);
      }
    });
    for (const c of this.mech.closures) {
      if (c.a >= 0) continue;
      if (c.kind === 'pin') pinSupport(pen, c.gx, c.gy, -Math.PI / 2);
      else this.slotGuide(pen, c.gx, c.gy, c.angle, true);
    }
    if (annotate && this.mech.g > 0) {
      const [bx, by] = this.gravityAt();
      arrow(pen, bx, by, bx, by - 0.3, C.ink, 1.3);
    }
  }

  /** Where the gravity arrow starts (its label is drawn with the moving layer, where labels live). */
  private gravityAt(): [number, number] {
    if (this.gAt) return this.gAt;
    return [this.bounds.cx + this.bounds.width / 2 - 0.3, this.bounds.cy - this.bounds.height / 2 + 0.6];
  }

  /** A slot: two parallel guide lines along `angle` through (x, y), hatched outside when on the ground. */
  private slotGuide(pen: Pen, x: number, y: number, angle: number, ground: boolean): void {
    const u = pen.u;
    const [c, s] = [Math.cos(angle), Math.sin(angle)];
    const half = 1.4;
    const gap = 7 * u;
    for (const side of [-1, 1]) {
      const ox = x - s * gap * side;
      const oy = y + c * gap * side;
      if (ground && side < 0) hatched(pen, ox + c * half, oy + s * half, ox - c * half, oy - s * half);
      else if (ground) hatched(pen, ox - c * half, oy - s * half, ox + c * half, oy + s * half);
      else {
        pen.begin();
        pen.move(ox - c * half, oy - s * half);
        pen.line(ox + c * half, oy + s * half);
        pen.draw({ stroke: C.ink, width: 1.2 });
      }
    }
  }

  // ----------------------------------------------------------------------------------- dynamic layer

  /** A label unless it would overlap one placed before it in this picture. */
  private label(pen: Pen, id: string, x: number, y: number, src: string, dx = 0, dy = 0): void {
    const glyphs = src.replace(/\\[a-zA-Z]+/g, 'x').replace(/[{}_^\s]/g, '').length;
    const w = (4.5 * glyphs + 2) * pen.u;
    const h = 9 * pen.u;
    const cx = x + dx * pen.u;
    const cy = y - dy * pen.u;
    const b = this.boxes;
    for (let k = 0; k < b.length; k += 4) {
      if (cx - w < b[k + 2]! && cx + w > b[k]! && cy - h < b[k + 3]! && cy + h > b[k + 1]!) return;
    }
    b.push(cx - w, cy - h, cx + w, cy + h);
    pen.label(id, x, y, src, dx, dy);
  }

  paintDynamic(pen: Pen, f: SchematicFrame): void {
    const { snap, params } = f;
    this.boxes.length = 0;
    const u = pen.u;
    const nb = this.mech.bodies.length;

    // tool paths and shadow controllers underneath everything
    if (f.running && f.trail && this.mech.tool >= 0) {
      this.paths[0]!.draw(pen, C.muted, [2, 3]);
      f.ghosts.forEach((slot, k) => {
        if (slot.on) this.paths[k + 1]?.draw(pen, C.ghosts[k % C.ghosts.length]!, [1, 3]);
      });
    }
    f.ghosts.forEach((slot, k) => {
      if (!slot.on) return;
      const ink = C.ghosts[k % C.ghosts.length]!;
      pen.begin();
      for (let i = 0; i < nb; i++) {
        const [x, y, p] = slot.frame(i + 1);
        const l = this.length(i, params);
        pen.move(x, y);
        pen.line(x + Math.cos(p) * l, y + Math.sin(p) * l);
      }
      pen.draw({ stroke: ink, width: 2.2, alpha: 0.55 });
      if (this.mech.tool >= 0)
        circle(pen, slot.v('TOOL'), slot.v('TOOL', 1), 3 * u, { fill: ink, alpha: 0.8 });
    });

    // prismatic guides on moving bodies, and slots that move with a body
    this.mech.bodies.forEach((b, i) => {
      if (b.joint !== 'prismatic' || b.parent < 0) return;
      const [jx, jy, jp] = this.jointPose(i, snap, params);
      const a = jp + b.axisAngle;
      const half = b.limit > 0 ? b.limit + 0.05 : 0.8;
      const [c, s] = [Math.cos(a), Math.sin(a)];
      pen.begin();
      pen.move(jx - c * half, jy - s * half);
      pen.line(jx + c * half, jy + s * half);
      pen.draw({ stroke: C.ink, width: 1, dash: DASH });
    });
    this.mech.closures.forEach((c) => {
      if (c.kind !== 'slot' || c.a < 0) return;
      const [ax, ay] = this.port(c.a, c.sa, snap, params);
      this.slotGuide(pen, ax, ay, snap.frame(c.a + 1)[2] + c.angle, false);
    });

    // bodies: blocks, then bars
    this.mech.bodies.forEach((raw, i) => {
      const b = normalise(raw);
      const [x, y, p] = snap.frame(i + 1);
      const ink = i === this.selected ? C.accent : C.ink;
      if (b.shape === 'block') {
        const [c, s] = [Math.cos(p), Math.sin(p)];
        const corner = (dx: number, dy: number): [number, number] => [
          x + c * dx - s * dy,
          y + s * dx + c * dy,
        ];
        pen.begin();
        const pts = [corner(-BLOCK.w / 2, -BLOCK.h / 2), corner(BLOCK.w / 2, -BLOCK.h / 2)];
        pts.push(corner(BLOCK.w / 2, BLOCK.h / 2), corner(-BLOCK.w / 2, BLOCK.h / 2));
        pen.move(...pts[0]!);
        for (const q of pts.slice(1)) pen.line(...q);
        pen.close();
        pen.draw({ stroke: ink, fill: C.paper, width: 1.8 });
        if (b.joint === 'prismatic') {
          for (const wx of [-0.11, 0.11]) {
            const [cx, cy] = corner(wx, -BLOCK.h / 2 - 0.02);
            circle(pen, cx, cy, 0.04, { stroke: ink, fill: C.paper, width: 1.4 });
            circle(pen, cx, cy, 1.4 * u, { fill: ink });
          }
        }
      }
      const l = this.length(i, params);
      if (l > 1e-3) {
        pen.begin();
        pen.move(x, y);
        pen.line(x + Math.cos(p) * l, y + Math.sin(p) * l);
        pen.draw({ stroke: ink, width: b.joint === 'fixed' ? LINK_W + 0.8 : LINK_W });
      }
      if (b.joint === 'prismatic' && b.shape !== 'block') {
        // a sleeve on the guide
        const [jx, jy, jp] = this.jointPose(i, snap, params);
        const a = jp + b.axisAngle;
        const [c, s] = [Math.cos(a), Math.sin(a)];
        const along = (x - jx) * c + (y - jy) * s;
        const [cx, cy] = [jx + c * along, jy + s * along];
        const [hw, hh] = [11 * u, 6 * u];
        pen.begin();
        pen.move(cx - c * hw + s * hh, cy - s * hw - c * hh);
        pen.line(cx + c * hw + s * hh, cy + s * hw - c * hh);
        pen.line(cx + c * hw - s * hh, cy + s * hw + c * hh);
        pen.line(cx - c * hw - s * hh, cy - s * hw + c * hh);
        pen.close();
        pen.draw({ stroke: ink, fill: C.paper, width: 1.5 });
      }
    });

    if (f.annotate && this.mech.g > 0) {
      const [bx, by] = this.gravityAt();
      this.label(pen, 'g', bx, by - 0.15, 'g', 12, 0);
    }

    // joints: hinges, welds, loop pins
    this.mech.bodies.forEach((b, i) => {
      const [x, y, p] = snap.frame(i + 1);
      const ink = i === this.selected ? C.accent : C.ink;
      if (b.joint === 'revolute') hinge(pen, x, y, ink, b.actuated);
      else if (b.joint === 'fixed') {
        const r = 4 * u;
        const [c, s] = [Math.cos(p + Math.PI / 4), Math.sin(p + Math.PI / 4)];
        pen.begin();
        pen.move(x + c * r, y + s * r);
        pen.line(x - s * r, y + c * r);
        pen.line(x - c * r, y - s * r);
        pen.line(x + s * r, y - c * r);
        pen.close();
        pen.draw({ fill: ink });
      }
    });
    this.mech.closures.forEach((c) => {
      const [px, py] = this.port(c.b, c.sb, snap, params);
      if (c.kind === 'slot') {
        const dir = (c.a < 0 ? 0 : snap.frame(c.a + 1)[2]) + c.angle;
        const [cs, sn] = [Math.cos(dir), Math.sin(dir)];
        const [hw, hh] = [9 * u, 5.5 * u];
        pen.begin();
        pen.move(px - cs * hw + sn * hh, py - sn * hw - cs * hh);
        pen.line(px + cs * hw + sn * hh, py + sn * hw - cs * hh);
        pen.line(px + cs * hw - sn * hh, py + sn * hw + cs * hh);
        pen.line(px - cs * hw - sn * hh, py - sn * hw + cs * hh);
        pen.close();
        pen.draw({ stroke: C.ink, fill: C.paper, width: 1.4 });
      }
      hinge(pen, px, py);
    });

    // joint coordinates: an arc from a dash-dotted reference line, or a dimension along the rail
    if (f.annotate) {
      this.mech.bodies.forEach((b, i) => {
        const j = this.coord[i]!;
        const info = j >= 0 ? this.d.coords[j] : undefined;
        if (!info) return;
        const [x, y, p] = snap.frame(i + 1);
        const [jx, jy, jp] = this.jointPose(i, snap, params);
        if (b.joint === 'revolute') {
          const ref = jp;
          const q = wrap(p - ref);
          const r = 30 * u;
          pen.begin();
          pen.move(x, y);
          pen.line(x + Math.cos(ref) * 46 * u, y + Math.sin(ref) * 46 * u);
          pen.draw({ stroke: C.ink, width: 0.8, dash: DASHDOT });
          arcArrow(pen, x, y, r, ref, ref + q, C.ink);
          const mid = ref + q / 2 + (Math.abs(q) < 0.35 ? 0.35 * Math.sign(q || 1) : 0);
          this.label(
            pen,
            `q${j}`,
            x + Math.cos(mid) * (r + 12 * u),
            y + Math.sin(mid) * (r + 12 * u),
            info.tex,
          );
        } else {
          const a = jp + b.axisAngle;
          const [c, s] = [Math.cos(a), Math.sin(a)];
          const along = (x - jx) * c + (y - jy) * s;
          // on the side of the rail away from the bodies that hang off the slider
          let side = 1;
          this.mech.bodies.forEach((child, k) => {
            if (child.parent === i && Math.sin(snap.frame(k + 1)[2] - a) > 0.3) side = -1;
          });
          const clear = b.shape === 'block' ? BLOCK.h / 2 + (side < 0 ? 0.08 : 0.03) : 0;
          const off = side * (clear + 22 * u);
          const [ox, oy] = [-s * off, c * off];
          pen.begin();
          pen.move(jx + ox * 0.6, jy + oy * 0.6);
          pen.line(jx + ox * 1.25, jy + oy * 1.25);
          pen.draw({ stroke: C.ink, width: 0.8 });
          if (Math.abs(along) > 8 * u) {
            arrow(pen, jx + ox, jy + oy, jx + c * along + ox, jy + s * along + oy, C.ink, 0.9);
          }
          this.label(pen, `q${j}`, jx + c * along * 0.5 + ox, jy + s * along * 0.5 + oy, info.tex, 0, -10);
        }
      });
    }

    // centres of mass (labelled after the coordinates, which win where labels would overlap)
    if (f.annotate) {
      this.mech.bodies.forEach((raw, i) => {
        const b = normalise(raw);
        if (b.joint === 'fixed' && b.m <= 0) return;
        const [x, y, p] = snap.frame(i + 1);
        const r = params[5 * i + 2] ?? b.r;
        const [cx, cy] = [x + Math.cos(p) * r, y + Math.sin(p) * r];
        if (b.shape === 'block') {
          // a cart: the label beside it, clear of whatever stands on it
          const off = BLOCK.w / 2 + 16 * u;
          this.label(pen, `m${i}`, cx - Math.cos(p) * off, cy - Math.sin(p) * off, `m_{${i + 1}}`);
          return;
        }
        comSymbol(pen, cx, cy, p);
        // on the clockwise side of the bar; joint angles open counter-clockwise
        const side = p - Math.PI / 2;
        this.label(pen, `m${i}`, cx + Math.cos(side) * 15 * u, cy + Math.sin(side) * 15 * u, `m_{${i + 1}}`);
      });
    }

    // actuator inputs: a torque arc at a hinge, a force arrow along a rail
    const umax = snap.v('UMAX') || 1;
    if (f.running && snap.v('KIND') !== 0) {
      this.mech.bodies.forEach((b, i) => {
        if (!b.actuated || b.joint === 'fixed') return;
        const a = this.actuator[i]!;
        if (a < 0) return;
        const v = Math.max(-1, Math.min(1, snap.v('U', a) / umax));
        if (Math.abs(v) < 0.02) return;
        const [x, y, p] = snap.frame(i + 1);
        if (b.joint === 'revolute') {
          arcArrow(pen, x, y, 17 * u, p + Math.PI / 2, p + Math.PI / 2 + v * 1.5 * Math.PI, C.input);
        } else {
          const [, , jp] = this.jointPose(i, snap, params);
          const ax = jp + b.axisAngle;
          const len = 0.5 * v;
          arrow(pen, x, y, x + Math.cos(ax) * len, y + Math.sin(ax) * len, C.input, 1.6);
        }
      });
    }

    // tool, targets, push
    const [tx, ty] = snap.frame(nb + 1);
    if (this.mech.tool >= 0) circle(pen, tx, ty, 2.6 * u, { fill: C.ink });
    if (f.running && snap.v('USE_TOOL') !== 0) {
      const gx = snap.v('TARGET');
      const gy = snap.v('TARGET', 1);
      const ink = snap.v('REACHABLE') !== 0 ? C.accent : C.warn;
      circle(pen, gx, gy, 8 * u, { stroke: ink, width: 1.2 });
      pen.begin();
      pen.move(gx - 12 * u, gy);
      pen.line(gx + 12 * u, gy);
      pen.move(gx, gy - 12 * u);
      pen.line(gx, gy + 12 * u);
      pen.draw({ stroke: ink, width: 0.9 });
    }
    if (f.running && snap.v('GOALS', 2) > 0) {
      const ex = snap.v('GOALS', 3);
      const ey = snap.v('GOALS', 4);
      circle(pen, ex, ey, 7 * u, { stroke: C.accent, width: 1, dash: [3, 2] });
    }
    if (f.running && snap.v('GOALS') > 0 && this.mech.tool >= 0) {
      const a = snap.v('GOALS', 1);
      arrow(pen, tx, ty, tx + Math.cos(a) * 32 * u, ty + Math.sin(a) * 32 * u, C.accent, 1);
    }
    if (f.push) spring(pen, tx, ty, f.push[0], f.push[1], C.input);

    // joint targets on actuated rails (where the joint controllers are taking them)
    if (f.running && snap.v('KIND') !== 0) {
      this.mech.bodies.forEach((b, i) => {
        const j = this.coord[i]!;
        if (b.joint !== 'prismatic' || !b.actuated || j < 0) return;
        const [jx, jy, jp] = this.jointPose(i, snap, params);
        const a = jp + b.axisAngle;
        const q = snap.v('Q_TARGET', j);
        const [px, py] = [jx + Math.cos(a) * q, jy + Math.sin(a) * q];
        const [nx, ny] = [-Math.sin(a), Math.cos(a)];
        const below = b.shape === 'block' ? BLOCK.h / 2 + 0.06 : 0;
        const [bx, by] = [px - nx * below, py - ny * below];
        const s = 6 * u;
        pen.begin();
        pen.move(bx, by);
        pen.line(bx - nx * s * 1.6 + Math.cos(a) * s, by - ny * s * 1.6 + Math.sin(a) * s);
        pen.line(bx - nx * s * 1.6 - Math.cos(a) * s, by - ny * s * 1.6 - Math.sin(a) * s);
        pen.close();
        pen.draw({ stroke: C.accent, fill: C.paper, width: 1.2 });
      });
    }

    // coordinate frames: {s} at the origin, {i} on every body, {t} at the tool
    if (f.frames) {
      triad(pen, 0, 0, 0, 32);
      this.label(pen, 'fs', 0, 0, String.raw`\{s\}`, -14, 12);
      for (let i = 0; i < nb; i++) {
        const [x, y, p] = snap.frame(i + 1);
        triad(pen, x, y, p);
        this.label(pen, `f${i}`, x, y, String.raw`\{${i + 1}\}`, -12, -12);
      }
      if (this.mech.tool >= 0) {
        const [, , tp] = snap.frame(nb + 1);
        triad(pen, tx, ty, tp, 20);
        this.label(pen, 'ft', tx, ty, String.raw`\{t\}`, 12, 12);
      }
    }
  }
}

/** The figure on screen: a canvas for the ground (redrawn when the view changes), one for everything that
 * moves (redrawn when something changed), and KaTeX labels over them. */
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
  /** The inks of the current theme. */
  private inks = inks(ink);
  /** Force the next frame to be drawn (selection or toggles changed). */
  dirty = true;

  constructor(mech: Mechanism, d: Derivation, bounds: Bounds) {
    this.fig = new Figure(mech, d, bounds);
    this.host.className = 'schematic';
    this.host.append(this.back, this.front, this.labels.el);
    this.backPen = new CanvasPen(this.back.getContext('2d')!, null);
    this.frontPen = new CanvasPen(this.front.getContext('2d')!, this.labels);
  }

  /** The theme changed: redraw everything in its inks. */
  restyle(): void {
    this.inks = inks(ink);
    this.staticKey = '';
    this.dirty = true;
  }

  select(i: number): void {
    if (i !== this.fig.selected) this.dirty = true;
    this.fig.selected = i;
  }

  resetTrail(): void {
    this.fig.resetTrail();
    this.dirty = true;
  }

  /** Draw a frame if anything on it changed. */
  draw(m: Mapping, f: SchematicFrame): void {
    C = this.inks;
    const dpr = Math.min(window.devicePixelRatio || 1, 2);
    const sKey = `${m.k.toFixed(4)},${m.x0.toFixed(2)},${m.y0.toFixed(2)},${m.w},${m.h},${dpr},${f.annotate}`;
    if (sKey !== this.staticKey) {
      this.staticKey = sKey;
      for (const c of [this.back, this.front]) {
        c.width = Math.round(m.w * dpr);
        c.height = Math.round(m.h * dpr);
      }
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
    const fKey = `${s.t},${tx},${ty},${s.v('TARGET')},${s.v('TARGET', 1)},${s.v('USE_TOOL')},${f.push},${f.ghosts.length},${f.running},${f.trail},${f.frames},${sum}`;
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
