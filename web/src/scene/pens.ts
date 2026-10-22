import { GREEK, tex } from '../eq/tex';
import { type Ink, type Inks, type Pen, PRINT, THIN } from './drawing';

// ------------------------------------------------------------------------------------------- pens

export class CanvasPen implements Pen {
  u = 1;
  private k = 1;
  private x0 = 0;
  private y0 = 0;

  constructor(
    readonly ctx: CanvasRenderingContext2D,
    private readonly labels: LabelLayer | null,
    public inks: Inks = PRINT,
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
export class SvgPen implements Pen {
  readonly inks = PRINT;
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
      `<text x="${X}" y="${Y}" text-anchor="middle" font-family="'Latin Modern Math','STIX Two Math','Cambria Math',serif" font-size="15" font-style="italic" fill="${this.inks.ink}">${texToSvg(src)}</text>`,
    );
  }
}

/** TikZ in centimetres (`scale` cm per metre); widths keep their proportion to the drawing. */
export class TikzPen implements Pen {
  readonly inks = PRINT;
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
    if (c === this.inks.ink) return 'black';
    if (c === this.inks.paper || c === '#fff') return 'white';
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
export class LabelLayer {
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
