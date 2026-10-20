/**
 * Live LaTeX: equations are typeset once with KaTeX; numeric slots are tagged with `\htmlId` and
 * afterwards only their text nodes are rewritten (no re-typesetting, no layout of the formula).
 */
import katex from 'katex';

export type Getter = () => number;
export type Format = (v: number) => string;

const MINUS = '−';

/** Adaptive fixed-point: keeps ~4 significant digits with a stable width. */
export const fix: Format = (v) => {
  if (!Number.isFinite(v)) return '—';
  const a = Math.abs(v);
  const s = a >= 1000 ? a.toFixed(0) : a >= 100 ? a.toFixed(1) : a >= 10 ? a.toFixed(2) : a.toFixed(3);
  return (v < 0 && Number(s) !== 0 ? MINUS : '') + s;
};

export const sci: Format = (v) => {
  if (!Number.isFinite(v)) return '—';
  if (v === 0) return '0';
  const [m, e] = v.toExponential(1).split('e');
  return `${(m ?? '').replace('-', MINUS)}e${(e ?? '').replace('-', MINUS).replace('+', '')}`;
};

export const deg: Format = (v) => (Number.isFinite(v) ? `${fix((v * 180) / Math.PI)}°` : '—');

interface Binding {
  id: string;
  get: Getter;
  fmt: Format;
  el: HTMLElement | null;
  last: string;
}

let uid = 0;

export const KATEX_OPTIONS = {
  throwOnError: false,
  strict: 'ignore' as const,
  trust: (ctx: { command: string }) => ctx.command === '\\htmlId' || ctx.command === '\\htmlClass',
};

/** Typeset static TeX into an element. */
export function tex(el: HTMLElement, src: string, display = false): void {
  katex.render(src, el, { ...KATEX_OPTIONS, displayMode: display });
}

/** A group of live numbers that belong to one rendered block (e.g. one section of the panel). */
export class LiveScope {
  private bindings: Binding[] = [];
  private pending: Binding[] = [];

  /** A live scalar slot; `cls` adds a colour/emphasis class. */
  num(get: Getter, fmt: Format = fix, cls = 'lv'): string {
    const b: Binding = { id: `lv${uid++}`, get, fmt, el: null, last: '' };
    this.pending.push(b);
    // braces make the slot an ordinary atom so KaTeX keeps operator spacing around it
    return String.raw`{\htmlClass{${cls}}{\htmlId{${b.id}}{0.000}}}`;
  }

  /** A live `rows × cols` matrix. */
  mat(
    rows: number,
    cols: number,
    get: (i: number, j: number) => number,
    fmt: Format = fix,
    cls = 'lv',
  ): string {
    const body: string[] = [];
    for (let i = 0; i < rows; i++) {
      const row: string[] = [];
      for (let j = 0; j < cols; j++) row.push(this.num(() => get(i, j), fmt, `${cls} cell`));
      body.push(row.join(' & '));
    }
    return String.raw`\begin{bmatrix}${body.join(String.raw`\\`)}\end{bmatrix}`;
  }

  vec(n: number, get: (i: number) => number, fmt: Format = fix, cls = 'lv'): string {
    return this.mat(n, 1, (i) => get(i), fmt, cls);
  }

  /** Typeset `src` into `el` and resolve the live slots it contains. */
  render(el: HTMLElement, src: string, display = true): void {
    tex(el, src, display);
    for (const b of this.pending) {
      b.el = el.querySelector<HTMLElement>(`#${b.id}`);
      if (b.el) this.bindings.push(b);
    }
    this.pending = [];
    this.update();
  }

  /** Rewrite only the slots whose formatted text changed. */
  update(): void {
    for (const b of this.bindings) {
      const s = b.fmt(b.get());
      if (s !== b.last && b.el) {
        b.el.textContent = s;
        b.last = s;
      }
    }
  }
}

/** Symbolic `rows × cols` matrix from generated LaTeX strings. */
export function texMat(m: string[][], env = 'bmatrix'): string {
  return String.raw`\begin{${env}}${m.map((r) => r.join(' & ')).join(String.raw`\\`)}\end{${env}}`;
}
