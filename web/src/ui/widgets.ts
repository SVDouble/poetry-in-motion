/** Small DOM widgets shared by the builder and the simulation panels. */
import { fix, tex } from '../eq/tex';

export interface SliderOpts {
  label: string;
  /** Render the label as TeX. */
  tex?: boolean;
  min: number;
  max: number;
  step?: number;
  value: number;
  unit?: string;
  log?: boolean;
  format?: (v: number) => string;
  hint?: string;
  onInput: (v: number) => void;
}

const STEPS = 1000;

export type Slider = HTMLElement & { set(v: number): void };

export function slider(o: SliderOpts): Slider {
  const el = document.createElement('label') as HTMLLabelElement & { set(v: number): void };
  el.className = 'param';
  if (o.hint) el.title = o.hint;
  const name = document.createElement('span');
  name.className = 'param-name';
  if (o.tex) tex(name, o.label);
  else name.textContent = o.label;
  const out = document.createElement('output');
  out.className = 'param-value';
  const input = document.createElement('input');
  input.type = 'range';
  input.min = '0';
  input.max = String(STEPS);
  const to = (v: number) =>
    o.log ? (Math.log(v / o.min) / Math.log(o.max / o.min)) * STEPS : ((v - o.min) / (o.max - o.min)) * STEPS;
  const from = (t: number) => {
    const f = t / STEPS;
    const v = o.log ? o.min * (o.max / o.min) ** f : o.min + f * (o.max - o.min);
    return o.step ? Math.round(v / o.step) * o.step : Number(v.toPrecision(4));
  };
  const show = (v: number) => {
    out.textContent = `${(o.format ?? fix)(v)}${o.unit ? ` ${o.unit}` : ''}`;
  };
  input.value = String(to(o.value));
  show(o.value);
  input.addEventListener('input', () => {
    const v = from(Number(input.value));
    show(v);
    o.onInput(v);
  });
  el.set = (v: number) => {
    input.value = String(to(v));
    show(v);
  };
  el.append(name, out, input);
  return el;
}

export function segmented<T extends string>(
  options: readonly (readonly [T, string])[],
  value: T,
  onPick: (v: T) => void,
  small = true,
): HTMLElement {
  const seg = document.createElement('div');
  seg.className = `segmented${small ? ' small' : ''}`;
  for (const [v, label] of options) {
    const b = button(label, () => {
      for (const x of seg.children) x.classList.toggle('on', x === b);
      onPick(v);
    });
    b.className = '';
    b.classList.toggle('on', v === value);
    seg.append(b);
  }
  return seg;
}

export function button(label: string, onClick: () => void, cls = 'btn small', title = ''): HTMLButtonElement {
  const b = document.createElement('button');
  b.type = 'button';
  b.className = cls;
  b.textContent = label;
  if (title) b.title = title;
  b.addEventListener('click', onClick);
  return b;
}

/** A labelled row holding a control (segmented buttons, a select…). */
export function row(label: string, control: HTMLElement): HTMLElement {
  const el = document.createElement('div');
  el.className = 'param row';
  const name = document.createElement('span');
  name.className = 'param-name small';
  name.textContent = label;
  el.append(name, control);
  return el;
}

export function card(title: string, ...children: HTMLElement[]): HTMLElement {
  const c = document.createElement('section');
  c.className = 'param-card';
  const h = document.createElement('h3');
  h.textContent = title;
  c.append(h, ...children);
  return c;
}

export function heading(title: string): HTMLElement {
  const h = document.createElement('h3');
  h.textContent = title;
  return h;
}

export function note(text: string, cls = 'prose hint-small'): HTMLElement {
  const p = document.createElement('p');
  p.className = cls;
  p.textContent = text;
  return p;
}

export function actions(...children: HTMLElement[]): HTMLElement {
  const el = document.createElement('div');
  el.className = 'tree-actions';
  el.append(...children);
  return el;
}

export const portLabel = (s: number) =>
  s === 0 ? 'origin' : s === 1 ? 'tip' : s === 0.5 ? 'middle' : `${s.toFixed(2)} l`;

export const DEG = 180 / Math.PI;

/** An element with a class and optional children. */
export function el<K extends keyof HTMLElementTagNameMap>(
  tag: K,
  cls = '',
  ...children: (Node | string)[]
): HTMLElementTagNameMap[K] {
  const e = document.createElement(tag);
  if (cls) e.className = cls;
  e.append(...children);
  return e;
}
