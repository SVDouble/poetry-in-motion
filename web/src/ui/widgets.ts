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

export interface StepperOpts {
  label: string;
  tex?: boolean;
  value: number;
  step: number;
  min: number;
  max: number;
  unit?: string;
  /** Shown value = stored value × scale (e.g. degrees). */
  scale?: number;
  digits?: number;
  hint?: string;
  onChange: (v: number) => void;
}

/**
 * A number with ◀ ▶ arrows: click to step, hold to repeat (accelerating), Shift for a tenth of the step,
 * the wheel to scroll through values, or type a value.
 */
export function stepper(o: StepperOpts): HTMLElement {
  const scale = o.scale ?? 1;
  const digits = o.digits ?? Math.max(0, Math.ceil(-Math.log10(o.step * scale)) + 1);
  let value = o.value;
  const wrap = el('div', 'param stepper-row');
  if (o.hint) wrap.title = o.hint;
  const name = el('span', 'param-name');
  if (o.tex) tex(name, o.label);
  else name.textContent = o.label;
  const box = el('span', 'stepper');
  const input = el('input', 'stepper-value');
  input.type = 'text';
  input.inputMode = 'decimal';
  const unit = el('span', 'stepper-unit', o.unit ?? '');
  const show = () => {
    input.value = (value * scale).toFixed(digits);
  };
  const set = (v: number) => {
    const next = Math.min(o.max, Math.max(o.min, Number(v.toFixed(10))));
    if (next === value) return;
    value = next;
    show();
    o.onChange(value);
  };
  const nudge = (dir: number, fine: boolean) => set(value + dir * o.step * (fine ? 0.1 : 1));
  const arrow = (label: string, dir: number) => {
    const b = el('button', 'stepper-arrow', label);
    b.type = 'button';
    let timer = 0;
    const stop = () => window.clearTimeout(timer);
    b.addEventListener('pointerdown', (e) => {
      e.preventDefault();
      nudge(dir, e.shiftKey);
      let delay = 380;
      const again = () => {
        if (!b.isConnected) return; // re-rendered meanwhile: stop repeating
        nudge(dir, e.shiftKey);
        delay = Math.max(40, delay * 0.8);
        timer = window.setTimeout(again, delay);
      };
      timer = window.setTimeout(again, delay);
    });
    for (const ev of ['pointerup', 'pointerleave', 'pointercancel'] as const) b.addEventListener(ev, stop);
    return b;
  };
  input.addEventListener('change', () => {
    const v = Number(input.value.replace(',', '.'));
    if (Number.isFinite(v)) set(v / scale);
    else show();
  });
  input.addEventListener('keydown', (e) => {
    if (e.key === 'ArrowUp' || e.key === 'ArrowDown') {
      e.preventDefault();
      nudge(e.key === 'ArrowUp' ? 1 : -1, e.shiftKey);
    }
  });
  box.addEventListener(
    'wheel',
    (e) => {
      e.preventDefault();
      nudge(e.deltaY < 0 ? 1 : -1, e.shiftKey);
    },
    { passive: false },
  );
  show();
  box.append(arrow('◀', -1), input, unit, arrow('▶', 1));
  wrap.append(name, box);
  return wrap;
}
