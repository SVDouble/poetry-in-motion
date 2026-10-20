/** Parameter sliders (linear or logarithmic) with KaTeX labels. */
import { fix, tex } from '../eq/tex';
import type { ParamSpec, UiParams } from '../systems/types';

const STEPS = 1000;

function toSlider(p: ParamSpec, v: number): number {
  if (p.log) return (Math.log(v / p.min) / Math.log(p.max / p.min)) * STEPS;
  return ((v - p.min) / (p.max - p.min)) * STEPS;
}

function fromSlider(p: ParamSpec, t: number): number {
  const f = t / STEPS;
  const v = p.log ? p.min * (p.max / p.min) ** f : p.min + f * (p.max - p.min);
  return p.step ? Math.round(v / p.step) * p.step : Number(v.toPrecision(4));
}

export function buildParams(host: HTMLElement, specs: ParamSpec[], ui: UiParams, onChange: () => void): void {
  host.replaceChildren();
  for (const [group, title] of [
    ['physical', 'Physical parameters'],
    ['control', 'Actuation & control'],
  ] as const) {
    const card = document.createElement('section');
    card.className = 'param-card';
    const h = document.createElement('h3');
    h.textContent = title;
    card.append(h);
    for (const p of specs.filter((s) => s.group === group)) {
      card.append(
        p.min === -1 && p.max === 1 && p.step === 2 ? toggle(p, ui, onChange) : slider(p, ui, onChange),
      );
    }
    host.append(card);
  }
  const reset = document.createElement('button');
  reset.className = 'btn ghost';
  reset.textContent = 'Restore defaults';
  reset.addEventListener('click', () => {
    for (const p of specs) ui[p.key] = p.value;
    buildParams(host, specs, ui, onChange);
    onChange();
  });
  host.append(reset);
}

function row(p: ParamSpec): { el: HTMLElement; out: HTMLElement } {
  const el = document.createElement('label');
  el.className = 'param';
  if (p.hint) el.title = p.hint;
  const name = document.createElement('span');
  name.className = 'param-name';
  tex(name, p.tex);
  const out = document.createElement('output');
  out.className = 'param-value';
  el.append(name, out);
  return { el, out };
}

function slider(p: ParamSpec, ui: UiParams, onChange: () => void): HTMLElement {
  const { el, out } = row(p);
  const input = document.createElement('input');
  input.type = 'range';
  input.min = '0';
  input.max = String(STEPS);
  input.step = '1';
  const show = () => {
    out.textContent = `${fix(ui[p.key] ?? p.value)} ${p.unit}`.trim();
  };
  input.value = String(toSlider(p, ui[p.key] ?? p.value));
  input.addEventListener('input', () => {
    ui[p.key] = fromSlider(p, Number(input.value));
    show();
    onChange();
  });
  show();
  el.append(input);
  return el;
}

function toggle(p: ParamSpec, ui: UiParams, onChange: () => void): HTMLElement {
  const { el, out } = row(p);
  out.remove();
  const seg = document.createElement('div');
  seg.className = 'segmented small';
  for (const [v, label] of [
    [1, 'up'],
    [-1, 'down'],
  ] as const) {
    const b = document.createElement('button');
    b.type = 'button';
    b.textContent = label;
    b.classList.toggle('on', (ui[p.key] ?? p.value) === v);
    b.addEventListener('click', () => {
      ui[p.key] = v;
      for (const x of seg.children) x.classList.toggle('on', x === b);
      onChange();
    });
    seg.append(b);
  }
  el.append(seg);
  return el;
}
