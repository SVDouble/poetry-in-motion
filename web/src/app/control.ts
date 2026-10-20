/**
 * The Control tab and the controller overlay. The tab compares every running slot live, lists the
 * controllers with their role (primary, shadow, off) and shows one page per controller: its parameters
 * first, then the idea, the derivation (generic and for this mechanism), stability and limitations.
 * The overlay keeps the primary's parameters on screen while simulating.
 */

import { pageContext, sections } from '../control/page';
import { FAMILIES, PAGES } from '../control/pages';
import { type ControlStore, SLOT_COLORS } from '../control/store';
import type { EquationPanel } from '../eq/panel';
import { fix } from '../eq/tex';
import type { Derivation } from '../model/derivation';
import { css } from '../scene/theme';
import type { ControllerSpec } from '../sim/controllers';
import type { Snapshot } from '../sim/layout';
import { button, el, type Slider, slider } from '../ui/widgets';

/** A slider for parameter i of controller `spec`, kept in sync with every other view of it. */
export function paramSlider(store: ControlStore, spec: ControllerSpec, i: number): Slider {
  const p = spec.params[i]!;
  const s: Slider = slider({
    label: p.tex,
    tex: true,
    min: p.min,
    max: p.max,
    log: p.log,
    unit: p.unit,
    value: store.param(spec.id, i),
    onInput: (v) => store.setParam(spec.id, i, v, set),
  });
  const set = (v: number) => (s.isConnected ? s.set(v) : unbind());
  const unbind = store.bind(spec.id, i, set);
  return s;
}

const swatch = (slot: number) => {
  const dot = el('span', 'swatch');
  dot.style.background = css(SLOT_COLORS[slot] ?? 0xffffff);
  return dot;
};

export class ControlView {
  /** The controller whose page is shown (−1: the primary). */
  private selected = -1;
  private strip: { row: HTMLElement; cells: HTMLElement[]; slot: number }[] = [];

  constructor(
    private readonly store: ControlStore,
    private readonly d: Derivation,
    private readonly snap: Snapshot,
    private readonly panel: EquationPanel,
  ) {}

  private get shown(): number {
    return this.selected < 0 ? this.store.primary : this.selected;
  }

  // ------------------------------------------------------------ tab

  /** The whole tab; re-render it when the slot assignment changes. */
  tab(): HTMLElement {
    const host = el('div', 'control-tab');
    host.append(this.comparison(), this.list());
    const page = el('div', 'ctrl-page');
    host.append(page);
    this.page(page);
    return host;
  }

  open(kind: number): void {
    this.selected = kind;
  }

  private comparison(): HTMLElement {
    const table = el('div', 'compare');
    const head = el('div', 'compare-row head');
    const tool = this.snap.v('USE_TOOL') !== 0;
    for (const h of ['', 'controller', tool ? '‖x − x_d‖' : '‖q − q*‖', '‖u‖', 'E [J]'])
      head.append(el('span', '', h));
    table.append(head);
    this.strip = [];
    this.store.slots.forEach((kind, slot) => {
      if (kind < 0) return;
      const spec = this.store.spec(kind);
      const name = el('span', 'name', spec?.name ?? '?');
      if (slot > 0) name.append(el('span', 'muted', ' shadow'));
      const cells = [el('span', 'num'), el('span', 'num'), el('span', 'num')];
      const row = el('div', 'compare-row', swatch(slot), name, ...cells);
      table.append(row);
      this.strip.push({ row, cells, slot });
    });
    const actions = el('div', 'compare-actions');
    const shadows = this.store.slots.some((k, i) => i > 0 && k >= 0);
    if (shadows) {
      actions.append(
        button(
          'Restart shadows from primary',
          () => this.store.sync(),
          'btn small',
          'copy the primary state into every shadow',
        ),
      );
    } else {
      actions.append(
        el('span', 'muted', 'Add shadows (◐) to compare controllers on the same mechanism, side by side.'),
      );
    }
    table.append(actions);
    return table;
  }

  private list(): HTMLElement {
    const host = el('div', 'ctrl-list');
    const byFamily = new Map<string, ControllerSpec[]>();
    for (const s of this.store.reg.specs) byFamily.set(s.family, [...(byFamily.get(s.family) ?? []), s]);
    for (const [family, specs] of byFamily) {
      const group = el('div', 'ctrl-family', el('h4', '', FAMILIES[family] ?? family));
      for (const s of specs) group.append(this.item(s));
      host.append(group);
    }
    return host;
  }

  private item(s: ControllerSpec): HTMLElement {
    const store = this.store;
    const why = store.reason(s.id);
    const slot = store.slotOf(s.id);
    const row = el('div', `ctrl-item${why ? ' disabled' : ''}${s.id === this.shown ? ' selected' : ''}`);
    const name = button(s.name, () => this.select(s.id), 'ctrl-name');
    if (why) name.title = `Not available: ${why}`;
    const primary = button(
      '●',
      () => store.setPrimary(s.id),
      `role${slot === 0 ? ' on' : ''}`,
      'run as the primary',
    );
    const shadow = button(
      '◐',
      () => store.toggleShadow(s.id),
      `role${slot > 0 ? ' on' : ''}`,
      'run as a shadow',
    );
    if (slot > 0) shadow.style.color = css(SLOT_COLORS[slot]!);
    primary.disabled = shadow.disabled = why !== null;
    shadow.disabled ||= slot === 0;
    row.append(name, primary, shadow);
    if (why) row.append(el('span', 'why', why));
    return row;
  }

  private select(kind: number): void {
    this.selected = kind;
    const tab = document.querySelector('.control-tab');
    tab?.replaceWith(this.tab());
  }

  private page(host: HTMLElement): void {
    const spec = this.store.spec(this.shown);
    const make = spec && PAGES[spec.key];
    if (!spec || !make) return;
    const ctx = pageContext(this.d, this.snap, spec, (i) => this.store.param(spec.id, i));
    const page = make(ctx);
    const head = el('div', 'ctrl-head', el('h3', '', spec.name), el('p', 'prose', page.lead));
    host.append(head);
    if (spec.params.length) {
      const card = el('section', 'param-card', el('h3', '', 'Parameters'));
      card.append(...spec.params.map((_, i) => paramSlider(this.store, spec, i)));
      host.append(card);
    }
    const why = this.store.reason(spec.id);
    if (why) host.append(el('p', 'prose warn', `Not available for this mechanism: ${why}.`));
    const body = el('div', 'ctrl-sections');
    host.append(body);
    this.panel.mount(sections(spec.key, page), body);
  }

  /** Refresh the live comparison numbers. */
  update(): void {
    const tool = this.snap.v('USE_TOOL') !== 0;
    for (const { cells, slot } of this.strip) {
      const v = this.snap.slots[slot]!;
      const values = [v.v(tool ? 'ERR_X' : 'ERR_Q'), v.v('EFFORT'), v.v('ENERGY', 2)];
      values.forEach((x, k) => {
        const text = fix(x);
        if (cells[k]!.textContent !== text) cells[k]!.textContent = text;
      });
    }
  }

  // ------------------------------------------------------------ overlay

  /** Primary chips, shadow legend and the primary's parameters, for the viewport overlay. */
  overlay(chips: HTMLElement, quick: HTMLElement, hint: HTMLElement): void {
    const store = this.store;
    chips.replaceChildren();
    store.usable().forEach((s, k) => {
      const b = button('', () => store.setPrimary(s.id), '');
      b.innerHTML = `<span class="num">${k + 1}</span>${s.name}`;
      const slot = store.slotOf(s.id);
      b.classList.toggle('on', slot === 0);
      b.setAttribute('aria-pressed', String(slot === 0));
      if (slot > 0) b.prepend(swatch(slot));
      b.title = slot > 0 ? 'running as a shadow' : 'run as the primary';
      chips.append(b);
    });
    const spec = store.spec(store.primary);
    quick.replaceChildren();
    if (spec) quick.append(...spec.params.map((_, i) => paramSlider(store, spec, i)));
    const make = spec && PAGES[spec.key];
    hint.textContent = make ? make(pageContext(this.d, this.snap, spec, () => Number.NaN)).lead : '';
  }
}
