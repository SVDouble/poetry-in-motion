/**
 * The controller UI: one menu for picking controllers (used everywhere), the dock on the viewport (primary,
 * shadows, the primary's parameters) and the Control tab (live comparison, one page per controller with
 * its parameters, guided experiments and the theory).
 */
import { LABS } from '../control/lab';
import { type Experiment, pageContext, sections } from '../control/page';
import { FAMILIES, PAGES, shortName } from '../control/pages';
import { type ControlStore, OFF, SLOT_COLORS } from '../control/store';
import { type EquationPanel, prose } from '../eq/panel';
import { fix } from '../eq/tex';
import type { Derivation } from '../model/derivation';
import { css } from '../scene/theme';
import type { ControllerSpec } from '../sim/controllers';
import type { Snapshot } from '../sim/layout';
import { closePopover, popover } from '../ui/popover';
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

export const swatch = (slot: number) => {
  const dot = el('span', 'swatch');
  dot.style.background = css(SLOT_COLORS[slot] ?? 0x16181d);
  return dot;
};

type MenuMode = 'primary' | 'shadow' | 'page';

/** Every controller grouped by family. Unavailable ones are shown with the reason (except for pages). */
export function controllerMenu(
  store: ControlStore,
  mode: MenuMode,
  pick: (kind: number) => void,
  current = -1,
): HTMLElement {
  const host = el('div', 'ctrl-menu');
  const families = new Map<string, ControllerSpec[]>();
  for (const s of store.reg.specs) families.set(s.family, [...(families.get(s.family) ?? []), s]);
  for (const [family, specs] of families) {
    host.append(el('h5', '', FAMILIES[family] ?? family));
    for (const s of specs) {
      const why = store.reason(s.id);
      const slot = store.slotOf(s.id);
      const item = button(
        '',
        () => {
          closePopover();
          pick(s.id);
        },
        'ctrl-menu-item',
      );
      item.append(slot >= 0 ? swatch(slot) : el('span', 'swatch empty'), el('span', 'name', s.name));
      if (mode === 'shadow' && slot > 0) item.append(el('span', 'tag', 'remove'));
      else if (slot === 0) item.append(el('span', 'tag', 'primary'));
      else if (slot > 0) item.append(el('span', 'tag', 'shadow'));
      if (why) item.append(el('span', 'why', why));
      item.disabled = mode !== 'page' && (why !== null || (mode === 'shadow' && slot === 0));
      item.classList.toggle('on', s.id === current);
      host.append(item);
    }
  }
  return host;
}

// ---------------------------------------------------------------- dock

/** Primary controller, shadow pills and the primary's parameters, floating on the viewport. */
export function dock(store: ControlStore, collapsed: boolean, onCollapse: (c: boolean) => void): HTMLElement {
  const host = el('div', 'dock');
  const spec = store.spec(store.primary);
  const head = el('div', 'dock-head');
  const primary = button(
    '',
    () =>
      popover(
        primary,
        controllerMenu(store, 'primary', (k) => store.setPrimary(k), store.primary),
      ),
    'dock-primary',
  );
  primary.append(swatch(0), el('span', '', spec?.name ?? '—'), el('span', 'caret', '▾'));
  primary.title = 'choose the primary controller (1–9)';
  head.append(primary);
  store.slots.forEach((kind, slot) => {
    if (slot === 0 || kind === OFF) return;
    const pill = el('span', 'pill', swatch(slot), el('span', '', shortName(store.spec(kind))));
    pill.title = store.spec(kind)?.name ?? '';
    pill.append(button('×', () => store.toggleShadow(kind), 'x', 'stop this shadow'));
    head.append(pill);
  });
  if (store.slots.includes(OFF)) {
    const add = button(
      '+ shadow',
      () =>
        popover(
          add,
          controllerMenu(store, 'shadow', (k) => store.toggleShadow(k)),
        ),
      'dock-add',
    );
    add.title = 'run another controller alongside, on its own copy of the mechanism (Shift+1–9)';
    head.append(add);
  }
  host.append(head);
  if (spec?.params.length) {
    const fold = button(
      collapsed ? '▸' : '▾',
      () => onCollapse(!collapsed),
      'dock-fold',
      collapsed ? 'show parameters' : 'hide parameters',
    );
    head.append(fold);
    if (!collapsed) {
      host.append(el('div', 'dock-params', ...spec.params.map((_, i) => paramSlider(store, spec, i))));
    }
  }
  return host;
}

// ---------------------------------------------------------------- Control tab

export interface ControlTabHost {
  store: ControlStore;
  d: Derivation;
  snap: Snapshot;
  panel: EquationPanel;
  run(kind: number, e: Experiment): void;
  watch(kind: number): void;
}

export class ControlTab {
  /** The controller whose page is shown (−1: follow the primary). */
  private selected = -1;
  private strip: { cells: HTMLElement[]; slot: number }[] = [];
  private host: HTMLElement | null = null;
  private notice: { kind: number; title: string } | null = null;

  constructor(private readonly h: ControlTabHost) {}

  private get shown(): number {
    return this.selected < 0 ? this.h.store.primary : this.selected;
  }

  /** Build the tab (replacing the previous rendering in place, if it is mounted). */
  render(): HTMLElement {
    const host = el('div', 'control-tab');
    host.append(this.comparison(), this.page());
    if (this.host?.isConnected) this.host.replaceWith(host);
    this.host = host;
    return host;
  }

  /** An experiment was set up: show its page with what to notice. */
  ran(kind: number, title: string): void {
    this.notice = { kind, title };
    this.selected = kind;
    this.render();
    this.host?.querySelector('.lab-item.open')?.scrollIntoView({ block: 'nearest', behavior: 'smooth' });
  }

  private comparison(): HTMLElement {
    const { store, snap } = this.h;
    const box = el('div', 'compare');
    const tool = snap.v('USE_TOOL') !== 0;
    const head = el('div', 'compare-row head');
    for (const t of ['', 'running', tool ? '‖x − x_d‖' : '‖q − q*‖', '‖u‖', 'E [J]', ''])
      head.append(el('span', '', t));
    box.append(head);
    this.strip = [];
    store.slots.forEach((kind, slot) => {
      if (kind < 0) return;
      const spec = store.spec(kind);
      const name = button(spec?.name ?? '?', () => this.open(kind), 'link', 'show its page');
      const cells = [el('span', 'num'), el('span', 'num'), el('span', 'num')];
      const act =
        slot === 0
          ? el('span', 'muted small', 'primary')
          : button('×', () => store.toggleShadow(kind), 'x', 'stop this shadow');
      box.append(el('div', 'compare-row', swatch(slot), name, ...cells, act));
      this.strip.push({ cells, slot });
    });
    const actions = el('div', 'compare-actions');
    if (store.slots.includes(OFF)) {
      const add = button(
        '+ shadow',
        () =>
          popover(
            add,
            controllerMenu(store, 'shadow', (k) => store.toggleShadow(k)),
          ),
        'btn small',
      );
      actions.append(add);
    }
    if (store.slots.some((k, i) => i > 0 && k >= 0)) {
      actions.append(
        button(
          'Restart shadows from primary',
          () => store.sync(),
          'btn small',
          'copy the primary state into every shadow',
        ),
      );
    } else {
      actions.append(
        el(
          'span',
          'muted small',
          'Shadows run other controllers on their own copy of the mechanism, under the same pushes.',
        ),
      );
    }
    box.append(actions);
    return box;
  }

  private open(kind: number): void {
    this.selected = kind;
    this.notice = null;
    this.render();
  }

  private page(): HTMLElement {
    const { store, d, snap, panel } = this.h;
    const host = el('div', 'ctrl-page');
    const spec = store.spec(this.shown);
    const make = spec && PAGES[spec.key];
    if (!spec || !make) return host;
    const ctx = pageContext(d, snap, spec, (i) => store.param(spec.id, i));
    const page = make(ctx);
    const lab = LABS[spec.key]?.(ctx);
    const why = store.reason(spec.id);
    const slot = store.slotOf(spec.id);

    const picker = button(
      '',
      () =>
        popover(
          picker,
          controllerMenu(store, 'page', (k) => this.open(k), spec.id),
        ),
      'page-picker',
    );
    picker.append(el('span', '', spec.name), el('span', 'caret', '▾'));
    picker.title = 'explain another controller';
    const role =
      slot === 0
        ? el('span', 'role-tag', swatch(0), 'primary')
        : why
          ? el('span', 'role-tag muted', 'not available')
          : el(
              'span',
              'role-actions',
              button('Make primary', () => store.setPrimary(spec.id), 'btn small'),
              ...(slot > 0 ? [] : [button('+ shadow', () => store.toggleShadow(spec.id), 'btn small')]),
            );
    host.append(el('div', 'ctrl-head', picker, role), prose(page.lead, 'prose lead'));
    if (why) host.append(el('p', 'prose warn', `Not available for this mechanism: ${why}.`));

    if (spec.params.length) {
      const card = el('section', 'param-card', el('h3', '', 'Parameters'));
      card.append(...spec.params.map((_, i) => paramSlider(store, spec, i)));
      card.append(
        el(
          'div',
          'card-actions',
          button('Defaults', () => store.resetParams(spec.id), 'btn small'),
        ),
      );
      host.append(card);
    }

    if (lab && !why && (lab.experiments.length || lab.watch.length)) {
      const card = el('section', 'param-card lab', el('h3', '', 'Try it'));
      if (lab.watch.length) {
        card.append(
          el(
            'div',
            'lab-row',
            el('span', 'prose small', 'Plot the signals that matter for this controller.'),
            button('Watch', () => this.h.watch(spec.id), 'btn small'),
          ),
        );
      }
      for (const e of lab.experiments) {
        const open = this.notice?.kind === spec.id && this.notice.title === e.title;
        const item = el('div', `lab-item${open ? ' open' : ''}`);
        item.append(
          el(
            'div',
            'lab-row',
            el('div', '', el('b', '', e.title), el('span', 'prose small', e.goal)),
            button('Run ▶', () => this.h.run(spec.id, e), 'btn small primary'),
          ),
        );
        if (open)
          item.append(el('div', 'lab-notice', el('span', 'muted small', 'What to notice'), prose(e.notice)));
        card.append(item);
      }
      card.append(el('p', 'muted small', 'Experiments start from default parameters, limits and goals.'));
      host.append(card);
    }

    const body = el('div', 'ctrl-sections');
    host.append(body);
    panel.mount(sections(spec.key, page), body);
    return host;
  }

  /** Refresh the live comparison numbers. */
  update(): void {
    const snap = this.h.snap;
    const tool = snap.v('USE_TOOL') !== 0;
    for (const { cells, slot } of this.strip) {
      const v = snap.slots[slot]!;
      [v.v(tool ? 'ERR_X' : 'ERR_Q'), v.v('EFFORT'), v.v('ENERGY', 2)].forEach((x, k) => {
        const text = fix(x);
        if (cells[k]!.textContent !== text) cells[k]!.textContent = text;
      });
    }
  }
}
