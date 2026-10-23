import {
  CATEGORIES,
  type Category,
  category,
  cloneMechanism,
  emptyMechanism,
  groundPoints,
  MAX_CLOSURES,
  type Mechanism,
  PRESETS,
  removeBody,
} from '../model/description';
import { SNAP_ANGLE } from '../scene/geometry';
import { initialFrames, poseBounds } from '../scene/kinematics';
import { drawThumbnail } from '../scene/schematic';
import { isDark } from '../scene/theme';
import { actions, button, card, DEG, heading, note, portLabel, slider } from '../ui/widgets';
import { connectionGraph, highlightGraph } from './graph';
import { bodyControls, elementControls, jointControls, type Selection } from './inspector';
import type { PartKind } from './parts';

export interface EditorEvents {
  change(m: Mechanism, discrete?: boolean): void;
  inspect(kind: Selection['kind'], index: number): void;
  select(i: number): void;
  /** Start a loop closure from the given port (the viewport then asks for the other end). */
  connect(body: number, s: number, kind: 'pin' | 'slot'): void;
  flash(message: string): void;
  /** A spring/damper was selected in the list (−1: none). */
  selectElement(k: number): void;
  /** Current length of spring/damper k. */
  elementLength(k: number): number;
  /** Pick up a part, as if from the palette. */
  startPart(kind: PartKind): void;
}

/** Gallery shelves of the presets. */
const THUMB: [number, number] = [104, 62];
const thumbs = new Map<string, HTMLCanvasElement>();

/** The preset drawn at its initial pose (cached per theme; a canvas can only sit in one place). */
function thumbnail(name: string): HTMLCanvasElement {
  const key = `${name}|${isDark()}`;
  const cached = thumbs.get(key);
  if (cached) return cached;
  const mech = PRESETS[name]!();
  const canvas = document.createElement('canvas');
  drawThumbnail(canvas, mech, initialFrames(mech), poseBounds(mech), THUMB);
  thumbs.set(key, canvas);
  return canvas;
}

export class Editor {
  private mech!: Mechanism;
  private selected = 0;
  private selectedElement = -1;
  private graphSelection: Selection | null = { kind: 'body', index: 0 };

  constructor(
    readonly host: HTMLElement,
    private readonly events: EditorEvents,
  ) {
    host.className = 'editor';
  }

  highlight(sel: Selection): void {
    this.graphSelection = sel;
    this.events.select(sel.kind === 'body' || sel.kind === 'joint' ? sel.index : -1);
    this.events.selectElement(sel.kind === 'element' ? sel.index : -1);
    highlightGraph(this.host, sel);
    const graph = this.host.querySelector<HTMLElement>('.connection-graph');
    const node = graph?.querySelector<HTMLElement>('.graph-node.on');
    if (graph && node) {
      const bounds = graph.getBoundingClientRect();
      const target = node.getBoundingClientRect();
      if (target.top < bounds.top) graph.scrollTop -= bounds.top - target.top + 8;
      if (target.bottom > bounds.bottom) graph.scrollTop += target.bottom - bounds.bottom + 8;
      if (target.left < bounds.left) graph.scrollLeft -= bounds.left - target.left + 8;
      if (target.right > bounds.right) graph.scrollLeft += target.right - bounds.right + 8;
    }
  }

  get activeSelection(): Selection | null {
    return this.graphSelection;
  }

  get mechanism(): Mechanism {
    return this.mech;
  }

  get selection(): number {
    return this.selected;
  }

  selectElement(k: number): void {
    const next = k >= 0 && k < this.mech.elements.length ? k : -1;
    if (next !== this.selectedElement) {
      this.selectedElement = next;
      this.render();
    }
    if (next >= 0) this.highlight({ kind: 'element', index: next });
    else this.events.selectElement(next);
  }

  load(mech: Mechanism, selected = this.selected): void {
    this.mech = cloneMechanism(mech);
    this.selected = mech.bodies.length ? Math.max(0, Math.min(selected, mech.bodies.length - 1)) : -1;
    if (this.selectedElement >= mech.elements.length) this.selectedElement = -1;
    this.graphSelection = this.selected < 0 ? null : { kind: 'body', index: this.selected };
    this.render();
  }

  select(i: number): void {
    if (i < 0 || i >= this.mech.bodies.length) return;
    if (i !== this.selected) {
      this.selected = i;
      this.render();
    }
    this.highlight({ kind: 'body', index: i });
  }

  /** Apply a new document (re-rendering the panel) and report it. */
  commit(m: Mechanism, selected = this.selected, discrete = true): void {
    this.mech = m;
    this.selected = m.bodies.length ? Math.max(0, Math.min(selected, m.bodies.length - 1)) : -1;
    if (this.selectedElement >= m.elements.length) this.selectedElement = -1;
    this.graphSelection = this.selected < 0 ? null : { kind: 'body', index: this.selected };
    this.render();
    this.events.change(cloneMechanism(m), discrete);
    this.events.select(this.selected);
  }

  private refuse(message: string): void {
    this.events.flash(message);
  }

  remove(i: number): void {
    if (i < 0 || i >= this.mech.bodies.length) return;
    const m = removeBody(this.mech, i);
    this.commit({ ...m, name: 'Custom' }, Math.max(0, Math.min(i - 1, m.bodies.length - 1)));
  }

  private render(): void {
    const focused =
      document.activeElement instanceof HTMLElement && this.host.contains(document.activeElement)
        ? document.activeElement
        : null;
    const focusKind = focused?.dataset.graphKind;
    const focusIndex = focused?.dataset.graphIndex;
    const previous = this.host.querySelector('.connection-graph');
    const offset = [previous?.scrollLeft ?? 0, previous?.scrollTop ?? 0];
    this.host.replaceChildren(
      this.graphCard(),
      this.presetsCard(),
      this.loopsCard(),
      this.springsCard(),
      ...(this.mech.bodies[this.selected] ? [this.bodyCard(this.selected)] : []),
    );
    const graph = this.host.querySelector('.connection-graph');
    if (graph) {
      graph.scrollLeft = offset[0]!;
      graph.scrollTop = offset[1]!;
    }
    highlightGraph(this.host, this.graphSelection);
    if (focusKind)
      Array.from(this.host.querySelectorAll<HTMLElement>('[data-graph-kind]'))
        .find((node) => node.dataset.graphKind === focusKind && node.dataset.graphIndex === focusIndex)
        ?.focus({ preventScroll: true });
  }

  private presetsCard(): HTMLElement {
    const grid = document.createElement('div');
    grid.className = 'preset-gallery';
    // a blank sheet first: build from scratch
    const blank = button(
      '',
      () => this.commit({ ...emptyMechanism(), g: this.mech.g }, -1),
      `preset-card${this.mech.bodies.length === 0 ? ' on' : ''}`,
    );
    blank.title = 'Empty scene: only the ground — drag parts in from the palette';
    const sheet = document.createElement('div');
    sheet.className = 'preset-blank';
    const blankLabel = document.createElement('span');
    blankLabel.textContent = 'Empty scene';
    blank.append(sheet, blankLabel);
    const start = document.createElement('h4');
    start.textContent = 'Blank';
    const startRow = document.createElement('div');
    startRow.className = 'preset-row';
    startRow.append(blank);
    grid.append(start, startRow);
    const shelves = new Map<Category, string[]>(CATEGORIES.map((c) => [c, []]));
    for (const [name, make] of Object.entries(PRESETS)) shelves.get(category(make()))!.push(name);
    for (const [group, names] of shelves) {
      if (!names.length) continue;
      const row = document.createElement('div');
      row.className = 'preset-row';
      for (const name of names) {
        const b = button(
          '',
          () => this.commit(PRESETS[name]!(), 0),
          `preset-card${name === this.mech.name ? ' on' : ''}`,
        );
        b.title = name;
        const label = document.createElement('span');
        label.textContent = name;
        b.append(thumbnail(name), label);
        row.append(b);
      }
      const h = document.createElement('h4');
      h.textContent = group;
      grid.append(h, row);
    }
    const g = slider({
      label: 'g',
      tex: true,
      min: 0,
      max: 20,
      value: this.mech.g,
      unit: 'm/s²',
      hint: 'gravity (along −y)',
      onInput: (v) => {
        this.mech.g = v;
        this.events.change(cloneMechanism(this.mech));
      },
    });
    return card('Start from', grid, g);
  }

  private graphCard(): HTMLElement {
    const anchors = groundPoints(this.mech).map(([x, y], i) => {
      const edit = button(
        `Ground point ${i + 1}: (${x.toFixed(2)}, ${y.toFixed(2)}) m`,
        () => this.events.inspect('ground', i),
        'btn small',
        'Edit this fixed attachment point',
      );
      edit.dataset.graphKind = 'ground';
      edit.dataset.graphIndex = String(i);
      return edit;
    });
    return card(
      'Connection graph',
      connectionGraph(this.mech, this.graphSelection, (kind, i) => {
        if (kind === 'ground' && !groundPoints(this.mech).length) this.events.startPart('ground');
        else this.events.inspect(kind, i);
      }),
      actions(
        button('+ Ground', () => this.events.startPart('ground')),
        button('+ Bar', () => this.events.startPart('bar')),
        button('+ Block', () => this.events.startPart('block')),
      ),
      note(
        'Bodies are rectangles; joints sit on the connections. Click either to edit. Dashed connections are loop constraints or springs.',
      ),
      note(
        'Place bodies and fixed Ground points first. Choose a rotational, translational or fixed joint, click a ground point or a point along a body, then click an unconnected body.',
      ),
      note(
        'Ground is the fixed world frame (0 DOF). Its points are attachment locations, not a collision floor.',
      ),
      actions(...anchors),
    );
  }

  private loopsCard(): HTMLElement {
    const c = card('Loops');
    if (this.mech.closures.length === 0) {
      c.append(
        note(
          'Close a kinematic loop by pinning a point of one body to another body or the ground (2 constraints, e.g. a four-bar), or by letting it slide in a slot (1 constraint, e.g. a slider-crank).',
        ),
      );
    }
    this.mech.closures.forEach((cl, k) => {
      const item = document.createElement('div');
      item.className = 'tree-item static';
      const a =
        cl.a < 0
          ? `ground (${cl.gx.toFixed(2)}, ${cl.gy.toFixed(2)})`
          : `body ${cl.a + 1} @ ${portLabel(cl.sa)}`;
      const what = cl.kind === 'pin' ? 'pinned to' : 'slides in a slot on';
      item.innerHTML = `<span class="joint-icon ${cl.kind}"></span><span class="tree-label">body ${cl.b + 1} @ ${portLabel(cl.sb)} <span class="muted">${what}</span> ${a}</span>`;
      const edit = (patch: Partial<typeof cl>) => {
        const m = cloneMechanism(this.mech);
        m.closures[k] = { ...cl, ...patch };
        return m;
      };
      item.append(
        button(
          '×',
          () => {
            const m = cloneMechanism(this.mech);
            m.closures.splice(k, 1);
            this.commit(m);
          },
          'icon-btn',
          'remove this closure',
        ),
      );
      c.append(item);
      if (cl.kind === 'slot') {
        c.append(
          slider({
            label: 'slot direction',
            min: -180,
            max: 180,
            step: SNAP_ANGLE * DEG,
            value: cl.angle * DEG,
            unit: '°',
            format: (v) => v.toFixed(0),
            hint: `direction of the slot in the frame of ${cl.a < 0 ? 'the ground' : `body ${cl.a + 1}`}`,
            onInput: (v) => {
              this.mech = edit({ angle: v / DEG });
              this.events.change(cloneMechanism(this.mech));
            },
          }),
        );
      }
    });
    const full = this.mech.closures.length >= MAX_CLOSURES;
    const start = (kind: 'pin' | 'slot', label: string) => {
      const b = button(label, () => this.events.connect(this.selected, 1, kind));
      b.disabled = full || this.selected < 0;
      return b;
    };
    c.append(
      actions(
        start('pin', `+ pin tip of body ${this.selected + 1} to…`),
        start('slot', `+ slide tip of body ${this.selected + 1} in a slot…`),
      ),
    );
    return c;
  }

  private springsCard(): HTMLElement {
    const c = card('Springs & dampers');
    if (this.mech.elements.length === 0) {
      c.append(
        note(
          'Join two points — on bodies or the ground — with a spring, a damper or both: drag one from the parts palette in the view, drop one end, then click the other.',
        ),
      );
    }
    this.mech.elements.forEach((e, k) => {
      const item = document.createElement('div');
      item.className = `tree-item${k === this.selectedElement ? ' on' : ''}`;
      const kind = e.k > 0 && e.d > 0 ? 'spring–damper' : e.k > 0 ? 'spring' : 'damper';
      const a =
        e.a < 0 ? `ground (${e.gx.toFixed(2)}, ${e.gy.toFixed(2)})` : `body ${e.a + 1} @ ${portLabel(e.sa)}`;
      item.innerHTML = `<span class="joint-icon spring"></span><span class="tree-label">${kind} ${k + 1} <span class="muted">${a} ↔ body ${e.b + 1} @ ${portLabel(e.sb)}</span></span>`;
      item.addEventListener('click', () => this.selectElement(k === this.selectedElement ? -1 : k));
      const rm = button(
        '×',
        () => {
          const m = cloneMechanism(this.mech);
          m.elements.splice(k, 1);
          this.selectedElement = -1;
          this.commit({ ...m, name: 'Custom' });
        },
        'icon-btn',
        'remove',
      );
      rm.addEventListener('click', (ev) => ev.stopPropagation());
      item.append(rm);
      c.append(item);
      if (k === this.selectedElement) {
        const editor = this;
        const ctx = {
          get mech() {
            return editor.mech;
          },
          commit: (m: Mechanism, structural = false) => {
            this.mech = m;
            if (structural) this.render();
            this.events.change(cloneMechanism(m), structural);
          },
          refuse: (msg: string) => this.refuse(msg),
          length: (j: number) => this.events.elementLength(j),
        };
        const box = document.createElement('div');
        box.className = 'element-fields';
        box.append(...elementControls(ctx, k));
        c.append(box);
      }
    });
    c.append(
      actions(
        button('+ spring', () => this.events.startPart('spring')),
        button('+ damper', () => this.events.startPart('damper')),
        button('+ spring–damper', () => this.events.startPart('springDamper')),
      ),
    );
    return c;
  }

  private bodyCard(i: number): HTMLElement {
    const editor = this;
    const ctx = {
      get mech() {
        return editor.mech;
      },
      commit: (m: Mechanism, structural = false) => {
        this.mech = m;
        if (structural) this.render();
        this.events.change(cloneMechanism(m), structural);
      },
      refuse: (msg: string) => this.refuse(msg),
      length: (k: number) => this.events.elementLength(k),
    };
    const c = card(`Body ${i + 1}`, ...bodyControls(ctx, i));
    if (!this.mech.bodies[i]!.detached) c.append(heading(`Joint ${i + 1}`), ...jointControls(ctx, i));
    else c.append(note('Unconnected draft body: add a joint to attach it.'));
    return c;
  }
}
