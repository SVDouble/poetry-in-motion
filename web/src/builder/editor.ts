/**
 * Structure editor: the mechanism graph as a tree of bodies (drag to re-hang, × to remove), loop closures
 * (pins and slots), and the settings of the selected joint and body. Every edit produces a new document; the
 * app re-derives it immediately and keeps an undo history.
 */
import { tex } from '../eq/tex';
import {
  type BodySpec,
  CATEGORIES,
  type Category,
  category,
  cloneMechanism,
  dof,
  emptyMechanism,
  isDescendant,
  type JointKind,
  link,
  MAX_BODIES,
  MAX_CLOSURES,
  MAX_DOF,
  type Mechanism,
  normalise,
  PRESETS,
  removeBody,
  reparent,
} from '../model/description';
import { initialFrames, poseBounds } from '../scene/kinematics';
import { drawThumbnail } from '../scene/schematic';
import { isDark } from '../scene/theme';
import { actions, button, card, DEG, heading, note, portLabel, row, segmented, slider } from '../ui/widgets';
import { elementControls } from './inspector';
import type { PartKind } from './parts';

export interface EditorEvents {
  change(m: Mechanism): void;
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

/** A numeric body/joint property bound to a slider. `scale` converts stored values to displayed ones. */
interface Field {
  key: keyof BodySpec;
  label: string;
  tex?: boolean;
  min: number;
  max: number;
  step?: number;
  unit?: string;
  log?: boolean;
  scale?: number;
  hint?: string;
  rerender?: boolean;
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
  private dragFrom = -1;

  constructor(
    readonly host: HTMLElement,
    private readonly events: EditorEvents,
  ) {
    host.className = 'editor';
  }

  get mechanism(): Mechanism {
    return this.mech;
  }

  get selection(): number {
    return this.selected;
  }

  get elementSelection(): number {
    return this.selectedElement;
  }

  selectElement(k: number): void {
    const next = k >= 0 && k < this.mech.elements.length ? k : -1;
    if (next !== this.selectedElement) {
      this.selectedElement = next;
      this.render();
    }
    this.events.selectElement(next);
  }

  load(mech: Mechanism, selected = this.selected): void {
    this.mech = cloneMechanism(mech);
    this.selected = Math.max(0, Math.min(selected, mech.bodies.length - 1));
    if (this.selectedElement >= mech.elements.length) this.selectedElement = -1;
    this.render();
  }

  select(i: number): void {
    if (i < 0 || i >= this.mech.bodies.length) return;
    if (i !== this.selected) {
      this.selected = i;
      this.render();
    }
    this.events.select(i);
  }

  /** Apply a new document (re-rendering the panel) and report it. */
  commit(m: Mechanism, selected = this.selected): void {
    this.mech = m;
    this.selected = m.bodies.length ? Math.max(0, Math.min(selected, m.bodies.length - 1)) : -1;
    if (this.selectedElement >= m.elements.length) this.selectedElement = -1;
    this.render();
    this.events.change(cloneMechanism(m));
    this.events.select(this.selected);
  }

  /** Small edits (sliders): no re-render, so the slider keeps the pointer. */
  private update(i: number, patch: Partial<BodySpec>, rerender = false): void {
    this.mech.bodies[i] = normalise({ ...this.mech.bodies[i]!, ...patch });
    if (rerender) this.render();
    this.events.change(cloneMechanism(this.mech));
  }

  private refuse(message: string): void {
    this.events.flash(message);
    this.render();
  }

  addChild(parent: number, joint: JointKind, s = 1): void {
    const m = cloneMechanism(this.mech);
    if (m.bodies.length >= MAX_BODIES) {
      this.refuse(`at most ${MAX_BODIES} bodies`);
      return;
    }
    if (joint !== 'fixed' && dof(m) >= MAX_DOF) {
      this.refuse(`at most ${MAX_DOF} moving joints`);
      return;
    }
    const siblings = m.bodies.filter((b) => b.parent === parent && Math.abs(b.mountS - s) < 1e-9).length;
    m.bodies.push(
      link(parent, joint, {
        mountS: parent >= 0 ? s : 0,
        mountX: parent < 0 ? 1 : 0,
        l: joint === 'prismatic' ? 0.5 : 0.8,
        m: 0.5,
        limit: joint === 'prismatic' ? 1 : 0,
        // continue straight from a tip, stand perpendicular elsewhere; fan siblings out so they do not overlap
        mountAngle:
          (parent >= 0 && s < 1 ? Math.PI / 2 : 0) + siblings * (Math.PI / 4) * (siblings % 2 ? 1 : -1),
      }),
    );
    if (m.tool < 0 || m.tool === parent) m.tool = m.bodies.length - 1;
    this.commit({ ...m, name: 'Custom' }, m.bodies.length - 1);
  }

  remove(i: number): void {
    if (this.mech.bodies.length <= 1) {
      // the last body: back to an empty scene (its springs and loops go with it)
      this.commit({ ...emptyMechanism(), g: this.mech.g }, -1);
      return;
    }
    const m = removeBody(this.mech, i);
    if (dof(m) === 0) {
      this.refuse('that would leave nothing that can move');
      return;
    }
    this.commit({ ...m, name: 'Custom' }, Math.max(0, Math.min(i - 1, m.bodies.length - 1)));
  }

  private render(): void {
    this.host.replaceChildren(
      this.presetsCard(),
      this.treeCard(),
      this.loopsCard(),
      this.springsCard(),
      ...(this.mech.bodies[this.selected] ? [this.bodyCard(this.selected)] : []),
    );
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

  private dropTarget(el: HTMLElement, parent: number): void {
    const ok = () =>
      this.dragFrom !== parent && (parent < 0 || !isDescendant(this.mech, parent, this.dragFrom));
    el.addEventListener('dragover', (e) => {
      if (this.dragFrom < 0) return;
      if (ok()) e.preventDefault();
      el.classList.toggle('drop-ok', ok());
      el.classList.toggle('drop-bad', !ok());
    });
    el.addEventListener('dragleave', () => el.classList.remove('drop-ok', 'drop-bad'));
    el.addEventListener('drop', (e) => {
      e.preventDefault();
      el.classList.remove('drop-ok', 'drop-bad');
      const from = this.dragFrom;
      this.dragFrom = -1;
      if (from < 0 || from === parent) return;
      const res = reparent(this.mech, from, parent);
      if (!res) {
        this.refuse('a body cannot hang from its own descendant — close a loop instead');
        return;
      }
      this.commit(res.mech, res.index);
    });
  }

  private treeCard(): HTMLElement {
    const list = document.createElement('div');
    list.className = 'tree';
    const ground = document.createElement('div');
    ground.className = 'tree-ground';
    ground.innerHTML =
      '<span class="ground-icon"></span>ground <span class="muted">— drop a body here to hang it from the ground</span>';
    this.dropTarget(ground, -1);
    list.append(ground);

    const depth = (i: number): number => {
      const p = this.mech.bodies[i]!.parent;
      return p < 0 ? 0 : depth(p) + 1;
    };
    this.mech.bodies.forEach((b, i) => {
      const item = document.createElement('div');
      item.className = `tree-item${i === this.selected ? ' on' : ''}`;
      item.style.paddingLeft = `${10 + 16 * depth(i)}px`;
      item.draggable = true;
      item.tabIndex = 0;
      item.title = 'click to select · drag onto another body (or the ground row) to re-hang it';
      item.addEventListener('dragstart', (e) => {
        this.dragFrom = i;
        e.dataTransfer?.setData('text/plain', String(i));
        item.classList.add('dragging');
      });
      item.addEventListener('dragend', () => item.classList.remove('dragging'));
      this.dropTarget(item, i);
      item.addEventListener('click', () => this.select(i));
      item.addEventListener('keydown', (e) => e.key === 'Enter' && this.select(i));

      const icon = document.createElement('span');
      icon.className = `joint-icon ${b.joint}${b.actuated || b.joint === 'fixed' ? '' : ' passive'}`;
      const label = document.createElement('span');
      label.className = 'tree-label';
      const details = [
        b.joint,
        b.joint !== 'fixed' && !b.actuated ? 'passive' : '',
        b.parent >= 0 ? `@ ${portLabel(b.mountS)}` : '',
        this.mech.tool === i ? 'tool' : '',
      ].filter(Boolean);
      label.innerHTML = `Body ${i + 1} <span class="muted">${details.join(', ')}</span>`;
      const q = document.createElement('span');
      q.className = 'tree-q';
      if (b.joint !== 'fixed') tex(q, b.joint === 'revolute' ? `\\theta_{${i + 1}}` : `d_{${i + 1}}`);
      const hasChildren = this.mech.bodies.some((x) => x.parent === i);
      const rm = button(
        '×',
        () => this.remove(i),
        'icon-btn',
        `remove body ${i + 1}${hasChildren ? ' and everything hanging from it' : ''}`,
      );
      rm.addEventListener('click', (e) => e.stopPropagation());
      item.append(icon, label, q, rm);
      list.append(item);
    });

    const parent = this.selected;
    const add = actions(
      ...(['revolute', 'prismatic', 'fixed'] as const).map((kind) =>
        button(
          `+ ${kind}`,
          () => this.addChild(parent, kind),
          'btn small',
          `attach at the tip of body ${parent + 1}`,
        ),
      ),
      button(
        '+ on ground',
        () => this.addChild(-1, 'revolute'),
        'btn small',
        'a new bar hinged to the ground',
      ),
    );
    return card(
      'Structure',
      list,
      add,
      note(
        'In the view: click a + port on the selected body to attach a child or close a loop there, drag ◆ to move its joint, drag ○ to rotate or stretch it (Shift: set its initial angle instead; Alt: no snapping). Delete removes, Ctrl+Z undoes.',
      ),
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
            step: 15,
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
      b.disabled = full;
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
        const ctx = {
          mech: this.mech,
          commit: (m: Mechanism, structural = false) => {
            this.mech = m;
            if (structural) this.render();
            this.events.change(cloneMechanism(m));
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

  /** The numeric settings of joint and body `i`, as a table. */
  private fields(b: BodySpec): Field[] {
    const angular = b.joint === 'revolute';
    const range = angular ? 180 : Math.max(b.limit, 1);
    const coord = (s: string) => (angular ? `\\theta_{${s}}` : `d_{${s}}`);
    const i = this.mech.bodies.indexOf(b) + 1;
    const qField = (key: 'q0' | 'qTarget', label: string, hint: string): Field => ({
      key,
      label,
      tex: true,
      min: -range,
      max: range,
      step: angular ? 1 : 0.01,
      unit: angular ? '°' : 'm',
      scale: angular ? DEG : 1,
      hint,
    });
    const joint: Field[] =
      b.parent < 0
        ? [
            { key: 'mountX', label: 'x₀', min: -3, max: 3, step: 0.05, unit: 'm' },
            { key: 'mountY', label: 'y₀', min: -3, max: 3, step: 0.05, unit: 'm' },
          ]
        : [
            {
              key: 'mountS',
              label: 'position s',
              min: 0,
              max: 1,
              step: 0.05,
              hint: 'joint position along the parent, as a fraction of its length (kept symbolic: s·l)',
            },
          ];
    joint.push({
      key: 'mountAngle',
      label: b.joint === 'fixed' ? 'angle' : 'mount angle',
      min: -180,
      max: 180,
      step: 15,
      unit: '°',
      scale: DEG,
      hint: 'rotation of the joint frame relative to the parent frame (0° = along the parent x-axis)',
    });
    if (b.joint === 'prismatic') {
      joint.push(
        { key: 'axisAngle', label: 'slide axis', min: -180, max: 180, step: 15, unit: '°', scale: DEG },
        {
          key: 'limit',
          label: 'range ±',
          min: 0,
          max: 3,
          step: 0.1,
          unit: 'm',
          hint: 'end stops (0 = unlimited)',
        },
      );
    }
    if (b.joint !== 'fixed') {
      joint.push(
        qField('q0', `${coord(String(i))}(0)`, 'initial value'),
        qField('qTarget', `${coord(String(i))}^{*}`, 'target / equilibrium used by the controllers'),
        {
          key: 'beta',
          label: `\\beta_{${i}}`,
          tex: true,
          min: 0,
          max: 2,
          step: 0.001,
          unit: angular ? 'N·m·s' : 'N·s/m',
          hint: 'viscous joint friction',
        },
      );
    }
    return joint;
  }

  private bodyFields(b: BodySpec, i: number): Field[] {
    const f: Field[] = [
      { key: 'm', label: `m_{${i + 1}}`, tex: true, min: 0.05, max: 10, log: true, unit: 'kg' },
      {
        key: 'l',
        label: `l_{${i + 1}}`,
        tex: true,
        min: 0,
        max: 2,
        step: 0.05,
        unit: 'm',
        hint: 'length: children sit at (s·l, 0) in this body frame',
      },
    ];
    if (!b.uniform) {
      f.push(
        {
          key: 'r',
          label: `r_{${i + 1}}`,
          tex: true,
          min: -1,
          max: 2,
          step: 0.01,
          unit: 'm',
          hint: 'centre of mass',
        },
        {
          key: 'I',
          label: `I_{${i + 1}}`,
          tex: true,
          min: 0,
          max: 2,
          step: 0.001,
          unit: 'kg·m²',
          hint: 'inertia',
        },
      );
    }
    return f;
  }

  private sliderFor(i: number, b: BodySpec, f: Field): HTMLElement {
    const scale = f.scale ?? 1;
    const opts = {
      label: f.label,
      tex: f.tex ?? false,
      min: f.min,
      max: f.max,
      value: (b[f.key] as number) * scale,
      log: f.log ?? false,
      onInput: (v: number) => this.update(i, { [f.key]: v / scale }, f.rerender),
      ...(f.step === undefined ? {} : { step: f.step }),
      ...(f.unit === undefined ? {} : { unit: f.unit }),
      ...(f.hint === undefined ? {} : { hint: f.hint }),
      ...(scale !== 1 ? { format: (v: number) => v.toFixed(0) } : {}),
      ...(f.key === 'mountS' ? { format: (v: number) => `${v.toFixed(2)} l` } : {}),
    };
    return slider(opts);
  }

  private bodyCard(i: number): HTMLElement {
    const b = this.mech.bodies[i]!;
    const c = card(`Joint ${i + 1}`);
    c.append(
      row(
        'type',
        segmented(
          [
            ['revolute', 'revolute'],
            ['prismatic', 'prismatic'],
            ['fixed', 'fixed'],
          ],
          b.joint,
          (v) => {
            if (v !== 'fixed' && b.joint === 'fixed' && dof(this.mech) >= MAX_DOF) {
              this.refuse(`at most ${MAX_DOF} moving joints`);
              return;
            }
            const m = cloneMechanism(this.mech);
            m.bodies[i] = normalise({
              ...b,
              joint: v,
              actuated: v !== 'fixed' && (b.actuated || b.joint === 'fixed'),
              limit: v === 'prismatic' ? Math.max(b.limit, 1) : 0,
              q0: 0,
              qTarget: 0,
            });
            if (dof(m) === 0) {
              this.refuse('at least one joint must be able to move');
              return;
            }
            this.commit({ ...m, name: 'Custom' });
          },
        ),
      ),
    );
    if (b.joint !== 'fixed') {
      c.append(
        row(
          'drive',
          segmented(
            [
              ['on', 'actuated'],
              ['off', 'passive'],
            ],
            b.actuated ? 'on' : 'off',
            (v) => this.update(i, { actuated: v === 'on' }, true),
          ),
        ),
      );
    }

    const parentSel = document.createElement('select');
    parentSel.className = 'btn small';
    parentSel.append(new Option('ground', '-1', false, b.parent < 0));
    this.mech.bodies.forEach((_, k) => {
      if (k !== i && !isDescendant(this.mech, k, i)) {
        parentSel.append(new Option(`body ${k + 1}`, String(k), false, b.parent === k));
      }
    });
    parentSel.addEventListener('change', () => {
      const res = reparent(this.mech, i, Number(parentSel.value));
      if (res) this.commit(res.mech, res.index);
    });
    c.append(row('hangs from', parentSel));
    if (b.parent >= 0) {
      const preset = (['0', '0.5', '1'] as const).find((s) => Number(s) === b.mountS) ?? ('' as '0');
      c.append(
        row(
          'on the parent at',
          segmented(
            [
              ['0', 'origin'],
              ['0.5', 'middle'],
              ['1', 'tip'],
            ],
            preset,
            (v) => this.update(i, { mountS: Number(v) }, true),
          ),
        ),
      );
    }
    for (const f of this.fields(b)) c.append(this.sliderFor(i, b, f));

    c.append(
      heading(`Body ${i + 1}`),
      row(
        'shape',
        segmented(
          [
            ['link', 'link'],
            ['block', 'block'],
          ],
          b.shape,
          (v) => this.update(i, { shape: v }),
        ),
      ),
    );
    for (const f of this.bodyFields(b, i)) c.append(this.sliderFor(i, b, f));
    c.append(
      row(
        'mass distribution',
        segmented(
          [
            ['uniform', 'uniform rod'],
            ['custom', 'custom'],
          ],
          b.uniform ? 'uniform' : 'custom',
          (v) => this.update(i, { uniform: v === 'uniform' }, true),
        ),
      ),
      row(
        'tool',
        segmented(
          [
            ['yes', 'tool at tip'],
            ['no', 'no'],
          ],
          this.mech.tool === i ? 'yes' : 'no',
          (v) => {
            const m = cloneMechanism(this.mech);
            m.tool = v === 'yes' ? i : m.tool === i ? -1 : m.tool;
            this.commit(m);
          },
        ),
      ),
    );
    return c;
  }
}
