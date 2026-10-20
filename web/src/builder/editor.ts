/**
 * Structure editor: the mechanism graph as a tree of bodies (drag to re-hang, × to remove) plus loop-closing
 * pins, and the settings of the selected joint and body. Every edit produces a new mechanism document; the
 * app re-derives it immediately and keeps an undo history.
 */
import { fix, tex } from '../eq/tex';
import {
  type BodySpec,
  cloneMechanism,
  dof,
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

const DEG = 180 / Math.PI;

interface SliderOpts {
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

export function slider(o: SliderOpts): HTMLElement {
  const el = document.createElement('label');
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
  el.append(name, out, input);
  return el;
}

export function segmented<T extends string>(
  options: [T, string][],
  value: T,
  onPick: (v: T) => void,
  small = true,
): HTMLElement {
  const seg = document.createElement('div');
  seg.className = `segmented${small ? ' small' : ''}`;
  for (const [v, label] of options) {
    const b = document.createElement('button');
    b.type = 'button';
    b.textContent = label;
    b.classList.toggle('on', v === value);
    b.addEventListener('click', () => {
      for (const x of seg.children) x.classList.toggle('on', x === b);
      onPick(v);
    });
    seg.append(b);
  }
  return seg;
}

function row(label: string, control: HTMLElement): HTMLElement {
  const el = document.createElement('div');
  el.className = 'param row';
  const name = document.createElement('span');
  name.className = 'param-name small';
  name.textContent = label;
  el.append(name, control);
  return el;
}

function card(title: string): HTMLElement {
  const c = document.createElement('section');
  c.className = 'param-card';
  const h = document.createElement('h3');
  h.textContent = title;
  c.append(h);
  return c;
}

export const portLabel = (s: number) =>
  s === 0 ? 'origin' : s === 1 ? 'tip' : s === 0.5 ? 'middle' : `${s.toFixed(2)} l`;

export interface EditorEvents {
  change(m: Mechanism): void;
  select(i: number): void;
  /** Start pinning a loop from the given port (the viewport then asks for the other end). */
  connect(body: number, s: number): void;
  flash(message: string): void;
}

export class Editor {
  private mech!: Mechanism;
  private selected = 0;
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

  load(mech: Mechanism, selected = this.selected): void {
    this.mech = cloneMechanism(mech);
    this.selected = Math.max(0, Math.min(selected, mech.bodies.length - 1));
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
    this.selected = Math.max(0, Math.min(selected, m.bodies.length - 1));
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

  addChild(parent: number, joint: JointKind, s = 1): void {
    const m = cloneMechanism(this.mech);
    if (m.bodies.length >= MAX_BODIES) {
      this.events.flash(`at most ${MAX_BODIES} bodies`);
      return;
    }
    if (joint !== 'fixed' && dof(m) >= MAX_DOF) {
      this.events.flash(`at most ${MAX_DOF} moving joints`);
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
    m.name = 'Custom';
    this.commit(m, m.bodies.length - 1);
  }

  remove(i: number): void {
    if (this.mech.bodies.length <= 1) {
      this.events.flash('a mechanism needs at least one body');
      return;
    }
    const m = removeBody(this.mech, i);
    if (dof(m) === 0) {
      this.events.flash('that would leave nothing that can move');
      return;
    }
    this.commit({ ...m, name: 'Custom' }, Math.max(0, Math.min(i - 1, m.bodies.length - 1)));
  }

  private render(): void {
    this.host.replaceChildren(
      this.presetsCard(),
      this.treeCard(),
      this.loopsCard(),
      this.bodyCard(this.selected),
    );
  }

  private presetsCard(): HTMLElement {
    const top = card('Start from');
    const presets = document.createElement('div');
    presets.className = 'preset-grid';
    for (const name of Object.keys(PRESETS)) {
      const b = document.createElement('button');
      b.type = 'button';
      b.className = `chip${name === this.mech.name ? ' on' : ''}`;
      b.textContent = name;
      b.addEventListener('click', () => this.commit(PRESETS[name]!(), 0));
      presets.append(b);
    }
    top.append(
      presets,
      slider({
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
      }),
    );
    return top;
  }

  private treeCard(): HTMLElement {
    const tree = card('Structure');
    const list = document.createElement('div');
    list.className = 'tree';

    const dropTarget = (el: HTMLElement, parent: number) => {
      el.addEventListener('dragover', (e) => {
        if (this.dragFrom < 0) return;
        const ok =
          this.dragFrom !== parent && (parent < 0 || !isDescendant(this.mech, parent, this.dragFrom));
        if (ok) e.preventDefault();
        el.classList.toggle('drop-ok', ok);
        el.classList.toggle('drop-bad', !ok);
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
          this.events.flash('a body cannot hang from its own descendant — pin a loop instead');
          return;
        }
        this.commit(res.mech, res.index);
      });
    };

    const groundRow = document.createElement('div');
    groundRow.className = 'tree-ground';
    groundRow.innerHTML =
      '<span class="ground-icon"></span>ground <span class="muted">— drop a body here to hang it from the ground</span>';
    dropTarget(groundRow, -1);
    list.append(groundRow);

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
      dropTarget(item, i);
      item.addEventListener('click', () => this.select(i));
      item.addEventListener('keydown', (e) => {
        if (e.key === 'Enter') this.select(i);
      });

      const icon = document.createElement('span');
      icon.className = `joint-icon ${b.joint}${b.actuated || b.joint === 'fixed' ? '' : ' passive'}`;
      const label = document.createElement('span');
      label.className = 'tree-label';
      const where = b.parent >= 0 ? ` @ ${portLabel(b.mountS)}` : '';
      const passive = b.joint !== 'fixed' && !b.actuated ? ', passive' : '';
      label.innerHTML = `Body ${i + 1} <span class="muted">${b.joint}${passive}${where}${this.mech.tool === i ? ', tool' : ''}</span>`;
      const q = document.createElement('span');
      q.className = 'tree-q';
      if (b.joint !== 'fixed') tex(q, b.joint === 'revolute' ? `\\theta_{${i + 1}}` : `d_{${i + 1}}`);
      const rm = document.createElement('button');
      rm.type = 'button';
      rm.className = 'icon-btn';
      rm.title = `remove body ${i + 1}${this.mech.bodies.some((x) => x.parent === i) ? ' and everything hanging from it' : ''}`;
      rm.textContent = '×';
      rm.addEventListener('click', (e) => {
        e.stopPropagation();
        this.remove(i);
      });
      item.append(icon, label, q, rm);
      list.append(item);
    });
    tree.append(list);

    const add = document.createElement('div');
    add.className = 'tree-actions';
    const parent = this.selected;
    for (const kind of ['revolute', 'prismatic', 'fixed'] as const) {
      const btn = document.createElement('button');
      btn.type = 'button';
      btn.className = 'btn small';
      btn.textContent = `+ ${kind}`;
      btn.title = `attach a ${kind} child at the tip of body ${parent + 1}`;
      btn.addEventListener('click', () => this.addChild(parent, kind));
      add.append(btn);
    }
    const ground = document.createElement('button');
    ground.type = 'button';
    ground.className = 'btn small ghost';
    ground.textContent = '+ on ground';
    ground.addEventListener('click', () => this.addChild(-1, 'revolute'));
    add.append(ground);
    const hint = document.createElement('p');
    hint.className = 'prose hint-small';
    hint.textContent =
      'In the view: click a + port on the selected body to attach a child there, drag ◆ to move its joint, drag ○ to rotate or stretch it (15° / 5 cm snapping, Alt for free). Delete removes, Ctrl+Z undoes.';
    tree.append(add, hint);
    return tree;
  }

  private loopsCard(): HTMLElement {
    const c = card('Loops');
    if (this.mech.closures.length === 0) {
      const p = document.createElement('p');
      p.className = 'prose hint-small';
      p.textContent =
        'Pin a point of one body to another body (or to the ground) to close a kinematic loop, e.g. a four-bar linkage. Each pin adds two constraints φ(q) = 0.';
      c.append(p);
    }
    this.mech.closures.forEach((cl, k) => {
      const item = document.createElement('div');
      item.className = 'tree-item static';
      const a =
        cl.a < 0
          ? `ground (${cl.gx.toFixed(2)}, ${cl.gy.toFixed(2)})`
          : `body ${cl.a + 1} @ ${portLabel(cl.sa)}`;
      item.innerHTML = `<span class="joint-icon pin"></span><span class="tree-label">${a} ↔ body ${cl.b + 1} @ ${portLabel(cl.sb)}</span>`;
      const rm = document.createElement('button');
      rm.type = 'button';
      rm.className = 'icon-btn';
      rm.title = 'remove this pin';
      rm.textContent = '×';
      rm.addEventListener('click', () => {
        const m = cloneMechanism(this.mech);
        m.closures.splice(k, 1);
        this.commit(m);
      });
      item.append(rm);
      c.append(item);
    });
    const pin = document.createElement('button');
    pin.type = 'button';
    pin.className = 'btn small';
    pin.textContent = `+ pin the tip of body ${this.selected + 1} to…`;
    pin.disabled = this.mech.closures.length >= MAX_CLOSURES;
    pin.addEventListener('click', () => this.events.connect(this.selected, 1));
    const actions = document.createElement('div');
    actions.className = 'tree-actions';
    actions.append(pin);
    c.append(actions);
    return c;
  }

  private bodyCard(i: number): HTMLElement {
    const b = this.mech.bodies[i]!;
    const c = card(`Joint ${i + 1}`);
    const fixed = b.joint === 'fixed';

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
              this.events.flash(`at most ${MAX_DOF} moving joints`);
              this.render();
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
              this.events.flash('at least one joint must be able to move');
              this.render();
              return;
            }
            this.commit({ ...m, name: 'Custom' });
          },
        ),
      ),
    );
    if (!fixed) {
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
      const preset = ['0', '0.5', '1'].includes(String(b.mountS)) ? String(b.mountS) : '';
      c.append(
        row(
          'on the parent at',
          segmented(
            [
              ['0', 'origin'],
              ['0.5', 'middle'],
              ['1', 'tip'],
            ],
            preset as '0' | '0.5' | '1',
            (v) => this.update(i, { mountS: Number(v) }, true),
          ),
        ),
        slider({
          label: 'position s',
          min: 0,
          max: 1,
          step: 0.05,
          value: b.mountS,
          format: (v) => `${v.toFixed(2)} l`,
          hint: 'joint position along the parent body, as a fraction of its length (kept symbolic: s·l)',
          onInput: (v) => this.update(i, { mountS: v }),
        }),
      );
    } else {
      c.append(
        slider({
          label: 'x₀',
          min: -3,
          max: 3,
          step: 0.05,
          value: b.mountX,
          unit: 'm',
          onInput: (v) => this.update(i, { mountX: v }),
        }),
        slider({
          label: 'y₀',
          min: -3,
          max: 3,
          step: 0.05,
          value: b.mountY,
          unit: 'm',
          onInput: (v) => this.update(i, { mountY: v }),
        }),
      );
    }
    c.append(
      slider({
        label: fixed ? 'angle' : 'mount angle',
        min: -180,
        max: 180,
        step: 15,
        value: b.mountAngle * DEG,
        unit: '°',
        format: (v) => v.toFixed(0),
        hint: 'rotation of the joint frame relative to the parent frame (0° = along the parent x-axis)',
        onInput: (v) => this.update(i, { mountAngle: v / DEG }),
      }),
    );
    if (b.joint === 'prismatic') {
      c.append(
        slider({
          label: 'slide axis',
          min: -180,
          max: 180,
          step: 15,
          value: b.axisAngle * DEG,
          unit: '°',
          format: (v) => v.toFixed(0),
          onInput: (v) => this.update(i, { axisAngle: v / DEG }),
        }),
        slider({
          label: 'range ±',
          min: 0,
          max: 3,
          step: 0.1,
          value: b.limit,
          unit: 'm',
          hint: 'end stops (0 = unlimited)',
          onInput: (v) => this.update(i, { limit: v }),
        }),
      );
    }
    if (!fixed) {
      const angular = b.joint === 'revolute';
      const qRange = angular ? 180 : Math.max(b.limit, 1);
      const qScale = angular ? DEG : 1;
      const qUnit = angular ? '°' : 'm';
      const sym = angular ? `\\theta_{${i + 1}}` : `d_{${i + 1}}`;
      const fmt = angular ? { format: (v: number) => v.toFixed(0) } : {};
      c.append(
        slider({
          label: String.raw`${sym}(0)`,
          tex: true,
          min: -qRange,
          max: qRange,
          step: angular ? 1 : 0.01,
          value: b.q0 * qScale,
          unit: qUnit,
          hint: 'initial value',
          ...fmt,
          onInput: (v) => this.update(i, { q0: v / qScale }),
        }),
        slider({
          label: String.raw`${sym}^{*}`,
          tex: true,
          min: -qRange,
          max: qRange,
          step: angular ? 1 : 0.01,
          value: b.qTarget * qScale,
          unit: qUnit,
          hint: 'target / equilibrium used by the controllers',
          ...fmt,
          onInput: (v) => this.update(i, { qTarget: v / qScale }),
        }),
        slider({
          label: String.raw`\beta_{${i + 1}}`,
          tex: true,
          min: 0,
          max: 2,
          step: 0.001,
          value: b.beta,
          unit: angular ? 'N·m·s' : 'N·s/m',
          hint: 'viscous joint friction',
          onInput: (v) => this.update(i, { beta: v }),
        }),
      );
    }

    const bodyHead = document.createElement('h3');
    bodyHead.textContent = `Body ${i + 1}`;
    c.append(bodyHead);
    c.append(
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
      slider({
        label: String.raw`m_{${i + 1}}`,
        tex: true,
        min: 0.05,
        max: 10,
        log: true,
        value: b.m,
        unit: 'kg',
        onInput: (v) => this.update(i, { m: v }),
      }),
      slider({
        label: String.raw`l_{${i + 1}}`,
        tex: true,
        min: 0,
        max: 2,
        step: 0.05,
        value: b.l,
        unit: 'm',
        hint: 'length: children sit at (s·l, 0) in this body frame',
        onInput: (v) => this.update(i, { l: v }),
      }),
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
    );
    if (!b.uniform) {
      c.append(
        slider({
          label: String.raw`r_{${i + 1}}`,
          tex: true,
          min: -1,
          max: 2,
          step: 0.01,
          value: b.r,
          unit: 'm',
          hint: 'centre of mass along the body x-axis',
          onInput: (v) => this.update(i, { r: v }),
        }),
        slider({
          label: String.raw`I_{${i + 1}}`,
          tex: true,
          min: 0,
          max: 2,
          step: 0.001,
          value: b.I,
          unit: 'kg·m²',
          hint: 'moment of inertia about the centre of mass',
          onInput: (v) => this.update(i, { I: v }),
        }),
      );
    }
    c.append(
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
