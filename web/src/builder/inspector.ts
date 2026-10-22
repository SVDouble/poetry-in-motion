/**
 * The inspector: click a body or a spring/damper in the builder view and its settings open beside it —
 * shape, joint, drive, and every number with ◀ ▶ arrows (hold to repeat, Shift for fine steps).
 *
 * The control builders are shared with the side panel, so both always offer the same edits.
 */
import {
  type BodySpec,
  cloneMechanism,
  dof,
  type Element,
  MAX_DOF,
  type Mechanism,
  normalise,
  removeBody,
} from '../model/description';
import { GRID, PORT_STEP, SNAP_ANGLE } from '../scene/geometry';
import { button, DEG, el, row, segmented, stepper } from '../ui/widgets';

export type Selection = { kind: 'body'; index: number } | { kind: 'element'; index: number };

export interface EditContext {
  mech: Mechanism;
  /** A new document; `structural`: the set of controls changes (re-render them). */
  commit(m: Mechanism, structural?: boolean): void;
  refuse(message: string): void;
  /** Current length of spring/damper k (for "relax"). */
  length(k: number): number;
}

const patchBody = (c: EditContext, i: number, patch: Partial<BodySpec>, structural = false) => {
  const m = cloneMechanism(c.mech);
  m.bodies[i] = normalise({ ...m.bodies[i]!, ...patch });
  c.commit({ ...m, name: 'Custom' }, structural);
};

const patchElement = (c: EditContext, k: number, patch: Partial<Element>, structural = false) => {
  const m = cloneMechanism(c.mech);
  m.elements[k] = { ...m.elements[k]!, ...patch };
  c.commit({ ...m, name: 'Custom' }, structural);
};

/** The essentials of body i and its joint. */
export function bodyControls(c: EditContext, i: number): HTMLElement[] {
  const b = c.mech.bodies[i]!;
  const out: HTMLElement[] = [];
  out.push(
    row(
      'shape',
      segmented(
        [
          ['link', 'bar'],
          ['block', 'block'],
        ],
        b.shape,
        (v) => patchBody(c, i, { shape: v }, true),
      ),
    ),
    row(
      'joint',
      segmented(
        [
          ['revolute', 'hinge'],
          ['prismatic', 'slider'],
          ['fixed', 'weld'],
        ],
        b.joint,
        (v) => {
          if (v !== 'fixed' && b.joint === 'fixed' && dof(c.mech) >= MAX_DOF) {
            c.refuse(`at most ${MAX_DOF} moving joints`);
            return;
          }
          const m = cloneMechanism(c.mech);
          m.bodies[i] = normalise({
            ...b,
            joint: v,
            actuated: v !== 'fixed' && (b.actuated || b.joint === 'fixed'),
            limit: v === 'prismatic' ? Math.max(b.limit, 1) : 0,
            q0: 0,
            qTarget: 0,
          });
          if (dof(m) === 0) {
            c.refuse('at least one joint must be able to move');
            return;
          }
          c.commit({ ...m, name: 'Custom' }, true);
        },
      ),
    ),
  );
  if (b.joint !== 'fixed') {
    out.push(
      row(
        'drive',
        segmented(
          [
            ['on', 'driven'],
            ['off', 'passive'],
          ],
          b.actuated ? 'on' : 'off',
          (v) => patchBody(c, i, { actuated: v === 'on' }, true),
        ),
      ),
    );
  }
  out.push(
    stepper({
      label: `l_{${i + 1}}`,
      tex: true,
      value: b.l,
      step: GRID,
      min: 0,
      max: 3,
      unit: 'm',
      hint: 'length',
      onChange: (v) => patchBody(c, i, { l: v }),
    }),
    stepper({
      label: `m_{${i + 1}}`,
      tex: true,
      value: b.m,
      step: 0.1,
      min: 0.01,
      max: 50,
      unit: 'kg',
      hint: 'mass',
      onChange: (v) => patchBody(c, i, { m: v }),
    }),
    row(
      'mass',
      segmented(
        [
          ['uniform', 'uniform rod'],
          ['custom', 'custom'],
        ],
        b.uniform ? 'uniform' : 'custom',
        (v) => patchBody(c, i, { uniform: v === 'uniform' }, true),
      ),
    ),
  );
  if (!b.uniform) {
    out.push(
      stepper({
        label: `r_{${i + 1}}`,
        tex: true,
        value: b.r,
        step: GRID,
        min: -2,
        max: 3,
        unit: 'm',
        hint: 'centre of mass along the body',
        onChange: (v) => patchBody(c, i, { r: v }),
      }),
      stepper({
        label: `I_{${i + 1}}`,
        tex: true,
        value: b.I,
        step: 0.01,
        min: 0,
        max: 10,
        unit: 'kg·m²',
        hint: 'inertia about the centre of mass',
        onChange: (v) => patchBody(c, i, { I: v }),
      }),
    );
  }
  out.push(
    stepper({
      label: b.joint === 'fixed' ? 'angle' : 'mount angle',
      value: b.mountAngle,
      step: SNAP_ANGLE,
      min: -Math.PI,
      max: Math.PI,
      unit: '°',
      scale: DEG,
      digits: 0,
      hint: 'rotation of the joint frame relative to the parent (0° = along the parent)',
      onChange: (v) => patchBody(c, i, { mountAngle: v }),
    }),
  );
  if (b.parent >= 0) {
    out.push(
      stepper({
        label: 'on the parent at',
        value: b.mountS,
        step: PORT_STEP,
        min: 0,
        max: 1,
        unit: 'l',
        hint: 'joint position along the parent, as a fraction of its length',
        onChange: (v) => patchBody(c, i, { mountS: v }),
      }),
    );
  }
  if (b.joint === 'prismatic') {
    out.push(
      stepper({
        label: 'slide axis',
        value: b.axisAngle,
        step: SNAP_ANGLE,
        min: -Math.PI,
        max: Math.PI,
        unit: '°',
        scale: DEG,
        digits: 0,
        onChange: (v) => patchBody(c, i, { axisAngle: v }),
      }),
      stepper({
        label: 'range ±',
        value: b.limit,
        step: 0.1,
        min: 0,
        max: 5,
        unit: 'm',
        hint: 'end stops (0 = unlimited)',
        onChange: (v) => patchBody(c, i, { limit: v }),
      }),
    );
  }
  if (b.joint !== 'fixed') {
    const angular = b.joint === 'revolute';
    out.push(
      stepper({
        label: `${angular ? '\\theta' : 'd'}_{${i + 1}}(0)`,
        tex: true,
        value: b.q0,
        step: angular ? Math.PI / 36 : GRID,
        min: angular ? -Math.PI : -5,
        max: angular ? Math.PI : 5,
        unit: angular ? '°' : 'm',
        ...(angular ? { scale: DEG, digits: 0 } : {}),
        hint: 'initial value',
        onChange: (v) => patchBody(c, i, { q0: v }),
      }),
      stepper({
        label: `\\beta_{${i + 1}}`,
        tex: true,
        value: b.beta,
        step: 0.01,
        min: 0,
        max: 5,
        unit: angular ? 'N·m·s' : 'N·s/m',
        hint: 'viscous joint friction',
        onChange: (v) => patchBody(c, i, { beta: v }),
      }),
    );
  }
  return out;
}

/** Stiffness, rest length, damping and the two ends of spring/damper k. */
export function elementControls(c: EditContext, k: number): HTMLElement[] {
  const e = c.mech.elements[k]!;
  const kind = e.k > 0 && e.d > 0 ? 'both' : e.k > 0 ? 'spring' : 'damper';
  const out: HTMLElement[] = [
    row(
      'kind',
      segmented(
        [
          ['spring', 'spring'],
          ['damper', 'damper'],
          ['both', 'both'],
        ],
        kind,
        (v) =>
          patchElement(
            c,
            k,
            {
              k: v === 'damper' ? 0 : e.k > 0 ? e.k : 50,
              d: v === 'spring' ? 0 : e.d > 0 ? e.d : 2,
            },
            true,
          ),
      ),
    ),
  ];
  if (e.k > 0) {
    out.push(
      stepper({
        label: `k_{${k + 1}}`,
        tex: true,
        value: e.k,
        step: 5,
        min: 0.5,
        max: 5000,
        unit: 'N/m',
        hint: 'stiffness',
        onChange: (v) => patchElement(c, k, { k: v }),
      }),
      stepper({
        label: `\\ell_{0,${k + 1}}`,
        tex: true,
        value: e.l0,
        step: GRID,
        min: 0,
        max: 5,
        unit: 'm',
        hint: 'rest length',
        onChange: (v) => patchElement(c, k, { l0: v }),
      }),
      el(
        'div',
        'card-actions',
        button(
          'relax: ℓ₀ = current length',
          () => patchElement(c, k, { l0: Number(c.length(k).toFixed(3)) }, true),
          'btn small',
          'set the rest length to the distance between the ends now',
        ),
      ),
    );
  }
  if (e.d > 0) {
    out.push(
      stepper({
        label: `c_{${k + 1}}`,
        tex: true,
        value: e.d,
        step: 0.1,
        min: 0.01,
        max: 500,
        unit: 'N·s/m',
        hint: 'damping',
        onChange: (v) => patchElement(c, k, { d: v }),
      }),
    );
  }
  const end = (which: 'a' | 'b') => {
    const body = which === 'a' ? e.a : e.b;
    if (body < 0) {
      return [
        stepper({
          label: 'ground x',
          value: e.gx,
          step: GRID,
          min: -10,
          max: 10,
          unit: 'm',
          onChange: (v) => patchElement(c, k, { gx: v }),
        }),
        stepper({
          label: 'ground y',
          value: e.gy,
          step: GRID,
          min: -10,
          max: 10,
          unit: 'm',
          onChange: (v) => patchElement(c, k, { gy: v }),
        }),
      ];
    }
    return [
      stepper({
        label: `end on body ${body + 1} at`,
        value: which === 'a' ? e.sa : e.sb,
        step: PORT_STEP,
        min: 0,
        max: 1,
        unit: 'l',
        hint: 'position along the body, as a fraction of its length',
        onChange: (v) => patchElement(c, k, which === 'a' ? { sa: v } : { sb: v }),
      }),
    ];
  };
  out.push(...end('a'), ...end('b'));
  return out;
}

export function describe(mech: Mechanism, sel: Selection): string {
  if (sel.kind === 'element') {
    const e = mech.elements[sel.index]!;
    const kind = e.k > 0 && e.d > 0 ? 'Spring–damper' : e.k > 0 ? 'Spring' : 'Damper';
    const a = e.a < 0 ? 'ground' : `body ${e.a + 1}`;
    return `${kind} ${sel.index + 1} · ${a} ↔ body ${e.b + 1}`;
  }
  const b = mech.bodies[sel.index]!;
  const joint = b.joint === 'revolute' ? 'hinge' : b.joint === 'prismatic' ? 'slider' : 'weld';
  return `Body ${sel.index + 1} · ${joint}${b.joint !== 'fixed' ? (b.actuated ? ', driven' : ', passive') : ''}`;
}

export interface InspectorEvents {
  context(): EditContext;
  remove(sel: Selection): void;
  /** Make the body the tool, or not. */
  tool(i: number, on: boolean): void;
  close(): void;
}

/** The floating card next to the selection in the view. */
export class Inspector {
  readonly el = el('div', 'inspector overlay');
  private sel: Selection | null = null;

  constructor(
    host: HTMLElement,
    private readonly events: InspectorEvents,
  ) {
    this.el.hidden = true;
    host.append(this.el);
    // keep wheel and pointer gestures inside the card
    this.el.addEventListener('pointerdown', (e) => e.stopPropagation());
  }

  get selection(): Selection | null {
    return this.sel;
  }

  /** Open for `sel`. */
  show(sel: Selection): void {
    this.sel = sel;
    this.render();
  }

  hide(): void {
    this.sel = null;
    this.el.hidden = true;
  }

  /** Rebuild the controls (after a structural change, or when the document changed elsewhere). */
  render(): void {
    const sel = this.sel;
    const c = this.events.context();
    const valid =
      sel && (sel.kind === 'body' ? sel.index < c.mech.bodies.length : sel.index < c.mech.elements.length);
    if (!sel || !valid) {
      this.hide();
      return;
    }
    const head = el('div', 'inspector-head', el('b', '', describe(c.mech, sel)));
    head.append(button('×', () => this.events.close(), 'icon-btn', 'close (Esc)'));
    const body = el('div', 'inspector-body');
    body.append(...(sel.kind === 'body' ? bodyControls(c, sel.index) : elementControls(c, sel.index)));
    const foot = el('div', 'inspector-foot');
    if (sel.kind === 'body') {
      const isTool = c.mech.tool === sel.index;
      foot.append(
        button(isTool ? 'tool at tip ✓' : 'make tip the tool', () => this.events.tool(sel.index, !isTool)),
      );
    }
    const removable = sel.kind === 'element' || c.mech.bodies.length > 1;
    const rm = button('delete', () => this.events.remove(sel), 'btn small danger');
    rm.disabled = !removable;
    foot.append(rm);
    this.el.replaceChildren(head, body, foot);
    this.el.hidden = false;
    this.position();
  }

  /** Docked at the right edge of the view, below the toggles: the mechanism is framed in the middle and
   * the parts palette sits on the left, which must stay reachable. */
  private position(): void {
    this.el.style.right = '12px';
  }
}

/** Remove a body (and what hangs from it) or a spring/damper. */
export function removeSelection(mech: Mechanism, sel: Selection): Mechanism | string {
  if (sel.kind === 'element') {
    const m = cloneMechanism(mech);
    m.elements.splice(sel.index, 1);
    return { ...m, name: 'Custom' };
  }
  if (mech.bodies.length <= 1) return 'a mechanism needs at least one body';
  const m = removeBody(mech, sel.index);
  if (dof(m) === 0) return 'that would leave nothing that can move';
  return { ...m, name: 'Custom' };
}
