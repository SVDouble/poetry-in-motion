/**
 * The parts of the builder's palette and how they join a mechanism: pure functions from a document and the
 * point(s) a part was dropped on to a new document (or the reason it cannot go there).
 */
import {
  cloneMechanism,
  dof,
  type JointKind,
  link,
  MAX_BODIES,
  MAX_CLOSURES,
  MAX_DOF,
  MAX_ELEMENTS,
  type Mechanism,
  spring,
} from '../model/description';

export type PartKind =
  | 'bar'
  | 'slider'
  | 'cart'
  | 'weld'
  | 'spring'
  | 'damper'
  | 'springDamper'
  | 'pin'
  | 'slot';

export interface PartSpec {
  kind: PartKind;
  label: string;
  hint: string;
  /** Joins two points (dropped one after the other) instead of hanging from one. */
  twoEnded: boolean;
}

export const PARTS: PartSpec[] = [
  { kind: 'bar', label: 'Bar', hint: 'a link on a hinge (revolute joint)', twoEnded: false },
  { kind: 'slider', label: 'Slider', hint: 'a link on a sliding joint (prismatic)', twoEnded: false },
  { kind: 'cart', label: 'Cart', hint: 'a block on a rail (prismatic)', twoEnded: false },
  { kind: 'weld', label: 'Weld', hint: 'a rigid bracket (fixed joint)', twoEnded: false },
  { kind: 'spring', label: 'Spring', hint: 'a linear spring between two points', twoEnded: true },
  { kind: 'damper', label: 'Damper', hint: 'a viscous damper between two points', twoEnded: true },
  { kind: 'springDamper', label: 'Spring–damper', hint: 'both, side by side', twoEnded: true },
  {
    kind: 'pin',
    label: 'Pin',
    hint: 'close a loop: pin a point to another body or the ground',
    twoEnded: true,
  },
  { kind: 'slot', label: 'Slot', hint: 'close a loop: let a point slide in a slot', twoEnded: true },
];

export const partSpec = (k: PartKind) => PARTS.find((p) => p.kind === k)!;

/** A point a part attaches to: `s` along body `body`, or the ground (`body` = −1) at (x, y). */
export interface Anchor {
  body: number;
  s: number;
  x: number;
  y: number;
}

export type Placed = { mech: Mechanism; select: { kind: 'body' | 'element'; index: number } } | string;

const SNAP = 0.05;
const snap = (v: number) => Math.round(v / SNAP) * SNAP;

/** Hang a bar, slider, cart or weld from `at`. */
export function placeBody(mech: Mechanism, kind: PartKind, at: Anchor): Placed {
  if (mech.bodies.length >= MAX_BODIES) return `at most ${MAX_BODIES} bodies`;
  const joint: JointKind = kind === 'bar' ? 'revolute' : kind === 'weld' ? 'fixed' : 'prismatic';
  if (joint !== 'fixed' && dof(mech) >= MAX_DOF) return `at most ${MAX_DOF} moving joints`;
  const m = cloneMechanism(mech);
  const parent = at.body;
  const onGround = parent < 0;
  // continue straight from a tip, stand perpendicular elsewhere, hang down from the ground; fan siblings
  const siblings = m.bodies.filter(
    (b) => b.parent === parent && (onGround || Math.abs(b.mountS - at.s) < 1e-9),
  );
  const fan = siblings.length * (Math.PI / 4) * (siblings.length % 2 ? 1 : -1);
  const angle = onGround ? (kind === 'cart' ? 0 : -Math.PI / 2) : at.s < 1 ? Math.PI / 2 : 0;
  const where = onGround ? { mountX: snap(at.x), mountY: snap(at.y), mountS: 0 } : { mountS: at.s };
  const patch =
    kind === 'cart'
      ? { m: 1, l: 0, r: 0, I: 0, uniform: false, limit: 1.5, shape: 'block' as const, beta: 0.1 }
      : kind === 'slider'
        ? { m: 0.5, l: 0.5, limit: 1 }
        : { m: 0.5, l: kind === 'weld' ? 0.6 : 0.8 };
  m.bodies.push(link(parent, joint, { ...where, mountAngle: angle + (onGround ? 0 : fan), ...patch }));
  const index = m.bodies.length - 1;
  if (kind !== 'weld' && (m.tool < 0 || m.tool === parent)) m.tool = index;
  return { mech: { ...m, name: 'Custom' }, select: { kind: 'body', index } };
}

/** Join two points with a spring, damper or both (it starts relaxed: ℓ₀ = the current distance). */
export function placeElement(mech: Mechanism, kind: PartKind, a: Anchor, b: Anchor): Placed {
  if (mech.elements.length >= MAX_ELEMENTS) return `at most ${MAX_ELEMENTS} springs and dampers`;
  // the second end must be on a body: put the ground (if any) first
  const [p, q] = b.body < 0 ? [b, a] : [a, b];
  if (q.body < 0) return 'a spring or damper needs at least one end on a body';
  if (p.body === q.body) return 'both ends are on the same body: it would do nothing';
  const l0 = Math.max(0.05, Number(Math.hypot(q.x - p.x, q.y - p.y).toFixed(3)));
  const [k, d] = kind === 'spring' ? [50, 0] : kind === 'damper' ? [0, 2] : [50, 2];
  const m = cloneMechanism(mech);
  m.elements.push(
    spring(p.body, p.s, q.body, q.s, {
      gx: p.body < 0 ? snap(p.x) : 0,
      gy: p.body < 0 ? snap(p.y) : 0,
      k,
      l0,
      d,
    }),
  );
  return { mech: { ...m, name: 'Custom' }, select: { kind: 'element', index: m.elements.length - 1 } };
}

/** Close a loop: pin point `a` (on a body) to `b`, or let it slide in a slot through `b`. */
export function placeLoop(mech: Mechanism, kind: 'pin' | 'slot', a: Anchor, b: Anchor): Placed {
  if (mech.closures.length >= MAX_CLOSURES) return `at most ${MAX_CLOSURES} loops`;
  const [from, to] = a.body < 0 ? [b, a] : [a, b];
  if (from.body < 0) return 'a loop needs a point on a body';
  if (from.body === to.body) return 'a body cannot be pinned to itself';
  const m = cloneMechanism(mech);
  const ground = to.body < 0;
  m.closures.push({
    kind,
    angle: 0,
    a: ground ? -1 : to.body,
    sa: ground ? 0 : to.s,
    b: from.body,
    sb: from.s,
    gx: ground ? snap(to.x) : 0,
    gy: ground ? snap(to.y) : 0,
  });
  return { mech: { ...m, name: 'Custom' }, select: { kind: 'body', index: from.body } };
}

/** Place any part: one anchor for bodies, two for springs, dampers and loops. */
export function place(mech: Mechanism, kind: PartKind, a: Anchor, b?: Anchor): Placed {
  if (!partSpec(kind).twoEnded) return placeBody(mech, kind, a);
  if (!b) return 'pick the second end';
  if (kind === 'pin' || kind === 'slot') return placeLoop(mech, kind, a, b);
  return placeElement(mech, kind, a, b);
}
