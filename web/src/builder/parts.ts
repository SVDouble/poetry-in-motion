/**
 * The parts of the builder's palette and how they join a mechanism: pure functions from a document and the
 * point(s) a part was dropped on to a new document (or the reason it cannot go there).
 */

import {
  cloneMechanism,
  dof,
  groundPoints,
  type JointKind,
  link,
  MAX_BODIES,
  MAX_CLOSURES,
  MAX_DOF,
  MAX_ELEMENTS,
  type Mechanism,
  reparent,
  spring,
} from '../model/description';
import { snap } from '../scene/geometry';
import { initialFrames } from '../scene/kinematics';

export type PartKind =
  | 'ground'
  | 'bar'
  | 'block'
  | 'revolute'
  | 'prismatic'
  | 'fixed'
  | 'spring'
  | 'damper'
  | 'springDamper'
  | 'pin'
  | 'slot';

export interface PartSpec {
  kind: PartKind;
  label: string;
  hint: string;
  group: 'Bodies' | 'Joints' | 'Connections';
  twoEnded: boolean;
}

export const PARTS: PartSpec[] = [
  {
    kind: 'ground',
    label: 'Ground',
    hint: 'place a fixed world attachment point',
    group: 'Bodies',
    twoEnded: false,
  },
  {
    kind: 'bar',
    label: 'Bar',
    hint: 'place a rigid bar; connect it with a separate joint',
    group: 'Bodies',
    twoEnded: false,
  },
  {
    kind: 'block',
    label: 'Block',
    hint: 'place a rigid block; a translational joint makes it a rail carriage',
    group: 'Bodies',
    twoEnded: false,
  },
  {
    kind: 'revolute',
    label: 'Rotational',
    hint: '1 relative angular DOF; choose an anchor, then the body to attach',
    group: 'Joints',
    twoEnded: true,
  },
  {
    kind: 'prismatic',
    label: 'Translational',
    hint: '1 relative sliding DOF; choose an anchor, then the body to attach',
    group: 'Joints',
    twoEnded: true,
  },
  {
    kind: 'fixed',
    label: 'Fixed',
    hint: '0 relative DOF; rigidly attach a body',
    group: 'Joints',
    twoEnded: true,
  },
  {
    kind: 'spring',
    label: 'Spring',
    hint: 'a linear spring between two points',
    group: 'Connections',
    twoEnded: true,
  },
  {
    kind: 'damper',
    label: 'Damper',
    hint: 'a viscous damper between two points',
    group: 'Connections',
    twoEnded: true,
  },
  {
    kind: 'springDamper',
    label: 'Spring–damper',
    hint: 'both, side by side',
    group: 'Connections',
    twoEnded: true,
  },
  {
    kind: 'pin',
    label: 'Loop pin',
    hint: 'close a loop between two points (2 constraints)',
    group: 'Connections',
    twoEnded: true,
  },
  {
    kind: 'slot',
    label: 'Loop slot',
    hint: 'close a loop with a sliding point (1 constraint)',
    group: 'Connections',
    twoEnded: true,
  },
];
export const isJoint = (kind: PartKind): kind is JointKind =>
  kind === 'revolute' || kind === 'prismatic' || kind === 'fixed';

export const partSpec = (k: PartKind) => PARTS.find((p) => p.kind === k)!;

/** A point a part attaches to: `s` along body `body`, or the ground (`body` = −1) at (x, y). */
export interface Anchor {
  body: number;
  s: number;
  x: number;
  y: number;
}

export type Placed =
  | { mech: Mechanism; select: { kind: 'body' | 'joint' | 'element'; index: number } }
  | string;

/** A body is initially an unconnected draft, with no implied joint or ground support. */
export function placeBody(mech: Mechanism, kind: 'bar' | 'block', at: Anchor): Placed {
  if (mech.bodies.length >= MAX_BODIES) return `at most ${MAX_BODIES} bodies`;
  const m = cloneMechanism(mech);
  m.bodies.push(
    link(-1, 'fixed', {
      detached: true,
      mountX: snap(at.x),
      mountY: snap(at.y),
      m: 0.5,
      l: kind === 'bar' ? 0.8 : 0,
      ...(kind === 'block' ? { shape: 'block', uniform: false, r: 0, I: 0.02 } : {}),
    }),
  );
  return { mech: { ...m, name: 'Custom' }, select: { kind: 'body', index: m.bodies.length - 1 } };
}

/** Attach a loose body at its origin. Reordering preserves all existing graph references. */
export function connectBody(mech: Mechanism, joint: JointKind, at: Anchor, body: number): Placed {
  if (!Number.isInteger(at.body) || at.body < -1 || at.body >= mech.bodies.length)
    return 'Choose a valid joint anchor.';
  const child = mech.bodies[body];
  if (!child?.detached) return 'Choose an unconnected body. To close a loop, use Loop pin or Loop slot.';
  if (at.body < 0 && !groundPoints(mech).some(([x, y]) => Math.hypot(x - at.x, y - at.y) < 1e-8))
    return 'Place a Ground anchor first, then start the joint there.';
  if (joint !== 'fixed' && dof(mech) >= MAX_DOF) return `at most ${MAX_DOF} moving joints`;
  const frames = initialFrames(mech);
  const angle = frames[body]![2] - (at.body < 0 ? 0 : frames[at.body]![2]);
  const result = reparent(mech, body, at.body);
  if (!result) return 'This would create a cycle. Use a loop closure for an already connected branch.';
  const b = result.mech.bodies[result.index]!;
  delete b.detached;
  Object.assign(b, {
    joint,
    actuated: joint !== 'fixed',
    mountS: at.body < 0 ? 0 : at.s,
    mountX: at.x,
    mountY: at.y,
    mountAngle: angle,
    axisAngle: -angle,
    q0: 0,
    qTarget: 0,
    limit: joint === 'prismatic' ? 1.5 : 0,
  });
  if (result.mech.tool < 0 && b.shape === 'link') result.mech.tool = result.index;
  return { mech: result.mech, select: { kind: 'joint', index: result.index } };
}

/** Complete a pending joint by drawing a new body from its anchor toward the pointer. */
export function placeAtJoint(
  mech: Mechanism,
  kind: 'bar' | 'block',
  joint: JointKind,
  from: Anchor,
  to: Anchor,
): Placed {
  const placed = placeBody(mech, kind, from);
  if (typeof placed === 'string') return placed;
  const b = placed.mech.bodies[placed.select.index]!;
  if (kind === 'bar') {
    b.mountAngle = Math.atan2(to.y - from.y, to.x - from.x);
    b.l = Math.max(0.1, Math.min(3, snap(Math.hypot(to.x - from.x, to.y - from.y))));
  }
  return connectBody(placed.mech, joint, from, placed.select.index);
}

/** Disconnect an incoming joint while retaining the body's current pose and its descendants. */
export function detachBody(mech: Mechanism, body: number): Mechanism {
  const [x, y, angle] = initialFrames(mech)[body]!;
  const m = cloneMechanism(mech);
  Object.assign(m.bodies[body]!, {
    detached: true,
    parent: -1,
    joint: 'fixed',
    actuated: false,
    mountX: x,
    mountY: y,
    mountAngle: angle,
    q0: 0,
    qTarget: 0,
  });
  return { ...m, name: 'Custom' };
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
  if (kind === 'ground') {
    const m = cloneMechanism(mech);
    m.anchors ??= [];
    const anchors = m.anchors;
    if (anchors.length >= MAX_BODIES) return `at most ${MAX_BODIES} ground anchors`;
    const point: [number, number] = [snap(a.x), snap(a.y)];
    if (!groundPoints(m).some(([x, y]) => Math.hypot(x - point[0], y - point[1]) < 1e-8)) anchors.push(point);
    return { mech: { ...m, name: 'Custom' }, select: { kind: 'body', index: -1 } };
  }
  if (kind === 'bar' || kind === 'block') return placeBody(mech, kind, a);
  if (!b) return 'pick the second end';
  if (isJoint(kind)) return connectBody(mech, kind, a, b.body);
  if (kind === 'pin' || kind === 'slot') return placeLoop(mech, kind, a, b);
  return placeElement(mech, kind, a, b);
}
