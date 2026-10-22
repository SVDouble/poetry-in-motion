/** Direct manipulation in the builder: pure functions from a pointer position to a new document. */
import { cloneMechanism, type Mechanism, normalise } from '../model/description';
import { PORT_STEP, SNAP_ANGLE, snap } from '../scene/geometry';
import type { RobotView } from '../scene/robot';
import type { Snapshot } from '../sim/layout';

const wrapAngle = (a: number) => Math.atan2(Math.sin(a), Math.cos(a));

export interface DragContext {
  mech: Mechanism;
  robot: RobotView;
  snap: Snapshot;
  params: number[];
  /** Metres between the drawn tip handle and the actual tip. */
  tipOffset: number;
}

/** ◆ handle: move a ground joint, or slide a child joint along its parent (s ∈ [0, 1]). */
export function dragJoint(
  c: DragContext,
  i: number,
  [x, y]: [number, number],
  free: boolean,
): Mechanism | null {
  const b = c.mech.bodies[i];
  if (!b || !c.robot.mech.bodies[i]) return null;
  const m = cloneMechanism(c.mech);
  const t = m.bodies[i]!;
  if (b.parent < 0) {
    t.mountX = free ? x : snap(x);
    t.mountY = free ? y : snap(y);
  } else {
    const [px, py, pp] = c.snap.frame(b.parent + 1);
    const l = c.robot.length(b.parent, c.params);
    if (l < 1e-6) return null;
    const along = ((x - px) * Math.cos(pp) + (y - py) * Math.sin(pp)) / l;
    t.mountS = Math.max(0, Math.min(1, free ? along : snap(along, PORT_STEP)));
  }
  return { ...m, name: 'Custom' };
}

/**
 * ○ handle: point the body at the pointer. By default this sets the mounting angle (where θ = 0 points) and
 * the length; with `initial` it sets the initial joint angle θ(0) instead.
 */
export function dragTip(
  c: DragContext,
  i: number,
  [x, y]: [number, number],
  free: boolean,
  initial: boolean,
): Mechanism | null {
  const b = c.mech.bodies[i];
  if (!b || !c.robot.mech.bodies[i]) return null;
  const [ox, oy] = c.snap.frame(i + 1);
  const [, , parentPhi] = c.robot.parentPose(i, c.snap);
  const desired = Math.atan2(y - oy, x - ox);
  const m = cloneMechanism(c.mech);
  const t = m.bodies[i]!;
  if (initial && b.joint === 'revolute') {
    const q0 = wrapAngle(desired - parentPhi - b.mountAngle);
    t.q0 = free ? q0 : snap(q0, SNAP_ANGLE);
    return { ...m, name: 'Custom' };
  }
  const own = b.joint === 'revolute' ? b.q0 : 0;
  const mount = wrapAngle(desired - parentPhi - own);
  t.mountAngle = free ? mount : snap(mount, SNAP_ANGLE);
  if (b.shape !== 'block') {
    const len = Math.hypot(x - ox, y - oy) - c.tipOffset;
    t.l = Math.max(0.1, Math.min(3, free ? len : snap(len)));
    m.bodies[i] = normalise(t);
  }
  return { ...m, name: 'Custom' };
}

/** Close a loop from `from` to a port of another body (b ≥ 0) or to a ground point (b = −1). */
export function closeLoop(
  mech: Mechanism,
  from: { body: number; s: number; kind: 'pin' | 'slot' },
  b: number,
  s: number,
  [x, y]: [number, number],
): Mechanism {
  const m = cloneMechanism(mech);
  const ground = b < 0;
  // a slot lives on the "a" side: the target body (or the ground); the starting point slides in it
  m.closures.push({
    kind: from.kind,
    angle: 0,
    a: ground ? -1 : b,
    sa: ground ? 0 : s,
    b: from.body,
    sb: from.s,
    gx: ground ? snap(x) : 0,
    gy: ground ? snap(y) : 0,
  });
  return { ...m, name: 'Custom' };
}
