/**
 * Forward kinematics of a mechanism description at its initial pose, without the simulator: the same
 * composition as the core (`g_si = g_s,parent · g_parent,joint · g_joint,i(qᵢ)`). Used to frame the view
 * on the pose and to draw preset thumbnails. Loop closures are not solved: the presets start closed.
 */
import { type Mechanism, normalise } from '../model/description';
import { BLOCK, type Bounds, compose, type Pose } from './geometry';

/** Body frames at q = q0 (index i = body i). */
export function initialFrames(mech: Mechanism): Pose[] {
  const frames: Pose[] = [];
  for (const raw of mech.bodies) {
    const b = normalise(raw);
    const parent = b.parent < 0 ? null : frames[b.parent]!;
    const mount: Pose =
      parent === null
        ? [b.mountX, b.mountY, b.mountAngle]
        : compose(parent, [b.mountS * normalise(mech.bodies[b.parent]!).l, 0, b.mountAngle]);
    const joint: Pose =
      b.joint === 'revolute'
        ? [0, 0, b.q0]
        : b.joint === 'prismatic'
          ? [Math.cos(b.axisAngle) * b.q0, Math.sin(b.axisAngle) * b.q0, 0]
          : [0, 0, 0];
    frames.push(compose(mount, joint));
  }
  return frames;
}

/** Region covered by the mechanism at its initial pose (bodies, carts, rails, ground points). */
export function poseBounds(mech: Mechanism): Bounds {
  const frames = initialFrames(mech);
  let [minX, minY, maxX, maxY] = [Infinity, Infinity, -Infinity, -Infinity];
  const grow = (x: number, y: number, r = 0) => {
    minX = Math.min(minX, x - r);
    maxX = Math.max(maxX, x + r);
    minY = Math.min(minY, y - r);
    maxY = Math.max(maxY, y + r);
  };
  mech.bodies.forEach((raw, i) => {
    const b = normalise(raw);
    const [x, y, p] = frames[i]!;
    grow(x, y, b.shape === 'block' ? BLOCK.w / 2 : 0.05);
    grow(x + Math.cos(p) * b.l, y + Math.sin(p) * b.l, 0.05);
    if (b.parent < 0) {
      grow(b.mountX, b.mountY, 0.15);
      if (b.joint === 'prismatic') {
        const a = b.mountAngle + b.axisAngle;
        const half = (b.limit > 0 ? b.limit : 1) + 0.2;
        grow(b.mountX + Math.cos(a) * half, b.mountY + Math.sin(a) * half);
        grow(b.mountX - Math.cos(a) * half, b.mountY - Math.sin(a) * half);
      }
    }
  });
  for (const c of mech.closures) if (c.a < 0) grow(c.gx, c.gy, 0.15);
  for (const e of mech.elements) if (e.a < 0) grow(e.gx, e.gy, 0.15);
  if (!Number.isFinite(minX)) return { cx: 0, cy: 0, width: 3, height: 3 };
  // a margin for labels and the swing of the bodies, and never closer than 1.6 m across
  const pad = 0.35 + 0.15 * Math.max(maxX - minX, maxY - minY);
  const width = Math.max(1.6, maxX - minX + 2 * pad);
  const height = Math.max(1.6, maxY - minY + 2 * pad);
  return { cx: (minX + maxX) / 2, cy: (minY + maxY) / 2, width, height };
}
