/** Plain geometry shared by the 3D scene and the schematic (no three.js). */
import { type Mechanism, normalise } from '../model/description';

export const BLOCK = { w: 0.36, h: 0.18, d: 0.22 };

export interface Bounds {
  cx: number;
  cy: number;
  width: number;
  height: number;
}

export type Pose = [number, number, number];
export const compose = ([x, y, p]: Pose, [dx, dy, dp]: Pose): Pose => [
  x + Math.cos(p) * dx - Math.sin(p) * dy,
  y + Math.sin(p) * dx + Math.cos(p) * dy,
  p + dp,
];

/** Region that the mechanism can reach (for framing the view). */
export function mechanismBounds(mech: Mechanism): Bounds {
  let minX = Infinity;
  let maxX = -Infinity;
  let minY = Infinity;
  let maxY = -Infinity;
  const grow = (x: number, y: number, r: number) => {
    minX = Math.min(minX, x - r);
    maxX = Math.max(maxX, x + r);
    minY = Math.min(minY, y - r);
    maxY = Math.max(maxY, y + r);
  };
  const reach = mech.bodies.map(() => 0);
  for (let i = mech.bodies.length - 1; i >= 0; i--) {
    const b = normalise(mech.bodies[i]!);
    const own = Math.max(b.l, b.shape === 'block' ? BLOCK.w : 0);
    reach[i] = Math.max(reach[i]!, own);
    if (b.parent >= 0) {
      const pb = mech.bodies[b.parent]!;
      const offset = b.mountS * pb.l;
      reach[b.parent] = Math.max(
        reach[b.parent]!,
        offset + reach[i]! + (b.joint === 'prismatic' ? b.limit : 0),
      );
    }
  }
  mech.bodies.forEach((b, i) => {
    if (b.parent >= 0) return;
    const r = reach[i]! + 0.25;
    if (b.joint === 'prismatic') {
      const lim = b.limit > 0 ? b.limit : 2;
      const a = b.mountAngle + b.axisAngle;
      for (const s of [-1, 1]) grow(b.mountX + s * lim * Math.cos(a), b.mountY + s * lim * Math.sin(a), r);
    } else {
      grow(b.mountX, b.mountY, r);
    }
  });
  for (const c of mech.closures) if (c.a < 0) grow(c.gx, c.gy, 0.3);
  if (!Number.isFinite(minX)) return { cx: 0, cy: 0, width: 4, height: 4 };
  return {
    cx: (minX + maxX) / 2,
    cy: (minY + maxY) / 2,
    width: maxX - minX + 0.4,
    height: maxY - minY + 0.4,
  };
}
