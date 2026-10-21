/**
 * Geometry of a built mechanism at the pose of a snapshot: points along bodies, joint and parent frames,
 * picking, and the region it can reach. The schematic draws from it and the builder edits through it.
 */
import type { Mechanism } from '../model/description';
import type { Snapshot } from '../sim/layout';
import { BLOCK, type Bounds, compose, mechanismBounds, type Pose } from './geometry';

export class RobotView {
  constructor(readonly mech: Mechanism) {}

  /** Region that the mechanism can reach (for framing the view). */
  bounds(): Bounds {
    return mechanismBounds(this.mech);
  }

  /** Length of body i (current parameter value). */
  length(i: number, params: number[]): number {
    return params[5 * i + 1] ?? this.mech.bodies[i]?.l ?? 0;
  }

  /** World position of the point at fraction `s` along body i. */
  port(i: number, s: number, snap: Snapshot, params: number[]): [number, number] {
    const [x, y, p] = snap.frame(i + 1);
    const d = s * this.length(i, params);
    return [x + Math.cos(p) * d, y + Math.sin(p) * d];
  }

  /** Nearest body to a point in the plane, or -1. */
  pick(x: number, y: number, snap: Snapshot, params: number[]): number {
    let best = -1;
    let bestD = 0.25;
    this.mech.bodies.forEach((b, i) => {
      const [bx, by] = snap.frame(i + 1);
      const [tx, ty] = this.port(i, 1, snap, params);
      const vx = tx - bx;
      const vy = ty - by;
      const len2 = vx * vx + vy * vy;
      const t = len2 > 0 ? Math.max(0, Math.min(1, ((x - bx) * vx + (y - by) * vy) / len2)) : 0;
      const d = Math.hypot(x - (bx + t * vx), y - (by + t * vy)) - (b.shape === 'block' ? BLOCK.h / 2 : 0);
      if (d < bestD) {
        bestD = d;
        best = i;
      }
    });
    return best;
  }

  /** Pose of the parent frame of body i (the ground frame for roots). */
  parentPose(i: number, snap: Snapshot): Pose {
    const b = this.mech.bodies[i]!;
    return b.parent < 0 ? [0, 0, 0] : snap.frame(b.parent + 1);
  }

  /** Pose of joint i's frame (where its rail or hinge sits) in the spatial frame. */
  jointPose(i: number, snap: Snapshot, params: number[]): Pose {
    const b = this.mech.bodies[i]!;
    if (b.parent < 0) return [b.mountX, b.mountY, b.mountAngle];
    const lp = this.length(b.parent, params);
    return compose(snap.frame(b.parent + 1), [b.mountS * lp, 0, b.mountAngle]);
  }
}
