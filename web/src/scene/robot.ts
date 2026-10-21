/**
 * Geometry of a built mechanism at the pose of a snapshot: points along bodies, joint and parent frames,
 * picking, and the region it can reach. The schematic draws from it and the builder edits through it.
 */
import { type Mechanism, normalise, PARAMS_PER_BODY } from '../model/description';
import type { Snapshot } from '../sim/layout';
import { BLOCK, type Bounds, mechanismBounds, mountPose, type Pose } from './geometry';

/** Where each body's parameters sit in the simulator's parameter vector (PARAMS_PER_BODY per body). */
const PARAM = { m: 0, l: 1, r: 2, I: 3, beta: 4 } as const;

/** Parameter `k` of body i, or undefined when there is no such body. */
export const param = (params: readonly number[], i: number, k: keyof typeof PARAM): number | undefined =>
  params[PARAMS_PER_BODY * i + PARAM[k]];

/** Distance from (px, py) to the segment a–b. */
export function segmentDistance(px: number, py: number, [ax, ay]: Point, [bx, by]: Point): number {
  const [vx, vy] = [bx - ax, by - ay];
  const len2 = vx * vx + vy * vy;
  const t = len2 > 0 ? Math.max(0, Math.min(1, ((px - ax) * vx + (py - ay) * vy) / len2)) : 0;
  return Math.hypot(px - (ax + t * vx), py - (ay + t * vy));
}

export type Point = [number, number];

export class RobotView {
  constructor(readonly mech: Mechanism) {}

  /** Region that the mechanism can reach (for framing the view). */
  bounds(): Bounds {
    return mechanismBounds(this.mech);
  }

  /** Length of body i (current parameter value). */
  length(i: number, params: number[]): number {
    return param(params, i, 'l') ?? this.mech.bodies[i]?.l ?? 0;
  }

  /** Centre of mass of body i. */
  com(i: number, snap: Snapshot, params: number[]): Point {
    const [x, y, p] = snap.frame(i + 1);
    const b = this.mech.bodies[i];
    const r = param(params, i, 'r') ?? (b ? normalise(b).r : 0);
    return [x + Math.cos(p) * r, y + Math.sin(p) * r];
  }

  /** A point that is either on the ground (body < 0) or at fraction s along a body. */
  anchor(body: number, s: number, ground: Point, snap: Snapshot, params: number[]): Point {
    return body < 0 ? ground : this.port(body, s, snap, params);
  }

  /** The two ends of spring/damper k. */
  elementEnds(k: number, snap: Snapshot, params: number[]): [Point, Point] {
    const e = this.mech.elements[k]!;
    return [this.anchor(e.a, e.sa, [e.gx, e.gy], snap, params), this.port(e.b, e.sb, snap, params)];
  }

  /** The spring/damper passing within `reach` of a point, or −1. */
  pickElement(x: number, y: number, snap: Snapshot, params: number[], reach: number): number {
    let [best, bestD] = [-1, reach];
    this.mech.elements.forEach((_, k) => {
      const d = segmentDistance(x, y, ...this.elementEnds(k, snap, params));
      if (d < bestD) [best, bestD] = [k, d];
    });
    return best;
  }

  /** World position of the point at fraction `s` along body i. */
  port(i: number, s: number, snap: Snapshot, params: number[]): Point {
    const [x, y, p] = snap.frame(i + 1);
    const d = s * this.length(i, params);
    return [x + Math.cos(p) * d, y + Math.sin(p) * d];
  }

  /** Nearest body to a point in the plane, or -1. */
  pick(x: number, y: number, snap: Snapshot, params: number[]): number {
    let best = -1;
    let bestD = 0.25;
    this.mech.bodies.forEach((b, i) => {
      const ends: [Point, Point] = [this.port(i, 0, snap, params), this.port(i, 1, snap, params)];
      const d = segmentDistance(x, y, ...ends) - (b.shape === 'block' ? BLOCK.h / 2 : 0);
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
    const root = b.parent < 0;
    return mountPose(b, root ? null : snap.frame(b.parent + 1), root ? 0 : this.length(b.parent, params));
  }
}
