/**
 * Generic visualisation of a built mechanism: bodies, joints (actuated hinges in steel, passive ones in amber,
 * welds as plates), ground anchors, prismatic rails with end stops, loop-closing pins, frames, centres of
 * mass, tool, targets, torque glyphs and a tool trail. Geometry follows the current parameter values every
 * frame; poses come from the snapshot.
 */
import * as THREE from 'three/webgpu';
import { actuatorIndex, coordIndex, type Mechanism, normalise } from '../model/description';
import type { Snapshot } from '../sim/layout';
import { Arrow, glow, matte, metal, TorqueArc, Trail, triad } from './primitives';
import type { Stage } from './stage';
import { palette } from './theme';

const LINK_COLORS = [0x5b6fa8, 0x7a8fcf, 0x8b7cff, 0x5fa8c9, 0x9a86d9, 0x6f8fb8, 0x7d6fd0, 0x5a8fd6];
const FIXED_COLOR = 0x59617a;
export const BLOCK = { w: 0.36, h: 0.18, d: 0.22 };

interface BodyVis {
  group: THREE.Group;
  shaft: THREE.Mesh;
  tip: THREE.Mesh;
  root: THREE.Mesh;
  com: THREE.Mesh;
  frame: THREE.Group;
  materials: THREE.MeshStandardMaterial[];
}

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

/** Hatched plate marking a point fixed to the ground, facing direction `dir`. */
function groundAnchor(dir: number): THREE.Group {
  const g = new THREE.Group();
  const plate = new THREE.Mesh(new THREE.BoxGeometry(0.06, 0.34, 0.24), matte(0x3a4258));
  plate.position.x = -0.1;
  g.add(plate);
  const pts: number[] = [];
  for (let k = -2; k <= 2; k++) {
    const y = k * 0.07;
    pts.push(-0.13, y, 0, -0.21, y - 0.07, 0);
  }
  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.Float32BufferAttribute(pts, 3));
  g.add(new THREE.LineSegments(geo, new THREE.LineBasicMaterial({ color: 0x8a93ad })));
  g.rotation.z = dir;
  return g;
}

export class RobotView {
  private readonly bodies: BodyVis[] = [];
  private readonly rails: { group: THREE.Group; body: number }[] = [];
  private readonly arcs = new Map<number, TorqueArc>();
  private readonly arrows = new Map<number, Arrow>();
  private readonly targets = new Map<number, THREE.Mesh>();
  private readonly pins: THREE.Group[] = [];
  private readonly slots = new Map<number, THREE.Group>();
  private readonly toolBall: THREE.Mesh;
  private readonly toolTarget: THREE.Group;
  /** Secondary goals: the target orientation at the tool, and the elbow target. */
  private readonly orientTarget: Arrow;
  private readonly elbowTarget: THREE.Group;
  private readonly trail = new Trail(palette.accent, 600);
  private readonly frames: THREE.Group[] = [];
  private readonly gArrow: Arrow;
  private readonly spring: THREE.Line;
  private readonly coord: number[];
  private readonly actuator: number[];
  showFrames = true;
  showTrail = true;

  constructor(
    stage: Stage,
    readonly mech: Mechanism,
  ) {
    const root = stage.root;
    this.coord = coordIndex(mech);
    this.actuator = actuatorIndex(mech);

    const ground = triad();
    root.add(ground);
    stage.label(ground, String.raw`\{s\}`, 'frame', -30, 8);
    this.frames.push(ground);

    mech.bodies.forEach((spec, i) => {
      const fixed = spec.joint === 'fixed';
      const color = fixed ? FIXED_COLOR : LINK_COLORS[i % LINK_COLORS.length]!;
      const mat = metal(color, 0.35, 0.55);
      const materials = [mat];
      const group = new THREE.Group();
      const radius = fixed ? 0.055 : 0.045;
      const shaft = new THREE.Mesh(new THREE.CylinderGeometry(radius, radius, 1, 24), mat);
      shaft.rotation.z = -Math.PI / 2;
      const capGeo = new THREE.SphereGeometry(radius, 24, 16);
      const tip = new THREE.Mesh(capGeo, mat);
      const rootCap = new THREE.Mesh(capGeo, mat);
      group.add(shaft, tip, rootCap);

      if (spec.shape === 'block') {
        const blockMat = metal(0x46598a, 0.4, 0.5);
        materials.push(blockMat);
        group.add(new THREE.Mesh(new THREE.BoxGeometry(BLOCK.w, BLOCK.h, BLOCK.d), blockMat));
        for (const sx of [-0.11, 0.11]) {
          const wheel = new THREE.Mesh(
            new THREE.CylinderGeometry(0.04, 0.04, BLOCK.d + 0.02, 24),
            matte(0x11151e),
          );
          wheel.rotation.x = Math.PI / 2;
          wheel.position.set(sx, -BLOCK.h / 2 - 0.02, 0);
          group.add(wheel);
        }
      }

      if (spec.joint === 'revolute') {
        const jointMat = spec.actuated ? metal(palette.steel, 0.2, 0.9) : metal(palette.coriolis, 0.35, 0.4);
        const hinge = new THREE.Mesh(new THREE.CylinderGeometry(0.07, 0.07, 0.16, 32), jointMat);
        hinge.rotation.x = Math.PI / 2;
        hinge.position.z = 0.04;
        group.add(hinge);
      } else if (fixed) {
        const weld = new THREE.Mesh(new THREE.BoxGeometry(0.11, 0.11, 0.14), metal(0x8a93ad, 0.5, 0.4));
        weld.rotation.z = Math.PI / 4;
        group.add(weld);
      }
      const com = new THREE.Mesh(new THREE.SphereGeometry(0.03, 16, 12), glow(palette.gravity));
      com.position.z = 0.12;
      com.visible = !fixed;
      group.add(com);
      root.add(group);

      const frame = triad();
      root.add(frame);
      stage.label(frame, String.raw`\{${i + 1}\}`, 'frame', 10, i % 2 ? 10 : -24);
      this.frames.push(frame);
      this.bodies.push({ group, shaft, tip, root: rootCap, com, frame, materials });

      if (spec.parent < 0 && spec.joint !== 'prismatic') {
        const anchor = groundAnchor(spec.mountAngle);
        anchor.position.set(spec.mountX, spec.mountY, -0.05);
        root.add(anchor);
      }

      if (spec.joint === 'prismatic') {
        const rail = new THREE.Group();
        const len = 2 * (spec.limit > 0 ? spec.limit : 2) + (spec.shape === 'block' ? BLOCK.w : 0.1);
        const bar = new THREE.Mesh(new THREE.BoxGeometry(len, 0.035, 0.08), metal(palette.steel, 0.25, 0.8));
        bar.position.y = spec.shape === 'block' ? -BLOCK.h / 2 - 0.07 : 0;
        bar.position.z = -0.15;
        rail.add(bar);
        if (spec.limit > 0) {
          for (const s of [-1, 1]) {
            const stop = new THREE.Mesh(
              new THREE.BoxGeometry(0.05, 0.16, 0.14),
              metal(palette.warn, 0.5, 0.2),
            );
            stop.position.set(s * (len / 2 + 0.025), bar.position.y + 0.06, -0.15);
            rail.add(stop);
          }
        }
        root.add(rail);
        this.rails.push({ group: rail, body: i });
        const tgt = new THREE.Mesh(new THREE.ConeGeometry(0.04, 0.08, 3), glow(palette.reference, 0.9));
        tgt.rotation.z = Math.PI;
        rail.add(tgt);
        this.targets.set(i, tgt);
      }

      if (spec.actuated && !fixed) {
        if (spec.joint === 'revolute') {
          const arc = new TorqueArc(palette.input, 0.16);
          root.add(arc);
          this.arcs.set(i, arc);
        } else {
          const arrow = new Arrow(palette.input, 0.016, 0.07);
          root.add(arrow);
          this.arrows.set(i, arrow);
        }
      }
    });

    // loop closures: a pin, or a slider block in a slot (the slot follows body a, or sits on the ground)
    for (const c of mech.closures) {
      const pin = new THREE.Group();
      if (c.kind === 'slot') {
        const slot = new THREE.Group();
        const groove = new THREE.Mesh(new THREE.BoxGeometry(3, 0.05, 0.06), metal(0x3a4258, 0.5, 0.4));
        groove.position.z = -0.12;
        const edge = new THREE.Mesh(new THREE.BoxGeometry(3, 0.012, 0.08), glow(palette.accent, 0.5));
        edge.position.z = -0.08;
        slot.add(groove, edge);
        root.add(slot);
        this.slots.set(this.pins.length, slot);
        const block = new THREE.Mesh(new THREE.BoxGeometry(0.16, 0.1, 0.12), metal(palette.accent, 0.3, 0.6));
        block.position.z = 0.02;
        pin.add(block);
      }
      const disc = new THREE.Mesh(
        new THREE.CylinderGeometry(0.06, 0.06, 0.2, 32),
        metal(palette.accent, 0.3, 0.6),
      );
      disc.rotation.x = Math.PI / 2;
      disc.position.z = 0.08;
      const ring = new THREE.Mesh(new THREE.TorusGeometry(0.085, 0.01, 8, 40), glow(palette.accent));
      ring.position.z = 0.2;
      pin.add(disc, ring);
      root.add(pin);
      this.pins.push(pin);
      if (c.a < 0) {
        const anchor = groundAnchor(-Math.PI / 2);
        anchor.position.set(c.gx, c.gy, -0.05);
        root.add(anchor);
      }
    }

    this.toolBall = new THREE.Mesh(new THREE.SphereGeometry(0.05, 24, 16), metal(0xe6ebf5, 0.25, 0.7));
    this.toolBall.visible = mech.tool >= 0;
    root.add(this.toolBall);
    const toolFrame = triad(0.16);
    root.add(toolFrame);
    toolFrame.visible = mech.tool >= 0;
    if (mech.tool >= 0) stage.label(toolFrame, String.raw`\{t\}`, 'frame', 10, -24);
    this.frames.push(toolFrame);

    this.toolTarget = new THREE.Group();
    this.toolTarget.add(
      new THREE.Mesh(new THREE.TorusGeometry(0.09, 0.008, 8, 48), glow(palette.reference)),
      new THREE.Mesh(new THREE.CircleGeometry(0.018, 24), glow(palette.reference)),
    );
    this.toolTarget.position.z = 0.4;
    root.add(this.toolTarget, this.trail);
    this.orientTarget = new Arrow(palette.coriolis, 0.008, 0.05);
    this.orientTarget.visible = false;
    this.elbowTarget = new THREE.Group();
    this.elbowTarget.add(
      new THREE.Mesh(new THREE.TorusGeometry(0.07, 0.007, 8, 40), glow(palette.coriolis)),
      new THREE.Mesh(new THREE.CircleGeometry(0.014, 20), glow(palette.coriolis)),
    );
    this.elbowTarget.visible = false;
    root.add(this.orientTarget, this.elbowTarget);

    // the push on the tool: a spring from the tool to the pointer
    const springGeo = new THREE.BufferGeometry();
    springGeo.setAttribute('position', new THREE.BufferAttribute(new Float32Array(6), 3));
    this.spring = new THREE.Line(springGeo, new THREE.LineBasicMaterial({ color: palette.input }));
    this.spring.frustumCulled = false;
    this.spring.visible = false;
    root.add(this.spring);

    this.gArrow = new Arrow(palette.gravity, 0.01, 0.05);
    this.gArrow.angle = -Math.PI / 2;
    this.gArrow.set(0.35);
    root.add(this.gArrow);
    stage.label(this.gArrow, 'g', 'gravity', 10, 12);
  }

  /** Region that the mechanism can reach (for framing the camera). */
  bounds(): Bounds {
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
    const reach = this.mech.bodies.map(() => 0);
    for (let i = this.mech.bodies.length - 1; i >= 0; i--) {
      const b = normalise(this.mech.bodies[i]!);
      const own = Math.max(b.l, b.shape === 'block' ? BLOCK.w : 0);
      reach[i] = Math.max(reach[i]!, own);
      if (b.parent >= 0) {
        const pb = this.mech.bodies[b.parent]!;
        const offset = b.mountS * pb.l;
        reach[b.parent] = Math.max(
          reach[b.parent]!,
          offset + reach[i]! + (b.joint === 'prismatic' ? b.limit : 0),
        );
      }
    }
    this.mech.bodies.forEach((b, i) => {
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
    for (const c of this.mech.closures) if (c.a < 0) grow(c.gx, c.gy, 0.3);
    if (!Number.isFinite(minX)) return { cx: 0, cy: 0, width: 4, height: 4 };
    return {
      cx: (minX + maxX) / 2,
      cy: (minY + maxY) / 2,
      width: maxX - minX + 0.4,
      height: maxY - minY + 0.4,
    };
  }

  select(i: number): void {
    this.bodies.forEach((b, k) => {
      for (const m of b.materials) m.emissive.setHex(k === i ? 0x3a5bc0 : 0x000000);
    });
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

  /** Show the push spring from the tool to `at` (world), or hide it. */
  push(at: [number, number] | null, snap: Snapshot): void {
    this.spring.visible = at !== null;
    if (!at) return;
    const pos = this.spring.geometry.getAttribute('position') as THREE.BufferAttribute;
    pos.setXYZ(0, snap.v('TOOL_P'), snap.v('TOOL_P', 1), 0.45);
    pos.setXYZ(1, at[0], at[1], 0.45);
    pos.needsUpdate = true;
  }

  update(snap: Snapshot, params: number[], running = true): void {
    const ik = running && snap.v('USE_TOOL') !== 0;
    const nb = this.mech.bodies.length;
    const umax = snap.v('UMAX') || 1;
    this.mech.bodies.forEach((spec, i) => {
      const v = this.bodies[i]!;
      const [x, y, p] = snap.frame(i + 1);
      v.group.position.set(x, y, 0);
      v.group.rotation.z = p;
      const l = this.length(i, params);
      const r = params[5 * i + 2] ?? spec.r;
      const showLink = l > 1e-3;
      v.shaft.visible = showLink;
      v.tip.visible = showLink;
      v.root.visible = showLink && spec.joint !== 'revolute' && spec.shape !== 'block';
      v.shaft.scale.y = Math.max(l, 1e-3);
      v.shaft.position.x = l / 2;
      v.tip.position.x = l;
      v.com.position.x = r;
      v.frame.position.set(x, y, 0.32);
      v.frame.rotation.z = p;
      v.frame.visible = this.showFrames;

      const a = this.actuator[i]!;
      const arc = this.arcs.get(i);
      if (arc && a >= 0) {
        arc.position.set(x, y, 0.35);
        arc.set(snap.v('U', a) / umax, p);
      }
      const arrow = this.arrows.get(i);
      if (arrow && a >= 0) {
        const [, , jp] = this.jointPose(i, snap, params);
        const axis = jp + spec.axisAngle;
        arrow.angle = axis;
        const u = snap.v('U', a) / umax;
        arrow.position.set(
          x + Math.cos(axis) * 0.2 * Math.sign(u),
          y + Math.sin(axis) * 0.2 * Math.sign(u),
          0.4,
        );
        arrow.set(u * 0.9);
      }
    });

    for (const { group, body } of this.rails) {
      const spec = this.mech.bodies[body]!;
      const [jx, jy, jp] = this.jointPose(body, snap, params);
      group.position.set(jx, jy, 0);
      group.rotation.z = jp + spec.axisAngle;
      const tgt = this.targets.get(body);
      const j = this.coord[body]!;
      if (tgt && j >= 0) {
        tgt.visible = spec.actuated && snap.v('KIND') !== 0;
        tgt.position.set(snap.v('Q_TARGET', j), spec.shape === 'block' ? -BLOCK.h / 2 - 0.02 : 0.08, 0.3);
      }
    }

    this.mech.closures.forEach((c, k) => {
      const [px, py] = this.port(c.b, c.sb, snap, params);
      this.pins[k]!.position.set(px, py, 0);
      const slot = this.slots.get(k);
      if (slot) {
        const [ax, ay] = c.a < 0 ? [c.gx, c.gy] : this.port(c.a, c.sa, snap, params);
        const dir = (c.a < 0 ? 0 : snap.frame(c.a + 1)[2]) + c.angle;
        slot.position.set(ax, ay, 0);
        slot.rotation.z = dir;
        this.pins[k]!.rotation.z = dir;
      }
    });

    this.frames[0]!.visible = this.showFrames;
    const toolFrame = this.frames[nb + 1]!;
    const [tx, ty, tp] = snap.frame(nb + 1);
    toolFrame.position.set(tx, ty, 0.36);
    toolFrame.rotation.z = tp;
    toolFrame.visible = this.showFrames && this.mech.tool >= 0;
    this.toolBall.position.set(tx, ty, 0.15);

    this.toolTarget.visible = ik;
    // secondary goals (snapshot GOALS: orientation ω, φ*, elbow ω, x*, y*, body)
    const orientOn = running && snap.v('GOALS') > 0 && this.mech.tool >= 0;
    this.orientTarget.visible = orientOn;
    if (orientOn) {
      this.orientTarget.position.set(tx, ty, 0.42);
      this.orientTarget.angle = snap.v('GOALS', 1);
      this.orientTarget.set(0.22);
    }
    const elbowOn = running && snap.v('GOALS', 2) > 0;
    this.elbowTarget.visible = elbowOn;
    if (elbowOn) this.elbowTarget.position.set(snap.v('GOALS', 3), snap.v('GOALS', 4), 0.42);
    this.toolTarget.position.set(snap.v('TARGET', 0), snap.v('TARGET', 1), 0.4);
    const reach = snap.v('REACHABLE') !== 0;
    for (const c of this.toolTarget.children) {
      ((c as THREE.Mesh).material as THREE.MeshBasicMaterial).color.setHex(
        reach ? palette.reference : palette.warn,
      );
    }

    const g = params[params.length - 1] ?? 9.81;
    this.gArrow.set(Math.min(0.5, (g / 9.81) * 0.35));
    const b = this.bounds();
    this.gArrow.position.set(b.cx + b.width / 2 - 0.3, b.cy - b.height / 2 + 0.55, 0);

    this.trail.visible = this.showTrail && this.mech.tool >= 0 && running;
    if (this.mech.tool >= 0 && running) this.trail.push(tx, ty);
  }

  resetTrail(): void {
    this.trail.reset();
  }
}
