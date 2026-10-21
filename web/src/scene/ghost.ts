/**
 * A shadow simulation drawn as a translucent silhouette of the mechanism: one bar per body, the cart
 * outline for blocks, the tool point and its trail, all in the slot's colour.
 */
import * as THREE from 'three/webgpu';
import type { Mechanism } from '../model/description';
import type { SlotView } from '../sim/layout';
import { BLOCK } from './geometry';
import { glow, Trail } from './primitives';
import type { Stage } from './stage';

const OPACITY = 0.38;

export class GhostView {
  private readonly bodies: { group: THREE.Group; bar: THREE.Mesh }[] = [];
  private readonly tool: THREE.Mesh;
  private readonly trail: Trail;
  private readonly root = new THREE.Group();

  constructor(
    stage: Stage,
    private readonly mech: Mechanism,
    color: number,
  ) {
    const mat = glow(color, OPACITY);
    for (const spec of mech.bodies) {
      const group = new THREE.Group();
      const bar = new THREE.Mesh(new THREE.BoxGeometry(1, 0.05, 0.02), mat);
      group.add(bar);
      if (spec.shape === 'block')
        group.add(new THREE.Mesh(new THREE.BoxGeometry(BLOCK.w, BLOCK.h, 0.02), mat));
      group.position.z = 0.5;
      this.root.add(group);
      this.bodies.push({ group, bar });
    }
    this.tool = new THREE.Mesh(new THREE.SphereGeometry(0.045, 16, 12), glow(color, 0.8));
    this.tool.visible = mech.tool >= 0;
    this.trail = new Trail(color, 400);
    this.root.add(this.tool, this.trail);
    stage.root.add(this.root);
  }

  set visible(on: boolean) {
    this.root.visible = on;
  }

  update(slot: SlotView, length: (i: number) => number, running: boolean): void {
    this.root.visible = slot.on;
    if (!slot.on) return;
    this.mech.bodies.forEach((_, i) => {
      const { group, bar } = this.bodies[i]!;
      const [x, y, p] = slot.frame(i + 1);
      group.position.set(x, y, 0.5);
      group.rotation.z = p;
      const l = length(i);
      bar.visible = l > 1e-3;
      bar.scale.x = Math.max(l, 1e-3);
      bar.position.x = l / 2;
    });
    const tx = slot.v('TOOL');
    const ty = slot.v('TOOL', 1);
    this.tool.position.set(tx, ty, 0.55);
    if (this.mech.tool >= 0 && running) this.trail.push(tx, ty);
  }

  resetTrail(): void {
    this.trail.reset();
  }
}
