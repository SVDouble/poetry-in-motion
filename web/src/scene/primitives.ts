/** Reusable scene pieces: materials, coordinate-frame triads, dynamic arrows, arcs and trails. */
import * as THREE from 'three/webgpu';
import { palette } from './theme';

export function metal(color: number, roughness = 0.35, metalness = 0.55): THREE.MeshStandardMaterial {
  return new THREE.MeshStandardMaterial({ color, roughness, metalness });
}

export function matte(color: number, opacity = 1): THREE.MeshStandardMaterial {
  return new THREE.MeshStandardMaterial({
    color,
    roughness: 0.8,
    metalness: 0,
    transparent: opacity < 1,
    opacity,
    depthWrite: opacity >= 1,
  });
}

export function glow(color: number, opacity = 1): THREE.MeshBasicMaterial {
  return new THREE.MeshBasicMaterial({ color, transparent: opacity < 1, opacity, depthWrite: opacity >= 1 });
}

/** Solid arrow pointing along +x from the origin; `set(length)` rescales the shaft only. */
export class Arrow extends THREE.Group {
  private readonly shaft: THREE.Mesh;
  private readonly head: THREE.Mesh;

  /** Direction of a positive arrow, radians from +x. */
  angle = 0;

  constructor(
    color: number,
    radius = 0.012,
    private readonly headLen = 0.06,
  ) {
    super();
    const mat = glow(color);
    this.shaft = new THREE.Mesh(new THREE.CylinderGeometry(radius, radius, 1, 12), mat);
    this.shaft.rotation.z = -Math.PI / 2;
    this.head = new THREE.Mesh(new THREE.ConeGeometry(radius * 2.6, headLen, 16), mat);
    this.head.rotation.z = -Math.PI / 2;
    this.add(this.shaft, this.head);
    this.set(0.3);
  }

  /** Signed length along +x; hidden when tiny. */
  set(len: number): void {
    const a = Math.abs(len);
    this.visible = a > 1e-3;
    if (!this.visible) return;
    const s = Math.max(a - this.headLen, 1e-4);
    this.shaft.scale.set(1, s, 1);
    this.shaft.position.x = s / 2;
    this.head.position.x = s + this.headLen / 2;
    this.rotation.z = this.angle + (len < 0 ? Math.PI : 0);
    this.shaft.visible = a > this.headLen;
  }
}

/** x (red) / y (green) axes and a z dot — a planar coordinate frame. */
export function triad(len = 0.22): THREE.Group {
  const g = new THREE.Group();
  const x = new Arrow(palette.axisX, 0.008, 0.045);
  x.set(len);
  const y = new Arrow(palette.axisY, 0.008, 0.045);
  y.angle = Math.PI / 2;
  y.set(len);
  const o = new THREE.Mesh(new THREE.SphereGeometry(0.018, 16, 12), glow(0xe8eefc));
  g.add(x, y, o);
  g.position.z = 0.3;
  return g;
}

/** Circular arc with an arrow head showing a joint torque (sign and magnitude). */
export class TorqueArc extends THREE.Group {
  private readonly line: THREE.Line;
  private readonly positions: Float32Array;
  private readonly head: THREE.Mesh;

  constructor(
    color: number,
    private readonly r = 0.12,
    private readonly segments = 48,
  ) {
    super();
    this.positions = new Float32Array((segments + 1) * 3);
    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.BufferAttribute(this.positions, 3));
    this.line = new THREE.Line(geo, new THREE.LineBasicMaterial({ color }));
    this.head = new THREE.Mesh(new THREE.ConeGeometry(0.022, 0.05, 12), glow(color));
    this.add(this.line, this.head);
    this.position.z = 0.35;
  }

  /** `frac` ∈ [−1, 1] of a full (0.85 turn) arc; positive is counter-clockwise. */
  set(frac: number, start = 0): void {
    const f = Math.max(-1, Math.min(1, frac));
    this.visible = Math.abs(f) > 0.01;
    if (!this.visible) return;
    const sweep = f * Math.PI * 1.7;
    for (let k = 0; k <= this.segments; k++) {
      const a = start + (sweep * k) / this.segments;
      this.positions[3 * k] = this.r * Math.cos(a);
      this.positions[3 * k + 1] = this.r * Math.sin(a);
      this.positions[3 * k + 2] = 0;
    }
    const attr = this.line.geometry.getAttribute('position') as THREE.BufferAttribute;
    attr.needsUpdate = true;
    const end = start + sweep;
    this.head.position.set(this.r * Math.cos(end), this.r * Math.sin(end), 0);
    this.head.rotation.z = end + (f > 0 ? 0 : Math.PI);
  }
}

/** Fading polyline of the most recent positions of a point. */
export class Trail extends THREE.Line {
  private readonly pos: Float32Array;
  private filled = 0;

  constructor(
    color: number,
    private readonly capacity = 400,
  ) {
    const pos = new Float32Array(capacity * 3);
    const col = new Float32Array(capacity * 4);
    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.BufferAttribute(pos, 3));
    geo.setAttribute('color', new THREE.BufferAttribute(col, 4));
    const c = new THREE.Color(color);
    for (let i = 0; i < capacity; i++) {
      const a = (i / capacity) ** 1.6;
      col.set([c.r, c.g, c.b, a * 0.9], i * 4);
    }
    super(geo, new THREE.LineBasicMaterial({ vertexColors: true, transparent: true, depthWrite: false }));
    this.pos = pos;
    this.frustumCulled = false;
  }

  push(x: number, y: number): void {
    // newest point always sits at the end of the buffer so it lines up with the opaque end of the ramp
    this.pos.copyWithin(0, 3);
    const i = (this.capacity - 1) * 3;
    this.pos[i] = x;
    this.pos[i + 1] = y;
    this.pos[i + 2] = 0.2;
    this.filled = Math.min(this.filled + 1, this.capacity);
    this.geometry.setDrawRange(this.capacity - this.filled, this.filled);
    (this.geometry.getAttribute('position') as THREE.BufferAttribute).needsUpdate = true;
  }

  reset(): void {
    this.filled = 0;
    this.geometry.setDrawRange(0, 0);
  }
}

/** Dashed circle (workspace boundaries etc.). */
export function dashedCircle(r: number, color: number, opacity = 0.35): THREE.LineSegments {
  const pts: number[] = [];
  const n = 96;
  for (let k = 0; k < n; k += 2) {
    const a0 = (k / n) * Math.PI * 2;
    const a1 = ((k + 1) / n) * Math.PI * 2;
    pts.push(r * Math.cos(a0), r * Math.sin(a0), -0.5, r * Math.cos(a1), r * Math.sin(a1), -0.5);
  }
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.Float32BufferAttribute(pts, 3));
  return new THREE.LineSegments(g, new THREE.LineBasicMaterial({ color, transparent: true, opacity }));
}
