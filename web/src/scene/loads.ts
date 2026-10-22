import { isDescendant, type Mechanism } from '../model/description';
import type { Snapshot } from '../sim/layout';
import { param, RobotView } from './robot';

/** A force at a point of a body (world coordinates, N). */
export interface PointForce {
  body: number;
  x: number;
  y: number;
  fx: number;
  fy: number;
  label: string;
}

/** Force balance in world metres, seconds, kilograms and newtons, independent of any drawing backend. */
export class Loads {
  private readonly geo: RobotView;
  constructor(private readonly mech: Mechanism) {
    this.geo = new RobotView(mech);
  }
  /** Forces at points of bodies, other than weights and joint reactions: springs and dampers, loop
   * closures (their multipliers λ), the push on the tool. */
  pointForces(snap: Snapshot, params: number[]): PointForce[] {
    const out: PointForce[] = [];
    const nb = this.mech.bodies.length;
    this.mech.elements.forEach((e, k) => {
      const [[ax, ay], [bx, by]] = this.geo.elementEnds(k, snap, params);
      const len = Math.hypot(bx - ax, by - ay);
      if (len < 1e-9) return;
      // tension pulls the ends together
      const t = snap.v('ELEMENT_F', k);
      const [ux, uy] = [(bx - ax) / len, (by - ay) / len];
      const name = `F_{${k + 1}}`;
      out.push({ body: e.b, x: bx, y: by, fx: -t * ux, fy: -t * uy, label: name });
      if (e.a >= 0) out.push({ body: e.a, x: ax, y: ay, fx: t * ux, fy: t * uy, label: name });
    });
    // −Aᵀλ: a pin φ = p_a − p_b pushes point a with −λ and point b with +λ; a slot φ = nᵀ(p_b − p_a)
    // pushes point b with −λ n and body a with +λ n where b touches it (at p_b: that is the moment the
    // turning of n puts on a)
    let row = 0;
    this.mech.closures.forEach((c, k) => {
      const [ax, ay] = this.geo.anchor(c.a, c.sa, [c.gx, c.gy], snap, params);
      const [bx, by] = this.geo.port(c.b, c.sb, snap, params);
      const name = `\\lambda_{${k + 1}}`;
      const add = (body: number, x: number, y: number, fx: number, fy: number) => {
        if (body >= 0) out.push({ body, x, y, fx, fy, label: name });
      };
      if (c.kind === 'pin') {
        const [lx, ly] = [snap.v('LAMBDA', row), snap.v('LAMBDA', row + 1)];
        add(c.a, ax, ay, -lx, -ly);
        add(c.b, bx, by, lx, ly);
        row += 2;
      } else {
        const dir = (c.a < 0 ? 0 : snap.frame(c.a + 1)[2]) + c.angle;
        const [nx, ny] = [-Math.sin(dir), Math.cos(dir)];
        const l = snap.v('LAMBDA', row);
        add(c.b, bx, by, -l * nx, -l * ny);
        add(c.a, bx, by, l * nx, l * ny);
        row += 1;
      }
    });
    if (this.mech.tool >= 0) {
      const [fx, fy] = [snap.v('FEXT'), snap.v('FEXT', 1)];
      const [tx, ty] = snap.frame(nb + 1);
      if (Math.hypot(fx, fy) > 1e-9)
        out.push({ body: this.mech.tool, x: tx, y: ty, fx, fy, label: 'F_{ext}' });
    }
    return out;
  }

  /** Force the parent (or the ground) exerts on body i at its joint: Newton for the subtree hanging from
   * it, R = Σ m (a_c − g) − (the other forces on the subtree from outside it). */
  reaction(i: number, snap: Snapshot, params: number[], forces: PointForce[]): [number, number] {
    const g = params[params.length - 1] ?? 9.81;
    let [rx, ry] = [0, 0];
    this.mech.bodies.forEach((_, k) => {
      if (!isDescendant(this.mech, k, i)) return;
      const m = param(params, k, 'm') ?? 0;
      rx += m * snap.v('ACOM', 2 * k);
      ry += m * (snap.v('ACOM', 2 * k + 1) + g);
    });
    for (const f of forces) {
      if (!isDescendant(this.mech, f.body, i)) continue;
      rx -= f.fx;
      ry -= f.fy;
    }
    return [rx, ry];
  }

  /** Moment the parent (or the ground) exerts on body i at its joint, about the joint: the angular-momentum
   * balance of the subtree it carries, Σ (I α + (c − j) × m (a_c − g)) − Σ (x − j) × F over the outside forces. */
  heldMoment(i: number, snap: Snapshot, params: number[], forces: PointForce[]): number {
    const g = params[params.length - 1] ?? 9.81;
    const [jx, jy] = snap.frame(i + 1);
    const cross = (x: number, y: number, fx: number, fy: number) => (x - jx) * fy - (y - jy) * fx;
    let m = 0;
    this.mech.bodies.forEach((_, k) => {
      if (!isDescendant(this.mech, k, i)) return;
      const mass = param(params, k, 'm') ?? 0;
      const [cx, cy] = this.geo.com(k, snap, params);
      m += (param(params, k, 'I') ?? 0) * snap.v('ALPHA', k);
      m += cross(cx, cy, mass * snap.v('ACOM', 2 * k), mass * (snap.v('ACOM', 2 * k + 1) + g));
    });
    for (const f of forces) if (isDescendant(this.mech, f.body, i)) m -= cross(f.x, f.y, f.fx, f.fy);
    return m;
  }
}
