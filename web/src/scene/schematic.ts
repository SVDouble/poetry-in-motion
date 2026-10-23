/**
 * Textbook view: the mechanism as a flat black-on-white schematic, in the symbols of engineering mechanics
 * texts: bars, open hinge circles, pin supports on hatched ground, carts on hatched rails, the quartered
 * centre-of-mass symbol, joint angles as arcs from dash-dotted reference lines, and a zigzag spring for the
 * push.
 *
 * Everything is drawn through a {@link Pen}: on screen into a 2D canvas,
 * for export into SVG or TikZ, so a figure of the current pose can be pasted into notes. Parts that never
 * move (ground, rails, gravity) live on a second canvas redrawn only when the view changes, and a frame is
 * only drawn when something on it changed.
 */
import type { Derivation } from '../model/derivation';
import { actuatorIndex, coordIndex, type Mechanism, normalise } from '../model/description';
import type { SlotView, Snapshot } from '../sim/layout';
import { DASH, DASHDOT, inks, LINK_W, type Pen } from './drawing';
import { BLOCK, type Bounds, compose, type Pose } from './geometry';
import { Loads } from './loads';
import { CanvasPen, SvgPen, TikzPen } from './pens';
import {
  arcArrow,
  arrow,
  bar,
  circle,
  comSymbol,
  dashpot,
  hatched,
  head,
  hinge,
  pinSupport,
  spring,
  triad,
  vector,
  wrap,
} from './primitives';
import { param, RobotView } from './robot';
import type { Mapping } from './stage';
import { ink, SLOT_COLORS } from './theme';

export interface SchematicFrame {
  snap: Snapshot;
  params: number[];
  running: boolean;
  /** Pointer the tool is pushed towards, or null. */
  push: [number, number] | null;
  /** Shadow slots to draw (empty when hidden). */
  ghosts: readonly SlotView[];
  /** Joint coordinates, masses and the gravity symbol. */
  annotate: boolean;
  /** The coordinate frames {s}, {1} … {n} and {t}. */
  frames: boolean;
  /** Dimension lines: the length of every bar. */
  dims?: boolean;
  /** Velocity arrows at the centres of mass (and the tool). */
  velocities?: boolean;
  /** Forces: weights, springs and dampers, loop closures, the push. */
  forces?: boolean;
  /** Free-body diagram of this body (−1: none): it is cut free and every force on it is drawn. */
  freeBody?: number;
  /** What an equation being pointed at is about. */
  focus?: Focus | null;
  /** The tool's path (of every slot shown). */
  trail: boolean;
}

const TRAIL = 240;

/** What an equation is about, to light up in the figure (see `data-focus` in the panels). */
export type Focus =
  | { kind: 'body' | 'frame' | 'com' | 'element' | 'loop' | 'joint'; index: number }
  | { kind: 'gravity' | 'velocities' | 'tool' | 'all' };

/** Parse a `data-focus` attribute: "body:2", "gravity", … */
export function parseFocus(s: string | undefined): Focus | null {
  if (!s) return null;
  const [kind, idx] = s.split(':');
  if (kind === 'gravity' || kind === 'velocities' || kind === 'tool' || kind === 'all') return { kind };
  if (['body', 'frame', 'com', 'element', 'loop', 'joint'].includes(kind!) && idx !== undefined)
    return { kind: kind as 'body', index: Number(idx) };
  return null;
}

/** A ring buffer of recent tool positions. */
class Path {
  readonly xy = new Float64Array(2 * TRAIL);
  len = 0;
  head = 0;

  reset(): void {
    this.len = 0;
    this.head = 0;
  }

  /** Points closer than 1 mm to the last one are skipped. */
  push(x: number, y: number): void {
    if (this.len > 0) {
      const last = (this.head - 1 + TRAIL) % TRAIL;
      const dx = x - this.xy[2 * last]!;
      const dy = y - this.xy[2 * last + 1]!;
      if (dx * dx + dy * dy < 1e-6) return;
    }
    this.xy[2 * this.head] = x;
    this.xy[2 * this.head + 1] = y;
    this.head = (this.head + 1) % TRAIL;
    this.len = Math.min(TRAIL, this.len + 1);
  }

  draw(pen: Pen, ink: string, dash: readonly number[]): void {
    if (this.len < 2) return;
    pen.begin();
    const first = (this.head - this.len + TRAIL) % TRAIL;
    for (let k = 0; k < this.len; k++) {
      const j = (first + k) % TRAIL;
      if (k === 0) pen.move(this.xy[2 * j]!, this.xy[2 * j + 1]!);
      else pen.line(this.xy[2 * j]!, this.xy[2 * j + 1]!);
    }
    pen.draw({ stroke: ink, width: 1, dash });
  }
}

/** The picture itself, independent of the screen: painting, the tool's path, and the exports. */
export class Figure {
  private readonly coord: number[];
  private readonly actuator: number[];
  /** Points, lengths and joint frames of the bodies at a pose. */
  private readonly geo: RobotView;
  private readonly loads: Loads;
  /** Tool paths: the primary, then the shadow slots. */
  private readonly paths = SLOT_COLORS.map(() => new Path());
  /** Where an export puts the gravity arrow (null: the corner of the reachable region). */
  private gAt: [number, number] | null = null;
  /** On screen: the gravity arrow sits in a corner of the view, wherever the view is (null: unknown). */
  screenG: [number, number] | null = null;
  /** On screen: where the arrow scales are listed (the bottom-left corner of the view; null: unknown). */
  screenKey: [number, number] | null = null;
  /** Highlighted body and spring/damper (−1: none). */
  selected = -1;
  selectedElement = -1;
  /** Label boxes of the picture being painted (world x0, y0, x1, y1, …): a label that would overlap is left out. */
  private readonly boxes: number[] = [];

  constructor(
    readonly mech: Mechanism,
    private readonly d: Derivation,
    private readonly bounds: Bounds,
  ) {
    this.coord = coordIndex(mech);
    this.actuator = actuatorIndex(mech);
    this.geo = new RobotView(mech);
    this.loads = new Loads(mech);
  }

  resetTrail(): void {
    for (const p of this.paths) p.reset();
  }

  /** Record the tool position of slot k (0: the primary). */
  track(x: number, y: number, k = 0): void {
    this.paths[k]?.push(x, y);
  }

  /** The current picture as a standalone SVG (screen pixels). */
  svg(m: Mapping, f: SchematicFrame): string {
    const pen = new SvgPen(m.k, m.x0, m.y0);
    this.paintExport(pen, f);
    const [x0, y0, x1, y1] = pen.box.map((v, i) => (i < 2 ? Math.floor(v - 10) : Math.ceil(v + 10)));
    const [w, h] = [x1! - x0!, y1! - y0!];
    return [
      `<svg xmlns="http://www.w3.org/2000/svg" width="${w}" height="${h}" viewBox="${x0} ${y0} ${w} ${h}">`,
      `<rect x="${x0}" y="${y0}" width="${w}" height="${h}" fill="#ffffff"/>`,
      ...pen.out,
      '</svg>',
    ].join('\n');
  }

  /** The current picture as a TikZ figure about 12 cm wide. */
  tikz(m: Mapping, f: SchematicFrame): string {
    const scale = 12 / Math.max(this.bounds.width, 1e-3);
    const pen = new TikzPen(1 / m.k, scale);
    this.paintExport(pen, f);
    return [
      `% ${this.mech.name} at t = ${f.snap.t.toFixed(2)} s — exported from Poetry in Motion`,
      '% needs \\usepackage{tikz}',
      ...pen.preamble(),
      '\\begin{tikzpicture}[line cap=round, line join=round]',
      ...pen.out,
      '\\end{tikzpicture}',
    ].join('\n');
  }

  /** The whole picture for a file: no shadow controllers, and gravity drawn beside the mechanism as it
   * stands (on screen it sits in a corner of the region the mechanism can reach). */
  private paintExport(pen: Pen, f: SchematicFrame): void {
    let [maxX, maxY] = [-Infinity, -Infinity];
    for (let i = 0; i < this.mech.bodies.length; i++) {
      const [x, y, p] = f.snap.frame(i + 1);
      const l = this.geo.length(i, f.params);
      maxX = Math.max(maxX, x, x + Math.cos(p) * l);
      maxY = Math.max(maxY, y, y + Math.sin(p) * l);
    }
    this.gAt = Number.isFinite(maxX) ? [maxX + 0.35, maxY] : null;
    this.paintStatic(pen, f.annotate);
    this.paintDynamic(pen, { ...f, ghosts: [] });
    this.gAt = null;
  }

  // ----------------------------------------------------------------------------------- static layer

  /** Ground supports, rails on the ground, ground pins and slots, gravity. */
  paintStatic(pen: Pen, annotate: boolean): void {
    const u = pen.u;
    for (const [x, y] of this.mech.anchors ?? []) {
      if (
        this.mech.bodies.some(
          (b) => b.parent < 0 && !b.detached && Math.hypot(b.mountX - x, b.mountY - y) < 1e-8,
        )
      )
        continue;
      hatched(pen, x + 10 * u, y - 4 * u, x - 10 * u, y - 4 * u);
      circle(pen, x, y, 3 * u, { stroke: pen.inks.ink, fill: pen.inks.paper, width: 1.2 });
    }
    this.mech.bodies.forEach((raw) => {
      if (raw.parent >= 0 || raw.detached) return;
      const b = normalise(raw);
      if (b.joint === 'prismatic') {
        const a = b.mountAngle + b.axisAngle;
        const [c, s] = [Math.cos(a), Math.sin(a)];
        const half = (b.limit > 0 ? b.limit : 2) + (b.shape === 'block' ? BLOCK.w / 2 : 0.05);
        // the rail under the wheels (or through the slider), hatched below
        const off = b.shape === 'block' ? -(BLOCK.h / 2 + 0.06) : -7 * u;
        const [ox, oy] = [b.mountX - s * off, b.mountY + c * off];
        hatched(pen, ox + c * half, oy + s * half, ox - c * half, oy - s * half);
        if (b.limit > 0) {
          for (const sgn of [-1, 1]) {
            const ex = ox + c * sgn * half;
            const ey = oy + s * sgn * half;
            pen.begin();
            pen.move(ex, ey);
            pen.line(ex - s * 16 * u, ey + c * 16 * u);
            pen.draw({ stroke: pen.inks.ink, width: 2.6 });
          }
        }
      } else if (b.joint === 'revolute') {
        pinSupport(pen, b.mountX, b.mountY, b.mountAngle + Math.PI);
      } else {
        // welded to the ground: a hatched wall behind the root
        const dir = b.mountAngle + Math.PI;
        const [c, s] = [Math.cos(dir), Math.sin(dir)];
        const w = 16 * u;
        hatched(pen, b.mountX - s * w, b.mountY + c * w, b.mountX + s * w, b.mountY - c * w);
      }
    });
    for (const c of this.mech.closures) {
      if (c.a >= 0) continue;
      if (c.kind === 'pin') pinSupport(pen, c.gx, c.gy, -Math.PI / 2);
      else this.slotGuide(pen, c.gx, c.gy, c.angle, true);
    }
    if (annotate && this.mech.g > 0) {
      const [bx, by] = this.gravityAt();
      arrow(pen, bx, by, bx, by - 44 * u, pen.inks.ink, 1.3);
    }
  }

  /** The strips that ground rails and slots (near horizontal or vertical) cover, as label boxes: a label
   * lands beside the hatching, not on it. */
  private groundStrips(u: number): number[] {
    const out: number[] = [];
    const strip = (x: number, y: number, angle: number, half: number, thick: number) => {
      const [c, s] = [Math.abs(Math.cos(angle)), Math.abs(Math.sin(angle))];
      if (Math.min(c, s) > 0.26) return; // a slanted strip's box would cover too much
      const [hx, hy] = c > s ? [half, thick] : [thick, half];
      out.push(x - hx, y - hy, x + hx, y + hy);
    };
    for (const raw of this.mech.bodies) {
      const b = normalise(raw);
      if (raw.parent >= 0 || b.joint !== 'prismatic') continue;
      const a = b.mountAngle + b.axisAngle;
      const half = (b.limit > 0 ? b.limit : 2) + 0.1;
      const off = b.shape === 'block' ? -(BLOCK.h / 2 + 0.06) : -7 * u;
      strip(b.mountX - Math.sin(a) * off, b.mountY + Math.cos(a) * off, a, half, 7 * u);
    }
    for (const c of this.mech.closures)
      if (c.a < 0 && c.kind !== 'pin') strip(c.gx, c.gy, c.angle, 1.4, 13 * u);
    return out;
  }

  /** Where the gravity arrow starts (its label is drawn with the moving layer, where labels live). */
  private gravityAt(): [number, number] {
    if (this.gAt) return this.gAt;
    if (this.screenG) return this.screenG;
    return [this.bounds.cx + this.bounds.width / 2 - 0.3, this.bounds.cy - this.bounds.height / 2 + 0.6];
  }

  /** A slot: two parallel guide lines along `angle` through (x, y), hatched outside when on the ground. */
  private slotGuide(pen: Pen, x: number, y: number, angle: number, ground: boolean): void {
    const u = pen.u;
    const [c, s] = [Math.cos(angle), Math.sin(angle)];
    const half = 1.4;
    const gap = 7 * u;
    for (const side of [-1, 1]) {
      const ox = x - s * gap * side;
      const oy = y + c * gap * side;
      if (ground && side < 0) hatched(pen, ox + c * half, oy + s * half, ox - c * half, oy - s * half);
      else if (ground) hatched(pen, ox - c * half, oy - s * half, ox + c * half, oy + s * half);
      else {
        pen.begin();
        pen.move(ox - c * half, oy - s * half);
        pen.line(ox + c * half, oy + s * half);
        pen.draw({ stroke: pen.inks.ink, width: 1.2 });
      }
    }
  }

  // ----------------------------------------------------------------------------------- dynamic layer

  /** A label unless it would overlap one placed before it in this picture. */
  private label(
    pen: Pen,
    id: string,
    x: number,
    y: number,
    src: string,
    dx = 0,
    dy = 0,
    /** Better places than the generic nudges, tried first (world points). */
    alternatives: readonly [number, number][] = [],
  ): void {
    const glyphs = src.replace(/\\[a-zA-Z]+/g, 'x').replace(/[{}_^\s]/g, '').length;
    const w = (4.5 * glyphs + 2) * pen.u;
    const h = 9 * pen.u;
    const b = this.boxes;
    const free = (cx: number, cy: number) => {
      for (let k = 0; k < b.length; k += 4) {
        if (cx - w < b[k + 2]! && cx + w > b[k]! && cy - h < b[k + 3]! && cy + h > b[k + 1]!) return false;
      }
      return true;
    };
    // where it belongs, else nudged just clear (above, below, right, left); dropped only when all are taken
    const step = (2 * h) / pen.u + 2;
    const sideways = (2 * w) / pen.u + 2;
    for (const [ax, ay] of alternatives) {
      if (!free(ax + dx * pen.u, ay - dy * pen.u)) continue;
      x = ax;
      y = ay;
      break;
    }
    for (const [ox, oy] of [
      [0, 0],
      [0, step],
      [0, -step],
      [sideways, 0],
      [-sideways, 0],
    ] as const) {
      const cx = x + (dx + ox) * pen.u;
      const cy = y - (dy + oy) * pen.u;
      if (!free(cx, cy)) continue;
      b.push(cx - w, cy - h, cx + w, cy + h);
      pen.label(id, x, y, src, dx + ox, dy + oy);
      return;
    }
  }

  /** Velocities, forces, a free-body diagram and what an equation points at. */
  private paintOverlays(pen: Pen, f: SchematicFrame): void {
    const { snap, params } = f;
    const u = pen.u;
    const nb = this.mech.bodies.length;
    const g = params[params.length - 1] ?? 9.81;
    const label = (id: string, x: number, y: number, src: string) => this.label(pen, id, x, y, src);
    const com = (k: number) => this.geo.com(k, snap, params);
    const moving = (k: number) => this.mech.bodies[k]!.joint !== 'fixed' || (param(params, k, 'm') ?? 0) > 0;
    // fixed scales, so that arrows compare across time: a round force (on the order of the heaviest weight) and a
    // round speed (about the size of the mechanism per second) are each drawn L long, listed in a key
    const size = Math.max(this.bounds.width, this.bounds.height);
    const L = 0.12 * size;
    const weight = Math.max(1e-3, ...this.mech.bodies.map((_, k) => (param(params, k, 'm') ?? 0) * g));
    // (the reference is at least half the heaviest weight: a weight is drawn about L, never past 2 L)
    const [fRef, vRef] = [roundDown(2 * weight), roundDown(0.5 * size)];
    const [fScale, vScale] = [L / fRef, L / vRef];
    const focus = f.focus ?? null;
    const showVel = f.velocities || focus?.kind === 'velocities' || focus?.kind === 'com';
    const showGrav = f.forces || focus?.kind === 'gravity';
    const showForce = showGrav || (f.freeBody ?? -1) >= 0;
    if (showVel || showForce) {
      const [kx, ky] = this.screenKey ?? [
        this.bounds.cx - this.bounds.width / 2 + 0.1,
        this.bounds.cy - this.bounds.height / 2 + 0.1,
      ];
      const rows: [string, string, string][] = [];
      if (showForce) rows.push([pen.inks.ink, String.raw`${fmt(fRef)}\,\mathrm{N}`, 'kf']);
      if (showVel) rows.push([pen.inks.accent, String.raw`${fmt(vRef)}\,\mathrm{m/s}`, 'kv']);
      rows.forEach(([ink, src, id], r) => {
        const y = ky + r * 20 * u;
        arrow(pen, kx, y, kx + L, y, ink, 1.6);
        // labels are centred on their point: clear of the arrowhead by half their width
        label(id, kx + L + (id === 'kv' ? 34 : 24) * u, y, src);
      });
    }

    if (showVel) {
      const vs = this.mech.bodies.map((_, k): [number, number] => [
        snap.v('VCOM', 2 * k),
        snap.v('VCOM', 2 * k + 1),
      ]);
      vs.forEach((v, k) => {
        if (!moving(k) || (focus?.kind === 'com' && focus.index !== k)) return;
        vector(pen, label, `v${k}`, com(k), v, vScale, pen.inks.accent, `v_{c${k + 1}}`);
      });
      // pointing at a centre of mass: the columns of its Jacobian, v_c = Σ_j (∂p_c/∂q_j) q̇_j — how fast
      // it moves per unit rate of each joint (drawn even at rest, where v_c is zero)
      if (focus?.kind === 'com' && focus.index < nb) {
        const k = focus.index;
        const maxn = snap.layout.maxn;
        this.d.coords.forEach((c, j) => {
          const col: [number, number] = [
            snap.v('JCOM', 2 * k * maxn + j),
            snap.v('JCOM', (2 * k + 1) * maxn + j),
          ];
          const src = String.raw`\partial_{${c.tex}}\, p_{c${k + 1}}`;
          vector(pen, label, `j${j}`, com(k), col, vScale, pen.inks.frame, src, [4, 3]);
        });
      }
    }
    const forces = f.forces || f.freeBody !== undefined ? this.loads.pointForces(snap, params) : [];
    if (showGrav || f.forces) {
      const weights = this.mech.bodies.map((_, k): [number, number] => [
        0,
        -(param(params, k, 'm') ?? 0) * g,
      ]);
      weights.forEach((w, k) => {
        if (moving(k)) vector(pen, label, `w${k}`, com(k), w, fScale, pen.inks.gravity, `m_{${k + 1}}g`);
      });
      if (f.forces) {
        forces.forEach((p, k) => {
          vector(pen, label, `pf${k}`, [p.x, p.y], [p.fx, p.fy], fScale, pen.inks.input, p.label);
        });
      }
    }

    // free-body diagram: the body cut free with every force on it, and m a_c as the resultant
    const i = f.freeBody ?? -1;
    if (i >= 0 && i < nb) {
      const m = param(params, i, 'm') ?? 0;
      const [jx, jy] = snap.frame(i + 1);
      // cut free: the rest of the mechanism fades, the body is drawn again on top
      pen.begin();
      pen.move(-1e3, -1e3);
      pen.line(1e3, -1e3);
      pen.line(1e3, 1e3);
      pen.line(-1e3, 1e3);
      pen.close();
      pen.draw({ fill: pen.inks.paper, alpha: 0.72 });
      const l = this.geo.length(i, params);
      const p = snap.frame(i + 1)[2];
      if (l > 1e-3) {
        pen.begin();
        bar(pen, [jx, jy, p], l);
        pen.draw({ stroke: pen.inks.ink, width: LINK_W });
      }
      comSymbol(pen, ...com(i), p);
      hinge(pen, jx, jy);
      this.mech.bodies.forEach((b, c) => {
        if (b.parent === i) hinge(pen, ...(snap.frame(c + 1).slice(0, 2) as [number, number]));
      });
      const list: { at: [number, number]; f: [number, number]; src: string; dash?: boolean }[] = [];
      list.push({ at: com(i), f: [0, -m * g], src: `m_{${i + 1}}g` });
      // the force through joint k on body k (sign +1) or on its parent (−1); a prismatic joint passes the
      // drive (and friction) along its axis, and a normal force across it: drawn apart
      const joint = (k: number, sign: number, at: [number, number]) => {
        const [rx, ry] = this.loads.reaction(k, snap, params, forces).map((v) => sign * v) as [
          number,
          number,
        ];
        const b = this.mech.bodies[k]!;
        const minus = sign < 0 ? '-' : '';
        if (b.joint !== 'prismatic') {
          list.push({ at, f: [rx, ry], src: `${minus}R_{${k + 1}}` });
          return;
        }
        const a = this.geo.jointPose(k, snap, params)[2] + b.axisAngle;
        const [ux, uy] = [Math.cos(a), Math.sin(a)];
        const along = rx * ux + ry * uy;
        list.push({ at, f: [along * ux, along * uy], src: `${minus}F_{${k + 1}}` });
        list.push({ at, f: [rx - along * ux, ry - along * uy], src: `${minus}N_{${k + 1}}` });
      };
      joint(i, 1, [jx, jy]);
      this.mech.bodies.forEach((b, c) => {
        if (b.parent === i) joint(c, -1, snap.frame(c + 1).slice(0, 2) as [number, number]);
      });
      for (const p of forces) if (p.body === i) list.push({ at: [p.x, p.y], f: [p.fx, p.fy], src: p.label });
      const ma: [number, number] = [m * snap.v('ACOM', 2 * i), m * snap.v('ACOM', 2 * i + 1)];
      list.push({ at: com(i), f: ma, src: `m_{${i + 1}}a_{${i + 1}}`, dash: true });
      list.forEach((x, k) => {
        const ink = x.dash ? pen.inks.inertial : x.src.includes('g') ? pen.inks.gravity : pen.inks.input;
        vector(pen, label, `fb${k}`, x.at, x.f, fScale, ink, x.src, x.dash ? [5, 3] : undefined);
      });
      // joint moments: the drive and friction at its own joint, their reactions at the children's
      // a hinge passes its drive less its friction; a slide or a weld passes whatever moment keeps the bodies
      // it carries turning as they do
      const moment = (k: number) => {
        const j = this.coord[k]!;
        if (this.mech.bodies[k]!.joint !== 'revolute' || j < 0)
          return this.loads.heldMoment(k, snap, params, forces);
        const a = this.actuator[k]!;
        return (a >= 0 ? snap.v('U', a) : 0) - (param(params, k, 'beta') ?? 0) * snap.v('QD', j);
      };
      const umax = snap.v('UMAX') || 1;
      // a child's joint may sit where the body's own does (a pole on a cart): its arc goes outside
      const arc = (k: number, sign: number, x: number, y: number, name: string) => {
        const mm = sign * moment(k);
        if (Math.abs(mm) < 1e-6) return;
        const span = Math.max(0.3, Math.min(1.6, (Math.abs(mm) / umax) * 6)) * Math.PI * Math.sign(mm);
        const r = (sign > 0 ? 24 : 34) * u;
        arcArrow(pen, x, y, r, Math.PI / 2, Math.PI / 2 + span, pen.inks.input);
        this.label(pen, `fm${name}`, x - r - 6 * u, y + r + 6 * u, name);
      };
      arc(i, 1, jx, jy, `M_{${i + 1}}`);
      this.mech.bodies.forEach((b, c) => {
        if (b.parent === i) {
          const [cx, cy] = snap.frame(c + 1);
          arc(c, -1, cx, cy, `-M_{${c + 1}}`);
        }
      });
    }

    // what an equation points at
    if (focus) {
      const ring = (x: number, y: number, r = 11) =>
        circle(pen, x, y, r * u, { stroke: pen.inks.accent, width: 2 });
      const glow = (k: number) => {
        pen.begin();
        bar(pen, snap.frame(k + 1), this.geo.length(k, params));
        pen.draw({ stroke: pen.inks.accent, width: LINK_W + 3, alpha: 0.45 });
      };
      if (focus.kind === 'all') {
        for (let k = 0; k < nb; k++) glow(k);
      }
      if (focus.kind === 'body' && focus.index < nb) glow(focus.index);
      if (focus.kind === 'frame' && focus.index < nb) {
        glow(focus.index);
        const [x, y, p] = snap.frame(focus.index + 1);
        triad(pen, x, y, p, 34);
      }
      if (focus.kind === 'com' && focus.index < nb) ring(...com(focus.index));
      if (focus.kind === 'joint') {
        const k = this.mech.bodies.findIndex((_, b) => this.coord[b] === focus.index);
        if (k >= 0) ring(snap.frame(k + 1)[0], snap.frame(k + 1)[1], 13);
      }
      if (focus.kind === 'tool' && this.mech.tool >= 0) {
        const [tx, ty, tp] = snap.frame(nb + 1);
        ring(tx, ty);
        triad(pen, tx, ty, tp, 28);
      }
      if (focus.kind === 'loop') {
        const c = this.mech.closures[focus.index];
        if (c) ring(...this.geo.port(c.b, c.sb, snap, params), 14);
      }
      if (focus.kind === 'element') {
        const e = this.mech.elements[focus.index];
        if (e) {
          const [[ax, ay], [bx, by]] = this.geo.elementEnds(focus.index, snap, params);
          pen.begin();
          pen.move(ax, ay);
          pen.line(bx, by);
          pen.draw({ stroke: pen.inks.accent, width: 8, alpha: 0.3 });
        }
      }
    }
  }

  paintDynamic(pen: Pen, f: SchematicFrame): void {
    const { snap, params } = f;
    this.boxes.length = 0;
    this.boxes.push(...this.groundStrips(pen.u));
    const u = pen.u;
    const nb = this.mech.bodies.length;

    // tool paths and shadow controllers underneath everything
    if (f.running && f.trail && this.mech.tool >= 0) {
      this.paths[0]!.draw(pen, pen.inks.muted, [2, 3]);
      f.ghosts.forEach((slot, k) => {
        if (slot.on) this.paths[k + 1]?.draw(pen, pen.inks.ghosts[k % pen.inks.ghosts.length]!, [1, 3]);
      });
    }
    f.ghosts.forEach((slot, k) => {
      if (!slot.on) return;
      const ink = pen.inks.ghosts[k % pen.inks.ghosts.length]!;
      pen.begin();
      for (let i = 0; i < nb; i++) {
        bar(pen, slot.frame(i + 1), this.geo.length(i, params));
      }
      pen.draw({ stroke: ink, width: 2.2, alpha: 0.55 });
      if (this.mech.tool >= 0)
        circle(pen, slot.v('TOOL'), slot.v('TOOL', 1), 3 * u, { fill: ink, alpha: 0.8 });
    });

    // prismatic guides on moving bodies, and slots that move with a body
    this.mech.bodies.forEach((b, i) => {
      if (b.joint !== 'prismatic' || b.parent < 0) return;
      const [jx, jy, jp] = this.geo.jointPose(i, snap, params);
      const a = jp + b.axisAngle;
      const half = b.limit > 0 ? b.limit + 0.05 : 0.8;
      const [c, s] = [Math.cos(a), Math.sin(a)];
      pen.begin();
      pen.move(jx - c * half, jy - s * half);
      pen.line(jx + c * half, jy + s * half);
      pen.draw({ stroke: pen.inks.ink, width: 1, dash: DASH });
    });
    this.mech.closures.forEach((c) => {
      if (c.kind !== 'slot' || c.a < 0) return;
      const [ax, ay] = this.geo.port(c.a, c.sa, snap, params);
      this.slotGuide(pen, ax, ay, snap.frame(c.a + 1)[2] + c.angle, false);
    });

    // springs and dampers: a zigzag, a dashpot, or both side by side between end bars
    this.mech.elements.forEach((e, k) => {
      const [[ax, ay], [bx, by]] = this.geo.elementEnds(k, snap, params);
      const len = Math.hypot(bx - ax, by - ay);
      if (len < 1e-6) return;
      const ink = k === this.selectedElement ? pen.inks.accent : pen.inks.ink;
      const [tx, ty] = [(bx - ax) / len, (by - ay) / len];
      const [nx, ny] = [-ty, tx];
      const ref = e.l0 > 0.05 ? e.l0 : 0.4;
      if (e.a < 0) {
        // anchored to the ground: a hatched wall across the spring's line, like the textbook figure
        const w = 16 * u;
        hatched(pen, ax - nx * w, ay - ny * w, ax + nx * w, ay + ny * w);
      }
      if (e.k > 0 && e.d > 0) {
        const o = 9 * u;
        const end = Math.min(8 * u, 0.15 * len);
        const [a2x, a2y, b2x, b2y] = [ax + tx * end, ay + ty * end, bx - tx * end, by - ty * end];
        pen.begin();
        pen.move(ax, ay);
        pen.line(a2x, a2y);
        pen.move(a2x + nx * o, a2y + ny * o);
        pen.line(a2x - nx * o, a2y - ny * o);
        pen.move(bx, by);
        pen.line(b2x, b2y);
        pen.move(b2x + nx * o, b2y + ny * o);
        pen.line(b2x - nx * o, b2y - ny * o);
        pen.draw({ stroke: ink, width: 1.2 });
        spring(pen, a2x + nx * o, a2y + ny * o, b2x + nx * o, b2y + ny * o, ink);
        dashpot(pen, a2x - nx * o, a2y - ny * o, b2x - nx * o, b2y - ny * o, ref, ink);
      } else if (e.k > 0) spring(pen, ax, ay, bx, by, ink);
      else dashpot(pen, ax, ay, bx, by, ref, ink);
      circle(pen, ax, ay, 2.6 * u, { stroke: ink, fill: pen.inks.paper, width: 1.2 });
      circle(pen, bx, by, 2.6 * u, { stroke: ink, fill: pen.inks.paper, width: 1.2 });
      if (f.annotate) {
        const name =
          e.k > 0 && e.d > 0
            ? String.raw`k_{${k + 1}},\,d_{${k + 1}}`
            : e.k > 0
              ? `k_{${k + 1}}`
              : `d_{${k + 1}}`;
        const off = (e.k > 0 && e.d > 0 ? 24 : 16) * u;
        this.label(pen, `e${k}`, (ax + bx) / 2 + nx * off, (ay + by) / 2 + ny * off, name);
      }
    });

    // bodies: blocks, then bars
    this.mech.bodies.forEach((raw, i) => {
      const b = normalise(raw);
      const [x, y, p] = snap.frame(i + 1);
      const ink = i === this.selected ? pen.inks.accent : pen.inks.ink;
      if (b.shape === 'block') {
        const [c, s] = [Math.cos(p), Math.sin(p)];
        const corner = (dx: number, dy: number): [number, number] => [
          x + c * dx - s * dy,
          y + s * dx + c * dy,
        ];
        pen.begin();
        const pts = [corner(-BLOCK.w / 2, -BLOCK.h / 2), corner(BLOCK.w / 2, -BLOCK.h / 2)];
        pts.push(corner(BLOCK.w / 2, BLOCK.h / 2), corner(-BLOCK.w / 2, BLOCK.h / 2));
        pen.move(...pts[0]!);
        for (const q of pts.slice(1)) pen.line(...q);
        pen.close();
        pen.draw({ stroke: ink, fill: pen.inks.paper, width: 1.8 });
      }
      const l = this.geo.length(i, params);
      if (l > 1e-3) {
        pen.begin();
        bar(pen, [x, y, p], l);
        pen.draw({ stroke: ink, width: b.joint === 'fixed' ? LINK_W + 0.8 : LINK_W });
      }
      if (b.joint === 'prismatic' && b.shape !== 'block') {
        // a sleeve on the guide
        const [jx, jy, jp] = this.geo.jointPose(i, snap, params);
        const a = jp + b.axisAngle;
        const [c, s] = [Math.cos(a), Math.sin(a)];
        const along = (x - jx) * c + (y - jy) * s;
        const [cx, cy] = [jx + c * along, jy + s * along];
        const [hw, hh] = [11 * u, 6 * u];
        pen.begin();
        pen.move(cx - c * hw + s * hh, cy - s * hw - c * hh);
        pen.line(cx + c * hw + s * hh, cy + s * hw - c * hh);
        pen.line(cx + c * hw - s * hh, cy + s * hw + c * hh);
        pen.line(cx - c * hw - s * hh, cy - s * hw + c * hh);
        pen.close();
        pen.draw({ stroke: ink, fill: pen.inks.paper, width: 1.5 });
      }
    });

    if (f.annotate && this.mech.g > 0) {
      const [bx, by] = this.gravityAt();
      this.label(pen, 'g', bx, by - 22 * u, 'g', 12, 0);
    }

    // joints: hinges, welds, loop pins
    this.mech.bodies.forEach((b, i) => {
      if (b.detached) return;
      const [x, y, p] = snap.frame(i + 1);
      const ink = i === this.selected ? pen.inks.accent : pen.inks.ink;
      if (b.joint === 'revolute') hinge(pen, x, y, ink, b.actuated);
      else if (b.joint === 'fixed') {
        const r = 4 * u;
        const [c, s] = [Math.cos(p + Math.PI / 4), Math.sin(p + Math.PI / 4)];
        pen.begin();
        pen.move(x + c * r, y + s * r);
        pen.line(x - s * r, y + c * r);
        pen.line(x - c * r, y - s * r);
        pen.line(x + s * r, y - c * r);
        pen.close();
        pen.draw({ fill: ink });
      }
    });
    this.mech.closures.forEach((c) => {
      const [px, py] = this.geo.port(c.b, c.sb, snap, params);
      if (c.kind === 'slot') {
        const dir = (c.a < 0 ? 0 : snap.frame(c.a + 1)[2]) + c.angle;
        const [cs, sn] = [Math.cos(dir), Math.sin(dir)];
        const [hw, hh] = [9 * u, 5.5 * u];
        pen.begin();
        pen.move(px - cs * hw + sn * hh, py - sn * hw - cs * hh);
        pen.line(px + cs * hw + sn * hh, py + sn * hw - cs * hh);
        pen.line(px + cs * hw - sn * hh, py + sn * hw + cs * hh);
        pen.line(px - cs * hw - sn * hh, py - sn * hw + cs * hh);
        pen.close();
        pen.draw({ stroke: pen.inks.ink, fill: pen.inks.paper, width: 1.4 });
      }
      hinge(pen, px, py);
    });

    // joint coordinates: an arc from a dash-dotted reference line, or a dimension along the rail
    if (f.annotate) {
      this.mech.bodies.forEach((b, i) => {
        const j = this.coord[i]!;
        const info = j >= 0 ? this.d.coords[j] : undefined;
        if (!info) return;
        const [x, y, p] = snap.frame(i + 1);
        const [jx, jy, jp] = this.geo.jointPose(i, snap, params);
        if (b.joint === 'revolute') {
          const ref = jp;
          const q = wrap(p - ref);
          const r = 30 * u;
          pen.begin();
          pen.move(x, y);
          pen.line(x + Math.cos(ref) * 46 * u, y + Math.sin(ref) * 46 * u);
          pen.draw({ stroke: pen.inks.ink, width: 0.8, dash: DASHDOT });
          arcArrow(pen, x, y, r, ref, ref + q, pen.inks.ink);
          const mid = ref + q / 2 + (Math.abs(q) < 0.35 ? 0.35 * Math.sign(q || 1) : 0);
          this.label(
            pen,
            `q${j}`,
            x + Math.cos(mid) * (r + 12 * u),
            y + Math.sin(mid) * (r + 12 * u),
            info.tex,
          );
        } else {
          const a = jp + b.axisAngle;
          const [c, s] = [Math.cos(a), Math.sin(a)];
          const along = (x - jx) * c + (y - jy) * s;
          // on the side of the rail away from the bodies that hang off the slider
          let side = 1;
          this.mech.bodies.forEach((child, k) => {
            if (child.parent === i && Math.sin(snap.frame(k + 1)[2] - a) > 0.3) side = -1;
          });
          const clear = b.shape === 'block' ? BLOCK.h / 2 + (side < 0 ? 0.08 : 0.03) : 0;
          const off = side * (clear + 22 * u);
          const [ox, oy] = [-s * off, c * off];
          pen.begin();
          pen.move(jx + ox * 0.6, jy + oy * 0.6);
          pen.line(jx + ox * 1.25, jy + oy * 1.25);
          pen.draw({ stroke: pen.inks.ink, width: 0.8 });
          if (Math.abs(along) > 8 * u) {
            arrow(pen, jx + ox, jy + oy, jx + c * along + ox, jy + s * along + oy, pen.inks.ink, 0.9);
          }
          this.label(pen, `q${j}`, jx + c * along * 0.5 + ox, jy + s * along * 0.5 + oy, info.tex, 0, -10);
        }
      });
    }

    // dimension lines: the length lᵢ of every bar, beside it on its counter-clockwise side
    if (f.dims) {
      this.mech.bodies.forEach((_, i) => {
        const l = this.geo.length(i, params);
        if (l < 1e-3) return;
        const [x, y, p] = snap.frame(i + 1);
        const [c, sn] = [Math.cos(p), Math.sin(p)];
        const [nx, ny] = [-sn, c];
        const off = 20 * u;
        const [ax, ay, bx, by] = [x + nx * off, y + ny * off, x + c * l + nx * off, y + sn * l + ny * off];
        pen.begin();
        for (const [ex, ey] of [
          [x, y],
          [x + c * l, y + sn * l],
        ] as const) {
          pen.move(ex + nx * 6 * u, ey + ny * 6 * u);
          pen.line(ex + nx * (off + 5 * u), ey + ny * (off + 5 * u));
        }
        pen.draw({ stroke: pen.inks.muted, width: 0.7 });
        pen.begin();
        pen.move(ax, ay);
        pen.line(bx, by);
        pen.draw({ stroke: pen.inks.ink, width: 0.8 });
        head(pen, bx, by, p, pen.inks.ink, 6);
        head(pen, ax, ay, p + Math.PI, pen.inks.ink, 6);
        this.label(pen, `l${i}`, (ax + bx) / 2 + nx * 10 * u, (ay + by) / 2 + ny * 10 * u, `l_{${i + 1}}`);
      });
    }

    // centres of mass (labelled after the coordinates, which win where labels would overlap)
    if (f.annotate) {
      this.mech.bodies.forEach((raw, i) => {
        const b = normalise(raw);
        if (b.joint === 'fixed' && b.m <= 0) return;
        const p = snap.frame(i + 1)[2];
        const [cx, cy] = this.geo.com(i, snap, params);
        if (b.shape === 'block') {
          // a cart: the label beside it, clear of whatever stands on it
          const off = BLOCK.w / 2 + 16 * u;
          this.label(pen, `m${i}`, cx - Math.cos(p) * off, cy - Math.sin(p) * off, `m_{${i + 1}}`);
          return;
        }
        comSymbol(pen, cx, cy, p);
        // on the clockwise side of the bar (joint angles open counter-clockwise), else the other side
        const [sx, sy] = [Math.sin(p) * 15 * u, -Math.cos(p) * 15 * u];
        this.label(pen, `m${i}`, cx + sx, cy + sy, `m_{${i + 1}}`, 0, 0, [
          [cx + sx, cy + sy],
          [cx - sx, cy - sy],
        ]);
      });
    }

    // actuator inputs: a torque arc at a hinge, a force arrow along a rail
    const umax = snap.v('UMAX') || 1;
    if (f.running && snap.v('KIND') !== 0) {
      this.mech.bodies.forEach((b, i) => {
        if (!b.actuated || b.joint === 'fixed') return;
        const a = this.actuator[i]!;
        if (a < 0) return;
        const v = Math.max(-1, Math.min(1, snap.v('U', a) / umax));
        if (Math.abs(v) < 0.02) return;
        const [x, y, p] = snap.frame(i + 1);
        if (b.joint === 'revolute') {
          arcArrow(pen, x, y, 17 * u, p + Math.PI / 2, p + Math.PI / 2 + v * 1.5 * Math.PI, pen.inks.input);
        } else {
          const [, , jp] = this.geo.jointPose(i, snap, params);
          const ax = jp + b.axisAngle;
          const len = 0.5 * v;
          arrow(pen, x, y, x + Math.cos(ax) * len, y + Math.sin(ax) * len, pen.inks.input, 1.6);
        }
      });
    }

    // tool, targets, push
    const [tx, ty] = snap.frame(nb + 1);
    if (this.mech.tool >= 0) circle(pen, tx, ty, 2.6 * u, { fill: pen.inks.ink });
    if (f.running && snap.v('USE_TOOL') !== 0) {
      const gx = snap.v('TARGET');
      const gy = snap.v('TARGET', 1);
      const ink = snap.v('REACHABLE') !== 0 ? pen.inks.accent : pen.inks.warn;
      circle(pen, gx, gy, 8 * u, { stroke: ink, width: 1.2 });
      pen.begin();
      pen.move(gx - 12 * u, gy);
      pen.line(gx + 12 * u, gy);
      pen.move(gx, gy - 12 * u);
      pen.line(gx, gy + 12 * u);
      pen.draw({ stroke: ink, width: 0.9 });
    }
    if (f.running && snap.v('GOALS', 2) > 0) {
      const ex = snap.v('GOALS', 3);
      const ey = snap.v('GOALS', 4);
      circle(pen, ex, ey, 7 * u, { stroke: pen.inks.accent, width: 1, dash: [3, 2] });
    }
    if (f.running && snap.v('GOALS') > 0 && this.mech.tool >= 0) {
      const a = snap.v('GOALS', 1);
      arrow(pen, tx, ty, tx + Math.cos(a) * 32 * u, ty + Math.sin(a) * 32 * u, pen.inks.accent, 1);
    }
    if (f.push) spring(pen, tx, ty, f.push[0], f.push[1], pen.inks.input);
    this.paintOverlays(pen, f);

    // joint targets on actuated rails (where the joint controllers are taking them)
    if (f.running && snap.v('KIND') !== 0) {
      this.mech.bodies.forEach((b, i) => {
        const j = this.coord[i]!;
        if (b.joint !== 'prismatic' || !b.actuated || j < 0) return;
        const [jx, jy, jp] = this.geo.jointPose(i, snap, params);
        const a = jp + b.axisAngle;
        const q = snap.v('Q_TARGET', j);
        const [px, py] = [jx + Math.cos(a) * q, jy + Math.sin(a) * q];
        const [nx, ny] = [-Math.sin(a), Math.cos(a)];
        const below = b.shape === 'block' ? BLOCK.h / 2 + 0.06 : 0;
        const [bx, by] = [px - nx * below, py - ny * below];
        const s = 6 * u;
        pen.begin();
        pen.move(bx, by);
        pen.line(bx - nx * s * 1.6 + Math.cos(a) * s, by - ny * s * 1.6 + Math.sin(a) * s);
        pen.line(bx - nx * s * 1.6 - Math.cos(a) * s, by - ny * s * 1.6 - Math.sin(a) * s);
        pen.close();
        pen.draw({ stroke: pen.inks.accent, fill: pen.inks.paper, width: 1.2 });
      });
    }

    // coordinate frames: {s} at the origin, {i} on every body, {t} at the tool
    if (f.frames) {
      triad(pen, 0, 0, 0, 32);
      this.label(pen, 'fs', 0, 0, String.raw`\{s\}`, -14, 12);
      for (let i = 0; i < nb; i++) {
        const [x, y, p] = snap.frame(i + 1);
        triad(pen, x, y, p);
        this.label(pen, `f${i}`, x, y, String.raw`\{${i + 1}\}`, -12, -12);
      }
      if (this.mech.tool >= 0) {
        const [, , tp] = snap.frame(nb + 1);
        triad(pen, tx, ty, tp, 20);
        this.label(pen, 'ft', tx, ty, String.raw`\{t\}`, 12, 12);
      }
    }
  }
}

/** The figure on screen: a canvas for the ground (redrawn when the view changes), one for everything that
 * moves (redrawn when something changed), and KaTeX labels over them. */

// ------------------------------------------------------------------------------------------- overlays

/** The largest 1, 2 or 5 × 10ᵏ not above v (a scale that reads easily). */
function roundDown(v: number): number {
  const e = 10 ** Math.floor(Math.log10(v));
  const m = v / e;
  return (m >= 5 ? 5 : m >= 2 ? 2 : 1) * e;
}

/** A scale value as TeX: no trailing zeros. */
const fmt = (v: number) => String(Number(v.toPrecision(2)));

/** A small drawing of a mechanism at its initial pose (frames from forward kinematics), fitted into
 * `canvas`: the preset gallery. No labels, no annotations, the current theme's inks. */
export function drawThumbnail(
  canvas: HTMLCanvasElement,
  mech: Mechanism,
  frames: Pose[],
  bounds: Bounds,
  [w, h]: [number, number],
): void {
  const dpr = Math.min(window.devicePixelRatio || 1, 2);
  canvas.style.width = `${w}px`;
  canvas.style.height = `${h}px`;
  canvas.width = Math.round(w * dpr);
  canvas.height = Math.round(h * dpr);
  const ctx = canvas.getContext('2d');
  if (!ctx) return;
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.lineCap = 'round';
  ctx.lineJoin = 'round';
  const k = Math.min(w / bounds.width, h / bounds.height);
  const pen = new CanvasPen(ctx, null, inks(ink));
  // symbols at 70 % of their size on screen
  pen.map(k, w / 2 - bounds.cx * k, h / 2 + bounds.cy * k);
  (pen as { u: number }).u = 0.7 / k;
  const nb = mech.bodies.length;
  const tip = (i: number): Pose => compose(frames[i]!, [normalise(mech.bodies[i]!).l, 0, 0]);
  const snap = {
    t: 0,
    frame: (j: number): Pose =>
      j === 0 ? [0, 0, 0] : j <= nb ? frames[j - 1]! : tip(Math.max(0, mech.tool)),
    v: () => 0,
  } as unknown as Snapshot;
  const params = mech.bodies.flatMap((raw) => {
    const b = normalise(raw);
    return [b.m, b.l, b.r, b.I, b.beta];
  });
  const fig = new Figure(mech, { coords: [] } as unknown as Derivation, bounds);
  fig.paintStatic(pen, false);
  fig.paintDynamic(pen, {
    snap,
    params,
    running: false,
    push: null,
    ghosts: [],
    annotate: false,
    frames: false,
    trail: false,
  });
}
