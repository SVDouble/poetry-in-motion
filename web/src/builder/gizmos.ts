/**
 * Direct manipulation in the builder viewport (an SVG layer over the figure):
 *
 * - **ports** on the selected body (origin, middle, tip): click to attach a child or pin a loop there;
 * - **joint handle** (diamond): drag to move a root joint on the ground, or slide a joint along its parent —
 *   dropped onto a port of another body, the body is re-hung there;
 * - **tip handle** (ring): drag to rotate the body about its joint and set its length — dropped onto a port
 *   of another body, the tip is pinned there (a loop);
 * - **placing** a part from the palette: every port lights up, the nearest one within reach is taken, empty
 *   space means the ground; parts with two ends (springs, dampers, loops) take a second point;
 * - **connect mode**: after "pin loop", every port of every other body is shown; click one, or empty ground.
 */
import { isDescendant, type Mechanism } from '../model/description';
import type { RobotView } from '../scene/robot';
import type { Stage } from '../scene/stage';
import type { Snapshot } from '../sim/layout';
import { type Anchor, type PartKind, partSpec } from './parts';

const SVG = 'http://www.w3.org/2000/svg';
export const PORTS = [0, 0.5, 1];
/** How close (CSS pixels) a dragged point must come to a port to snap to it. */
const REACH = 22;

export interface GizmoEvents {
  port(body: number, s: number, screen: [number, number]): void;
  /** Connect mode: a port was chosen (body ≥ 0), or a ground point (body = −1, world coordinates). */
  connect(body: number, s: number, world: [number, number]): void;
  dragStart(): void;
  dragJoint(world: [number, number], free: boolean): void;
  /** `initial`: Shift held — set the initial joint angle instead of the mounting angle. */
  dragTip(world: [number, number], free: boolean, initial: boolean): void;
  dragEnd(): void;
  /** The tip of `body` was dropped onto a port: pin it there. */
  snapTip(body: number, to: Anchor): void;
  /** The joint of `body` was dropped onto a port: hang the body there. */
  snapJoint(body: number, to: Anchor): void;
  /** A part from the palette was placed (the second anchor for two-ended parts). */
  place(kind: PartKind, a: Anchor, b?: Anchor): void;
  /** Placing was cancelled, or needs its second end (for the hint line). */
  placing(state: 'first' | 'second' | 'done'): void;
}

function el<K extends keyof SVGElementTagNameMap>(
  tag: K,
  attrs: Record<string, string | number>,
): SVGElementTagNameMap[K] {
  const e = document.createElementNS(SVG, tag);
  for (const [k, v] of Object.entries(attrs)) e.setAttribute(k, String(v));
  return e;
}

interface Scene {
  mech: Mechanism;
  robot: RobotView;
  snap: Snapshot;
  params: number[];
}

export class Gizmos {
  readonly svg: SVGSVGElement;
  private selected = -1;
  /** Port the loop starts from, while in connect mode. */
  connectFrom: { body: number; s: number } | null = null;
  private dragging: 'joint' | 'tip' | null = null;
  private signature = '';
  private scene: Scene | null = null;
  /** The pointer (CSS pixels in the viewport) and the port it would snap to. */
  private pointer: [number, number] | null = null;
  private hover: Anchor | null = null;
  private placing: { kind: PartKind; from: Anchor | null } | null = null;

  constructor(
    private readonly host: HTMLElement,
    private readonly stage: Stage,
    private readonly events: GizmoEvents,
  ) {
    this.svg = el('svg', { class: 'gizmos' });
    host.append(this.svg);
    this.svg.addEventListener('pointerdown', (e) => {
      if (this.connectFrom && e.target === this.svg) {
        e.stopPropagation();
        const [x, y] = this.stage.toWorld(e.clientX, e.clientY);
        this.events.connect(-1, 0, [x, y]);
      }
    });
    // placing: a press in the view is the drop (for parts clicked in the palette); swallow it so the app
    // does not also select
    host.addEventListener(
      'pointerdown',
      (e) => {
        if (!this.placing) return;
        if (e.target instanceof Element && e.target.closest('.palette, .inspector, .overlay')) return;
        e.stopPropagation();
        this.track(e);
      },
      true,
    );
    window.addEventListener('pointermove', (e) => {
      if (this.placing) {
        this.track(e);
        return;
      }
      if (!this.dragging) return;
      const w = this.stage.toWorld(e.clientX, e.clientY);
      if (this.dragging === 'joint') this.events.dragJoint(w, e.altKey);
      else this.events.dragTip(w, e.altKey, e.shiftKey);
      this.track(e);
    });
    window.addEventListener('pointerup', (e) => {
      if (this.placing) {
        if (this.inside(e)) this.drop(e);
        return;
      }
      if (!this.dragging) return;
      const what = this.dragging;
      this.dragging = null;
      this.svg.classList.remove('dragging');
      const to = this.hover;
      this.hover = null;
      this.pointer = null;
      this.events.dragEnd();
      if (to && this.selected >= 0) {
        if (what === 'tip') this.events.snapTip(this.selected, to);
        else this.events.snapJoint(this.selected, to);
      }
    });
    window.addEventListener('keydown', (e) => {
      if (e.key === 'Escape' && this.placing) this.cancel();
    });
  }

  select(i: number): void {
    this.selected = i;
  }

  set visible(on: boolean) {
    this.svg.style.display = on ? '' : 'none';
    if (!on) this.cancel();
  }

  /** Start placing a part from the palette. */
  beginPlace(kind: PartKind): void {
    this.placing = { kind, from: null };
    this.connectFrom = null;
    this.host.classList.add('placing');
    this.signature = '';
    this.events.placing('first');
  }

  cancel(): void {
    if (!this.placing) return;
    this.placing = null;
    this.hover = null;
    this.pointer = null;
    this.host.classList.remove('placing');
    this.signature = '';
    this.events.placing('done');
  }

  private inside(e: PointerEvent): boolean {
    const r = this.host.getBoundingClientRect();
    return e.clientX >= r.left && e.clientX <= r.right && e.clientY >= r.top && e.clientY <= r.bottom;
  }

  /** Follow the pointer and find the port it would snap to. */
  private track(e: PointerEvent): void {
    const r = this.host.getBoundingClientRect();
    this.pointer = [e.clientX - r.left, e.clientY - r.top];
    this.hover = this.nearest(this.pointer, this.dragging !== null);
    this.signature = '';
  }

  /** The port nearest to a screen point within reach (dragging: not on the dragged body's own subtree). */
  private nearest([px, py]: [number, number], dragging: boolean): Anchor | null {
    const sc = this.scene;
    if (!sc) return null;
    let best: Anchor | null = null;
    let bestD = REACH;
    sc.mech.bodies.forEach((b, k) => {
      if (dragging && this.selected >= 0) {
        // the tip snaps to anything but its own body; the joint only to bodies outside its subtree
        if (k === this.selected) return;
        if (this.dragging === 'joint' && isDescendant(sc.mech, k, this.selected)) return;
      }
      for (const s of b.l > 1e-3 ? PORTS : [0]) {
        const [x, y] = sc.robot.port(k, s, sc.snap, sc.params);
        const [sx, sy] = this.stage.toScreen(x, y);
        const d = Math.hypot(sx - px, sy - py);
        if (d < bestD) {
          bestD = d;
          best = { body: k, s, x, y };
        }
      }
    });
    return best;
  }

  private drop(e: PointerEvent): void {
    const p = this.placing;
    if (!p) return;
    this.track(e);
    const [x, y] = this.stage.toWorld(e.clientX, e.clientY);
    const at: Anchor = this.hover ?? { body: -1, s: 0, x, y };
    if (partSpec(p.kind).twoEnded && !p.from) {
      p.from = at;
      this.signature = '';
      this.events.placing('second');
      return;
    }
    const from = p.from;
    this.placing = null;
    this.hover = null;
    this.host.classList.remove('placing');
    this.signature = '';
    this.events.placing('done');
    if (from) this.events.place(p.kind, from, at);
    else this.events.place(p.kind, at);
  }

  /** Redraw at the current pose. Cheap: a handful of SVG nodes. */
  update(mech: Mechanism, robot: RobotView, snap: Snapshot, params: number[]): void {
    this.scene = { mech, robot, snap, params };
    const svg = this.svg;
    // redraw only when something visible changed (keeps hover states and saves work while paused)
    const frames = Array.from({ length: mech.bodies.length + 1 }, (_, k) =>
      snap.frame(k).map((v) => v.toFixed(4)),
    );
    const [vw, vh] = this.stage.toScreen(0, 0);
    const sig = `${this.selected}|${JSON.stringify(this.connectFrom)}|${frames.join()}|${params.join()}|${vw.toFixed(1)},${vh.toFixed(1)}`;
    if (sig === this.signature) return;
    this.signature = sig;
    svg.replaceChildren();
    svg.classList.toggle('connecting', this.connectFrom !== null);
    const i = this.selected;
    const scr = (x: number, y: number) => this.stage.toScreen(x, y);
    const allPorts = (skip = -1) => {
      mech.bodies.forEach((b, k) => {
        if (k === skip) return;
        for (const s of b.l > 1e-3 ? PORTS : [0]) {
          const [px, py] = scr(...robot.port(k, s, snap, params));
          const hot = this.hover?.body === k && this.hover.s === s;
          svg.append(
            el('circle', { cx: px, cy: py, r: hot ? 9 : 6, class: `port target${hot ? ' hot' : ''}` }),
          );
        }
      });
    };

    if (this.placing) {
      allPorts();
      const p = this.placing;
      const at = this.pointer;
      if (at) {
        const [hx, hy] = this.hover ? scr(this.hover.x, this.hover.y) : at;
        if (p.from) {
          const [fx, fy] = scr(p.from.x, p.from.y);
          svg.append(el('line', { x1: fx, y1: fy, x2: hx, y2: hy, class: 'rubber' }));
          svg.append(el('circle', { cx: fx, cy: fy, r: 5, class: 'port origin-port' }));
        }
        const label = el('text', { x: hx + 14, y: hy - 12, class: 'place-label' });
        const target = this.hover
          ? `body ${this.hover.body + 1}, ${this.hover.s === 0 ? 'origin' : this.hover.s === 1 ? 'tip' : 'middle'}`
          : 'ground';
        label.textContent = `${partSpec(p.kind).label}${p.from ? ' · second end' : ''} → ${target}`;
        svg.append(el('circle', { cx: hx, cy: hy, r: 3, class: 'drop-dot' }), label);
      }
      return;
    }

    if (this.connectFrom) {
      const from = this.connectFrom;
      const [fx, fy] = scr(...robot.port(from.body, from.s, snap, params));
      svg.append(el('circle', { cx: fx, cy: fy, r: 9, class: 'port origin-port' }));
      mech.bodies.forEach((b, k) => {
        if (k === from.body) return;
        for (const s of b.l > 1e-3 ? PORTS : [0]) {
          const [px, py] = scr(...robot.port(k, s, snap, params));
          const c = el('circle', { cx: px, cy: py, r: 7, class: 'port target' });
          c.addEventListener('pointerdown', (e) => {
            e.stopPropagation();
            this.events.connect(k, s, robot.port(k, s, snap, params));
          });
          svg.append(c);
        }
      });
      return;
    }
    if (i < 0 || i >= mech.bodies.length) return;
    // while a handle is dragged: the ports it could snap to
    if (this.dragging) {
      mech.bodies.forEach((b, k) => {
        if (k === i || (this.dragging === 'joint' && isDescendant(mech, k, i))) return;
        for (const s of b.l > 1e-3 ? PORTS : [0]) {
          const [px, py] = scr(...robot.port(k, s, snap, params));
          const hot = this.hover?.body === k && this.hover.s === s;
          svg.append(
            el('circle', { cx: px, cy: py, r: hot ? 9 : 5, class: `port target${hot ? ' hot' : ''}` }),
          );
        }
      });
    }
    const b = mech.bodies[i]!;
    const [jx, jy] = snap.frame(i + 1);
    const [sx, sy] = scr(jx, jy);
    const [tx, ty] = scr(...robot.port(i, 1, snap, params));
    const hasLength = b.l > 1e-3;

    // body axis guide
    if (hasLength) svg.append(el('line', { x1: sx, y1: sy, x2: tx, y2: ty, class: 'axis' }));

    // ports
    for (const s of hasLength ? PORTS : [0]) {
      const [px, py] = scr(...robot.port(i, s, snap, params));
      const g = el('g', { class: 'port-group' });
      const c = el('circle', { cx: px, cy: py, r: 7, class: 'port' });
      const plus = el('text', { x: px, y: py + 4, class: 'port-plus' });
      plus.textContent = '+';
      g.append(c, plus);
      g.addEventListener('pointerdown', (e) => {
        e.stopPropagation();
        this.events.port(i, s, [px, py]);
      });
      svg.append(g);
    }

    // joint handle (diamond): offset slightly so it does not hide the origin port
    const off = 18;
    const dh = el('rect', {
      x: sx - 7 - off,
      y: sy - 7 + off,
      width: 14,
      height: 14,
      transform: `rotate(45 ${sx - off} ${sy + off})`,
      class: 'handle joint',
    });
    const jt = el('title', {});
    jt.textContent =
      b.parent < 0
        ? 'drag: move the joint on the ground (Alt: no snapping) — drop it on a port to hang the body there'
        : 'drag: slide the joint along its parent — drop it on a port of another body to hang it there';
    dh.append(jt);
    dh.addEventListener('pointerdown', (e) => this.start(e, 'joint'));
    svg.append(dh);

    // tip handle (ring) — rotate and stretch
    const tipLen = Math.max(Math.hypot(tx - sx, ty - sy), 1);
    const ux = (tx - sx) / tipLen;
    const uy = (ty - sy) / tipLen;
    const hx = hasLength ? tx + ux * 22 : sx + 40;
    const hy = hasLength ? ty + uy * 22 : sy;
    svg.append(
      el('line', { x1: hasLength ? tx : sx, y1: hasLength ? ty : sy, x2: hx, y2: hy, class: 'stem' }),
    );
    const th = el('circle', { cx: hx, cy: hy, r: 9, class: 'handle tip' });
    const tt = el('title', {});
    tt.textContent =
      'drag: rotate about the joint and set the length (snaps to 15° / 5 cm; Alt: free; Shift: set the initial angle θ(0) instead) — drop it on a port of another body to pin the tip there';
    th.append(tt);
    th.addEventListener('pointerdown', (e) => this.start(e, 'tip'));
    svg.append(th);
  }

  private start(e: PointerEvent, what: 'joint' | 'tip'): void {
    e.stopPropagation();
    e.preventDefault();
    this.dragging = what;
    this.svg.classList.add('dragging');
    this.events.dragStart();
  }
}
