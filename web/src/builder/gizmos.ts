/**
 * Direct manipulation in the builder viewport (an SVG layer over the canvas):
 *
 * - **ports** on the selected body (origin, middle, tip): click to attach a child or pin a loop there;
 * - **joint handle** (diamond): drag to move a root joint on the ground, or slide a joint along its parent;
 * - **tip handle** (ring): drag to rotate the body about its joint and set its length;
 * - **connect mode**: after "pin loop", every port of every other body is shown; click one, or empty ground.
 */
import type { Mechanism } from '../model/description';
import type { RobotView } from '../scene/robot';
import type { Stage } from '../scene/stage';
import type { Snapshot } from '../sim/layout';

const SVG = 'http://www.w3.org/2000/svg';
export const PORTS = [0, 0.5, 1];

export interface GizmoEvents {
  port(body: number, s: number, screen: [number, number]): void;
  /** Connect mode: a port was chosen (body ≥ 0), or a ground point (body = −1, world coordinates). */
  connect(body: number, s: number, world: [number, number]): void;
  dragStart(): void;
  dragJoint(world: [number, number], free: boolean): void;
  /** `initial`: Shift held — set the initial joint angle instead of the mounting angle. */
  dragTip(world: [number, number], free: boolean, initial: boolean): void;
  dragEnd(): void;
}

function el<K extends keyof SVGElementTagNameMap>(
  tag: K,
  attrs: Record<string, string | number>,
): SVGElementTagNameMap[K] {
  const e = document.createElementNS(SVG, tag);
  for (const [k, v] of Object.entries(attrs)) e.setAttribute(k, String(v));
  return e;
}

export class Gizmos {
  readonly svg: SVGSVGElement;
  private selected = -1;
  /** Port the loop starts from, while in connect mode. */
  connectFrom: { body: number; s: number } | null = null;
  private dragging: 'joint' | 'tip' | null = null;
  private signature = '';

  constructor(
    host: HTMLElement,
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
    window.addEventListener('pointermove', (e) => {
      if (!this.dragging) return;
      const w = this.stage.toWorld(e.clientX, e.clientY);
      if (this.dragging === 'joint') this.events.dragJoint(w, e.altKey);
      else this.events.dragTip(w, e.altKey, e.shiftKey);
    });
    window.addEventListener('pointerup', () => {
      if (!this.dragging) return;
      this.dragging = null;
      this.svg.classList.remove('dragging');
      this.events.dragEnd();
    });
  }

  select(i: number): void {
    this.selected = i;
  }

  set visible(on: boolean) {
    this.svg.style.display = on ? '' : 'none';
  }

  /** Redraw at the current pose. Cheap: a handful of SVG nodes. */
  update(mech: Mechanism, robot: RobotView, snap: Snapshot, params: number[]): void {
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
        ? 'drag: move the joint on the ground (Alt: no snapping)'
        : 'drag: slide the joint along its parent';
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
      'drag: rotate about the joint and set the length (snaps to 15° / 5 cm; Alt: free; Shift: set the initial angle θ(0) instead)';
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
