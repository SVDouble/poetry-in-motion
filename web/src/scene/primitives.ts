import { HATCH, type Ink, JOINT_R, type Pen } from './drawing';
import type { Pose } from './geometry';

// ------------------------------------------------------------------------------------------- symbols

/** A line on the ground with hatching on its left side (seen from (x0, y0) towards (x1, y1)). */
export function hatched(pen: Pen, x0: number, y0: number, x1: number, y1: number): void {
  const len = Math.hypot(x1 - x0, y1 - y0);
  if (len < 1e-9) return;
  const [tx, ty] = [(x1 - x0) / len, (y1 - y0) / len];
  const [nx, ny] = [-ty, tx];
  const step = HATCH * pen.u;
  const h = HATCH * pen.u;
  pen.begin();
  pen.move(x0, y0);
  pen.line(x1, y1);
  pen.draw({ stroke: pen.inks.ink, width: 1.4 });
  pen.begin();
  for (let s = step * 0.5; s <= len; s += step) {
    const bx = x0 + tx * s;
    const by = y0 + ty * s;
    pen.move(bx, by);
    pen.line(bx + (nx - tx) * h * 0.75, by + (ny - ty) * h * 0.75);
  }
  pen.draw({ stroke: pen.inks.ink, width: 0.8 });
}

/** Pin support: a triangle from the hinge at (x, y) towards direction `dir`, standing on hatched ground. */
export function pinSupport(pen: Pen, x: number, y: number, dir: number): void {
  const u = pen.u;
  const [cx, cy] = [Math.cos(dir), Math.sin(dir)];
  const [nx, ny] = [-cy, cx];
  const h = 15 * u;
  const w = 9 * u;
  pen.begin();
  pen.move(x, y);
  pen.line(x + cx * h + nx * w, y + cy * h + ny * w);
  pen.line(x + cx * h - nx * w, y + cy * h - ny * w);
  pen.close();
  pen.draw({ stroke: pen.inks.ink, fill: pen.inks.paper, width: 1.4 });
  const b = 15 * u;
  hatched(pen, x + cx * h - nx * b, y + cy * h - ny * b, x + cx * h + nx * b, y + cy * h + ny * b);
}

export function circle(pen: Pen, x: number, y: number, r: number, ink: Ink): void {
  pen.begin();
  pen.arc(x, y, r, 0, 2 * Math.PI);
  pen.close();
  pen.draw(ink);
}

/** A hinge: an open circle; a driven one (a motor in the joint) gets a second, concentric ring. */
export function hinge(pen: Pen, x: number, y: number, ink = pen.inks.ink, driven = false): void {
  if (driven) circle(pen, x, y, (JOINT_R + 3.2) * pen.u, { stroke: ink, fill: pen.inks.paper, width: 1 });
  circle(pen, x, y, JOINT_R * pen.u, { stroke: ink, fill: pen.inks.paper, width: 1.5 });
}

/** The centre-of-mass symbol: a circle with two opposite quadrants filled. */
export function comSymbol(pen: Pen, x: number, y: number, p: number): void {
  const r = 5 * pen.u;
  circle(pen, x, y, r, { stroke: pen.inks.ink, fill: pen.inks.paper, width: 1 });
  for (const a of [p, p + Math.PI]) {
    pen.begin();
    pen.move(x, y);
    pen.arc(x, y, r, a, a + Math.PI / 2);
    pen.close();
    pen.draw({ fill: pen.inks.ink });
  }
}

/** Arrow head at (x, y) pointing along `dir`. */
export function head(pen: Pen, x: number, y: number, dir: number, ink: string, size = 7): void {
  const s = size * pen.u;
  const [c, sn] = [Math.cos(dir), Math.sin(dir)];
  pen.begin();
  pen.move(x, y);
  pen.line(x - c * s - sn * s * 0.38, y - sn * s + c * s * 0.38);
  pen.line(x - c * s + sn * s * 0.38, y - sn * s - c * s * 0.38);
  pen.close();
  pen.draw({ fill: ink });
}

export function arrow(
  pen: Pen,
  x0: number,
  y0: number,
  x1: number,
  y1: number,
  ink: string,
  width = 1.2,
): void {
  const dir = Math.atan2(y1 - y0, x1 - x0);
  const back = 5 * pen.u;
  pen.begin();
  pen.move(x0, y0);
  pen.line(x1 - Math.cos(dir) * back, y1 - Math.sin(dir) * back);
  pen.draw({ stroke: ink, width });
  head(pen, x1, y1, dir, ink);
}

/** An arc arrow around (x, y) from angle a0 to a1, head at a1. */
export function arcArrow(
  pen: Pen,
  x: number,
  y: number,
  r: number,
  a0: number,
  a1: number,
  ink: string,
): void {
  if (Math.abs(a1 - a0) * r < 6 * pen.u) return;
  const s = Math.sign(a1 - a0);
  const trim = (6 * pen.u) / r;
  pen.begin();
  pen.arc(x, y, r, a0, a1 - s * trim);
  pen.draw({ stroke: ink, width: 1.2 });
  head(pen, x + r * Math.cos(a1), y + r * Math.sin(a1), a1 + (s * Math.PI) / 2, ink);
}

/** A dashpot from (x0, y0) to (x1, y1): a cylinder on the first end, a piston rod from the second whose
 * plate stays inside the cylinder; the cylinder is sized by the reference length `ref`, so the rod visibly
 * slides in and out. */
export function dashpot(
  pen: Pen,
  x0: number,
  y0: number,
  x1: number,
  y1: number,
  ref: number,
  ink: string,
): void {
  const len = Math.hypot(x1 - x0, y1 - y0);
  if (len < 1e-6) return;
  const [tx, ty] = [(x1 - x0) / len, (y1 - y0) / len];
  const [nx, ny] = [-ty, tx];
  const w = 6 * pen.u;
  const size = Math.min(ref, len);
  const lead = 0.15 * size;
  const cyl = 0.5 * size;
  const at = (s: number, o = 0): [number, number] => [x0 + tx * s + nx * o, y0 + ty * s + ny * o];
  pen.begin();
  pen.move(x0, y0);
  pen.line(...at(lead));
  // the cylinder: a back wall and two sides, open towards the piston
  pen.move(...at(lead + cyl, w));
  pen.line(...at(lead, w));
  pen.line(...at(lead, -w));
  pen.line(...at(lead + cyl, -w));
  // the piston plate and its rod to the second end
  const p = lead + 0.65 * cyl;
  pen.move(...at(p, w * 0.7));
  pen.line(...at(p, -w * 0.7));
  pen.move(...at(p));
  pen.line(x1, y1);
  pen.draw({ stroke: ink, width: 1.2 });
}

/** A zigzag spring from (x0, y0) to (x1, y1) with straight ends. */
export function spring(pen: Pen, x0: number, y0: number, x1: number, y1: number, ink: string): void {
  const len = Math.hypot(x1 - x0, y1 - y0);
  if (len < 1e-6) return;
  const [tx, ty] = [(x1 - x0) / len, (y1 - y0) / len];
  const [nx, ny] = [-ty, tx];
  const end = Math.min(len * 0.15, 10 * pen.u);
  const body = len - 2 * end;
  const coils = Math.max(4, Math.min(14, Math.round(body / (9 * pen.u))));
  const amp = 6 * pen.u;
  pen.begin();
  pen.move(x0, y0);
  pen.line(x0 + tx * end, y0 + ty * end);
  for (let k = 0; k < coils; k++) {
    const s = end + (body * (k + 0.5)) / coils;
    const side = k % 2 ? -1 : 1;
    pen.line(x0 + tx * s + nx * amp * side, y0 + ty * s + ny * amp * side);
  }
  pen.line(x1 - tx * end, y1 - ty * end);
  pen.line(x1, y1);
  pen.draw({ stroke: ink, width: 1.2 });
}

export const wrap = (a: number) => Math.atan2(Math.sin(a), Math.cos(a));

// ------------------------------------------------------------------------------------------- view

/** A coordinate frame: x and y axes as arrows from (x, y), turned by p. */
/** The line of a bar of length l from its frame's origin along its x-axis (into the current path). */
export function bar(pen: Pen, [x, y, p]: Pose, l: number): void {
  pen.move(x, y);
  pen.line(x + Math.cos(p) * l, y + Math.sin(p) * l);
}

export function triad(pen: Pen, x: number, y: number, p: number, size = 26): void {
  const s = size * pen.u;
  arrow(pen, x, y, x + Math.cos(p) * s, y + Math.sin(p) * s, pen.inks.frame, 1);
  arrow(pen, x, y, x - Math.sin(p) * s * 0.8, y + Math.cos(p) * s * 0.8, pen.inks.frame, 1);
}

/** An arrow of length `len` (world) at (x, y) along (dx, dy), labelled at its head. */
export function vector(
  pen: Pen,
  label: (id: string, x: number, y: number, src: string) => void,
  id: string,
  [x, y]: [number, number],
  [dx, dy]: [number, number],
  scale: number,
  ink: string,
  src: string,
  dash?: readonly number[],
): void {
  const n = Math.hypot(dx, dy);
  if (n * scale < 4 * pen.u) return;
  const [ex, ey] = [x + dx * scale, y + dy * scale];
  if (dash) {
    pen.begin();
    pen.move(x, y);
    pen.line(ex, ey);
    pen.draw({ stroke: ink, width: 1.3, dash });
    head(pen, ex, ey, Math.atan2(dy, dx), ink);
  } else arrow(pen, x, y, ex, ey, ink, 1.6);
  if (src) label(id, ex + (dx / n) * 12 * pen.u, ey + (dy / n) * 12 * pen.u, src);
}
