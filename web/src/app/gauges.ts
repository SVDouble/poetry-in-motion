/**
 * Gauges on the simulation view: per-joint bars of the terms of M q̈ + C q̇ + N = τ (which one dominates —
 * gravity at rest, inertia in a fast swing), and an energy gauge (what is stored, what was supplied, what
 * friction took). Drawn on a small canvas at ~30 Hz in the theme's inks.
 */
import type { Derivation } from '../model/derivation';
import { ink, palette } from '../scene/theme';
import type { Field, Snapshot } from '../sim/layout';
import type { Settings } from './settings';

const W = 268;
const ROW = 8;
const FONT = '15px KaTeX_Main, "Latin Modern Roman", serif';
const SUB = '12.5px KaTeX_Main, "Latin Modern Roman", serif';

/** A symbol with a subscript, drawn by hand (the canvas cannot typeset). */
function symbol(c: CanvasRenderingContext2D, x: number, y: number, main: string, sub: string): void {
  c.font = `italic ${FONT}`;
  c.fillText(main, x, y);
  const w = c.measureText(main).width;
  c.font = SUB;
  c.fillText(sub, x + w + 0.5, y + 3.5);
}

export class Gauges {
  readonly el = document.createElement('div');
  private readonly canvas = document.createElement('canvas');
  private d: Derivation | null = null;
  private scaleTau = 1;
  private scaleE = 1;
  private last = 0;

  constructor(private readonly settings: Settings) {
    this.el.className = 'gauges overlay';
    this.el.append(this.canvas);
    this.el.hidden = true;
  }

  load(d: Derivation): void {
    this.d = d;
    this.scaleTau = 1;
    this.scaleE = 1;
    this.apply();
  }

  /** Show or hide after a settings change or a stage change. */
  apply(visible = true): void {
    this.el.hidden = !visible || !(this.settings.torqueBars || this.settings.energyGauge);
  }

  draw(snap: Snapshot, now: number): void {
    const d = this.d;
    if (this.el.hidden || !d || now - this.last < 33) return;
    this.last = now;
    const n = d.n;
    const bars = this.settings.torqueBars;
    const energy = this.settings.energyGauge;
    const height = (bars ? 22 + n * (4 * ROW + 10) : 0) + (energy ? 24 + 6 * 19 + 42 : 0) + 8;
    const dpr = Math.min(window.devicePixelRatio || 1, 2);
    if (this.canvas.height !== Math.round(height * dpr)) {
      this.canvas.width = Math.round(W * dpr);
      this.canvas.height = Math.round(height * dpr);
      this.canvas.style.width = `${W}px`;
      this.canvas.style.height = `${height}px`;
    }
    const c = this.canvas.getContext('2d');
    if (!c) return;
    c.setTransform(dpr, 0, 0, dpr, 0, 0);
    c.clearRect(0, 0, W, height);
    const [text, muted, line] = [ink(palette.ink), ink(palette.muted), ink(palette.muted)];
    let y = 4;
    const title = (s: string) => {
      c.fillStyle = muted;
      c.font = `small-caps ${FONT}`;
      c.fillText(s, 8, y + 12);
      y += 20;
    };
    const x0 = 44;
    const span = W - x0 - 12;
    const mid = x0 + span / 2;

    if (bars) {
      title('Torques, joint by joint');
      const terms: [Field, number][] = [
        ['MQDD', palette.inertial],
        ['CQD', palette.coriolis],
        ['N', palette.gravity],
        ['TAU', palette.input],
      ];
      let big = 0;
      for (let j = 0; j < n; j++) for (const [f] of terms) big = Math.max(big, Math.abs(snap.v(f, j)));
      // the scale follows the largest term, shrinking slowly so the bars do not jump
      this.scaleTau = Math.max(big, this.scaleTau * 0.985, 1e-6);
      const k = span / 2 / this.scaleTau;
      for (let j = 0; j < n; j++) {
        c.fillStyle = text;
        const coord = d.coords[j]!;
        symbol(c, 8, y + 2 * ROW + 2, coord.kind === 'prismatic' ? 'd' : 'θ', String(coord.body + 1));
        c.strokeStyle = line;
        c.lineWidth = 0.6;
        c.beginPath();
        c.moveTo(mid, y - 1);
        c.lineTo(mid, y + 4 * ROW + 1);
        c.stroke();
        terms.forEach(([f, col], r) => {
          const v = snap.v(f, j) * k;
          c.fillStyle = ink(col);
          const tau = f === 'TAU';
          c.fillRect(Math.min(mid, mid + v), y + r * ROW + (tau ? 0 : 1), Math.abs(v), tau ? ROW : ROW - 2);
        });
        y += 4 * ROW + 10;
      }
      c.font = SUB;
      const legend: [string, number][] = [
        ['M q̈', palette.inertial],
        ['C q̇', palette.coriolis],
        ['N', palette.gravity],
        ['τ', palette.input],
      ];
      let lx = 8;
      for (const [s, col] of legend) {
        c.fillStyle = ink(col);
        c.fillRect(lx, y - 6, 10, 4);
        c.fillStyle = text;
        c.font = `italic ${FONT}`;
        c.fillText(s, lx + 13, y);
        lx += 62;
      }
      y += 10;
    }

    if (energy) {
      title('Energy');
      // names on the left, bars in the middle, values on the right
      const [bx0, bspan] = [118, W - 118 - 66];
      const bmid = bx0 + bspan / 2;
      const [ke, pe] = [snap.v('ENERGY'), snap.v('ENERGY', 1)];
      const rows: [string, number, number][] = [
        ['T  kinetic', ke, palette.inertial],
        ['V  potential', pe, palette.gravity],
        ['E = T + V', ke + pe, palette.ink],
        ['supplied by inputs', snap.v('WORK'), palette.input],
        ['by pushes and kicks', snap.v('WORK', 1), palette.accent],
        ['lost to friction', -snap.v('WORK', 3), palette.coriolis],
      ];
      const big = Math.max(...rows.map((r) => Math.abs(r[1])), 1e-6);
      this.scaleE = Math.max(big, this.scaleE * 0.985);
      const k = bspan / 2 / this.scaleE;
      for (const [name, v, col] of rows) {
        c.strokeStyle = line;
        c.lineWidth = 0.6;
        c.beginPath();
        c.moveTo(bmid, y + 2);
        c.lineTo(bmid, y + 16);
        c.stroke();
        c.fillStyle = ink(col);
        c.fillRect(Math.min(bmid, bmid + v * k), y + 5, Math.abs(v * k), 8);
        c.fillStyle = text;
        c.font = SUB;
        c.fillText(name, 8, y + 13);
        c.textAlign = 'right';
        c.fillText(`${v.toFixed(2)} J`, W - 8, y + 13);
        c.textAlign = 'left';
        y += 19;
      }
      c.fillStyle = muted;
      c.font = `italic ${SUB}`;
      c.fillText(`budget residual ${snap.v('BUDGET').toExponential(1)} J`, 8, y + 14);
      const recoveries = snap.v('RECOVERY');
      c.fillStyle = ink(recoveries > 0 ? palette.warn : palette.muted);
      c.fillText(`${recoveries} recoveries · ${snap.v('RECOVERY', 1)} loop retries`, 8, y + 32);
    }
  }
}
