import { css, palette, SLOT_COLORS } from './theme';

/** Stroke and fill of one path. Widths and dashes are in symbol units (CSS pixels on screen). */
export interface Ink {
  stroke?: string;
  fill?: string;
  width?: number;
  dash?: readonly number[];
  alpha?: number;
}

/** Drawing in world coordinates (m, y up); symbol sizes are multiples of `u` (m per CSS pixel). */
export interface Pen {
  readonly inks: Inks;
  readonly u: number;
  begin(): void;
  move(x: number, y: number): void;
  line(x: number, y: number): void;
  /** Arc from angle a0 to a1 (rad, counter-clockwise when a1 > a0), joined to the current point. */
  arc(x: number, y: number, r: number, a0: number, a1: number): void;
  close(): void;
  draw(ink: Ink): void;
  /** A TeX label at (x, y) shifted by (dx, dy) pixels (y down), centred. */
  label(id: string, x: number, y: number, src: string, dx?: number, dy?: number): void;
}

/** The inks of one picture. */
export interface Inks {
  ink: string;
  paper: string;
  muted: string;
  accent: string;
  input: string;
  warn: string;
  /** Coordinate frames. */
  frame: string;
  /** The colours of the terms of M q̈ + C q̇ + N = τ: weights are green like N, moments red like τ. */
  gravity: string;
  inertial: string;
  coriolis: string;
  /** Shadow controllers, in their slot inks. */
  ghosts: string[];
}

export const inks = (draw: (c: number) => string): Inks => ({
  ink: draw(palette.ink),
  paper: draw(palette.paper),
  muted: draw(palette.muted),
  accent: draw(palette.accent),
  input: draw(palette.input),
  warn: draw(palette.warn),
  frame: draw(palette.frame),
  gravity: draw(palette.gravity),
  inertial: draw(palette.inertial),
  coriolis: draw(palette.coriolis),
  ghosts: SLOT_COLORS.slice(1).map(draw),
});

/** Files are for paper: always ink on white. */
export const PRINT = inks(css);

export const LINK_W = 3.8;
export const THIN = 1.1;
export const JOINT_R = 5;
export const HATCH = 7;
export const DASHDOT = [10, 3, 2, 3] as const;
export const DASH = [5, 4] as const;
