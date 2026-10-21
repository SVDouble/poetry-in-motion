/**
 * Inks shared by the figure, the plots and the equations (mirrors the CSS custom properties). Colours are
 * named by their light (ink-on-paper) value; `ink` gives the one to draw with in the current theme.
 */
export const palette = {
  ink: 0x16181d,
  paper: 0xfbfaf6,
  muted: 0x8a8f99,
  frame: 0x45556f,
  inertial: 0x4b3a9e,
  coriolis: 0xb5651d,
  gravity: 0x1f7a52,
  input: 0xb8322a,
  accent: 0x1d5fae,
  reference: 0x1d5fae,
  warn: 0xc26a12,
} as const;

/** One ink per simulation slot: the primary, then the shadows. */
export const SLOT_COLORS = [palette.accent, 0xb07a12, 0x1f8a68, 0xb83a63, 0x5d48c4] as const;

/** Further plot inks. */
export const OLIVE = 0x6b7a1f;
export const PLUM = 0x9a2f7a;

/** Chalk on a blackboard: the same inks, lightened for a dark background. */
const DARK = new Map<number, number>([
  [palette.ink, 0xf4f2ec],
  [palette.paper, 0x050506],
  [palette.muted, 0x9b988f],
  [palette.frame, 0xa9bfe0],
  [palette.inertial, 0xb9adff],
  [palette.coriolis, 0xffb46b],
  [palette.gravity, 0x6fe0a8],
  [palette.input, 0xff8f82],
  [palette.accent, 0x8cc0ff],
  [palette.warn, 0xffb35c],
  [0xb07a12, 0xf0c45a],
  [0x1f8a68, 0x66e0b8],
  [0xb83a63, 0xff8fb4],
  [0x5d48c4, 0xc0b3ff],
  [OLIVE, 0xc6d873],
  [PLUM, 0xf099d8],
]);

let darkTheme = false;

export const isDark = (): boolean => darkTheme;

export function setDark(on: boolean): void {
  darkTheme = on;
}

export const css = (c: number): string => `#${c.toString(16).padStart(6, '0')}`;

/** The colour to draw ink `c` with in the current theme. */
export const ink = (c: number): string => css(darkTheme ? (DARK.get(c) ?? c) : c);
