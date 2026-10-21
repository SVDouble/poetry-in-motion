/** Inks shared by the schematic, the plots and the equations (mirrors the CSS custom properties). */
export const palette = {
  ink: 0x16181d,
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

export const css = (c: number): string => `#${c.toString(16).padStart(6, '0')}`;
