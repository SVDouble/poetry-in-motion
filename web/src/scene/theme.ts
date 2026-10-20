/** Colours shared by the 3D scene, the plots and the equations (mirrors the CSS custom properties). */
export const palette = {
  inertial: 0x8b7cff,
  coriolis: 0xffb547,
  gravity: 0x3ad6a6,
  input: 0xff6b8b,
  axisX: 0xff5f6d,
  axisY: 0x4cd97b,
  steel: 0xc9d1e0,
  graphite: 0x2a3142,
  accent: 0x6cc4ff,
  reference: 0x6cc4ff,
  warn: 0xff8a4c,
} as const;

export const css = (c: number): string => `#${c.toString(16).padStart(6, '0')}`;
