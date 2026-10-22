/** Physical admissibility and control presentation are separate: a slider is not a validator. */
export const PHYSICAL = {
  m: { unit: 'kg', min: 0, max: 50, step: 0.1 },
  l: { unit: 'm', min: 0, max: 3, step: 0.05 },
  r: { unit: 'm', min: -2, max: 3, step: 0.05 },
  I: { unit: 'kg·m²', min: 0, max: 10, step: 0.01 },
  beta: { unit: 'N·s/m | N·m·s/rad', min: 0, max: 2, step: 0.001 },
  g: { unit: 'm/s²', min: -20, max: 20, step: 0.01 },
} as const;
export type PhysicalField = keyof typeof PHYSICAL;

export function validPhysical(key: PhysicalField, value: unknown): value is number {
  return typeof value === 'number' && Number.isFinite(value) && (key === 'g' || key === 'r' || value >= 0);
}

/** Explicit ranges for compact sliders; the inspector exposes wider increments for authoring. */
const COMPACT = {
  m: { min: 0.05, max: 10 },
  l: { max: 2 },
  r: { min: -1, max: 2, step: 0.01 },
  I: { max: 2, step: 0.001 },
  beta: {},
  g: {},
} as const;
export function physicalControl(key: PhysicalField, surface: 'inspector' | 'slider' | 'runtime') {
  return { ...PHYSICAL[key], ...(surface === 'inspector' ? {} : COMPACT[key]) };
}

/** Uniform-rod dependencies apply to authored documents; runtime overrides remain independent. */
export function uniformRod(m: number, l: number) {
  return { r: l / 2, I: (m * l * l) / 12 };
}
