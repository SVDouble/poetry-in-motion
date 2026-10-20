/** Controller modes offered for a built mechanism, and the keyboard mapping of actuators. */
import { MODE } from '../eq/live';
import type { Derivation } from '../model/derivation';

export interface ModeSpec {
  id: number;
  label: string;
  hint: string;
  /** Offered when this holds (n coordinates, nu actuators, dof = n − constraints). */
  when: (n: number, nu: number, dof: number) => boolean;
}

export const MODES: ModeSpec[] = [
  {
    id: MODE.PASSIVE,
    label: 'Passive',
    hint: 'No controller: gravity, friction and your inputs only.',
    when: () => true,
  },
  { id: MODE.PD, label: 'PD', hint: 'Independent joint PD. Gravity makes it sag.', when: (_n, nu) => nu > 0 },
  {
    id: MODE.PD_GRAVITY,
    label: 'PD + gravity',
    hint: 'PD plus exact gravity compensation: no steady-state error, transients still depend on M(q).',
    when: (_n, nu) => nu > 0,
  },
  {
    id: MODE.COMPUTED_TORQUE,
    label: 'Computed torque',
    hint: 'Inverse dynamics on the constraint manifold: identical linear error dynamics everywhere.',
    when: (_n, nu, dof) => nu > 0 && nu >= dof,
  },
  {
    id: MODE.LQR,
    label: 'LQR',
    hint: 'Linear-quadratic regulator about the target q*.',
    when: (_n, nu) => nu > 0,
  },
  {
    id: MODE.SWING_UP,
    label: 'Swing-up',
    hint: 'Energy pumping through the actuated joints, then an LQR catch at q*.',
    when: (_n, nu, dof) => nu > 0 && nu < dof,
  },
];

export const dofOf = (d: Derivation) => d.n - d.phi.length;
export const availableModes = (d: Derivation) => MODES.filter((m) => m.when(d.n, d.nu, dofOf(d)));

export function defaultMode(d: Derivation): number {
  if (d.nu === 0) return MODE.PASSIVE;
  return d.nu >= dofOf(d) ? MODE.COMPUTED_TORQUE : MODE.LQR;
}

/** Key pairs (negative, positive) driving actuator 0, 1, … */
export const ACTUATOR_KEYS: [string, string][] = [
  ['arrowleft', 'arrowright'],
  ['a', 'd'],
  ['j', 'l'],
  ['u', 'o'],
  ['z', 'c'],
  ['v', 'n'],
];
export const ACTUATOR_LABELS = ['← →', 'A D', 'J L', 'U O', 'Z C', 'V N'];
