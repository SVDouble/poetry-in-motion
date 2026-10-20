/** Every controller's page, by the key of its spec in the core. */
import * as joint from './joint';
import { lqr, mpc } from './optimal';
import type { Page, PageContext } from './page';
import { swingUp } from './swing';
import { admittance, impedance, vmc } from './task';

export const PAGES: Record<string, (c: PageContext) => Page> = {
  passive: joint.passive,
  pd: joint.pd,
  pd_gravity: joint.pdGravity,
  computed_torque: joint.computedTorque,
  passivity: joint.passivity,
  vmc,
  impedance,
  admittance,
  lqr,
  mpc,
  swing_up: swingUp,
};

export const FAMILIES: Record<string, string> = {
  none: 'Baseline',
  joint: 'Joint space',
  energy: 'Energy shaping & passivity',
  interaction: 'Interaction',
  optimal: 'Optimal',
  underactuated: 'Underactuated',
};
