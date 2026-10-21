/** Simulation settings shared by every slot (and the view toggles), with the core flags that mirror them. */
import type { Derivation } from '../model/derivation';
import type { SimClient } from '../sim/client';
import type { Flag } from '../sim/layout';

export interface Settings {
  umax: number;
  /** Manual input of the keyboard, N·m | N. */
  key: number;
  refWn: number;
  pushK: number;
  posture: number;
  manipulability: number;
  /** Priority 2: tool orientation (natural frequency, 0 = off) and its target (rad). */
  orient: number;
  orientTarget: number;
  /** Priority 3: the elbow (natural frequency, 0 = off) and its target point. */
  elbow: number;
  elbowX: number;
  elbowY: number;
  useTool: boolean;
  coriolis: boolean;
  frames: boolean;
  trail: boolean;
  ghosts: boolean;
  /** Joint coordinates, masses and gravity on the figure. */
  labels: boolean;
  /** Show the keyboard legend on the viewport. */
  keys: boolean;
}

export const defaultSettings = (): Settings => ({
  umax: 120,
  key: 20,
  refWn: 4,
  pushK: 200,
  posture: 4,
  manipulability: 0,
  orient: 0,
  orientTarget: 0,
  elbow: 0,
  elbowX: 0,
  elbowY: 0,
  useTool: false,
  coriolis: true,
  frames: false,
  trail: true,
  ghosts: true,
  labels: true,
  keys: false,
});

export type Numeric = { [K in keyof Settings]: Settings[K] extends number ? K : never }[keyof Settings];

/** Settings mirrored by core flags. */
export const SETTING_FLAGS: [Numeric, Flag][] = [
  ['umax', 'Umax'],
  ['refWn', 'RefWn'],
  ['pushK', 'PushK'],
  ['posture', 'Posture'],
  ['manipulability', 'Manipulability'],
  ['orient', 'Orient'],
  ['orientTarget', 'OrientTarget'],
  ['elbow', 'Elbow'],
  ['elbowX', 'ElbowX'],
  ['elbowY', 'ElbowY'],
];

export function pushSettings(client: Pick<SimClient, 'flag'>, s: Settings): void {
  for (const [key, flag] of SETTING_FLAGS) client.flag(flag, s[key]);
  client.flag('Coriolis', s.coriolis ? 1 : 0);
  client.flag('UseTool', s.useTool ? 1 : 0);
}

/** Can a tool target be followed? Needs a tool and a full actuator per DOF (IK + joint control) or a task controller. */
export const toolTargets = (d: Derivation) => d.tool !== null && d.nu > 0;
