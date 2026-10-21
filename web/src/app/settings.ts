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
  /** Dimension lines (bar lengths) on the figure. */
  dims: boolean;
  /** Velocity arrows; forces (weights, springs, loops, push); a free-body diagram of a clicked body. */
  velocities: boolean;
  forces: boolean;
  freeBody: boolean;
  /** Per-joint bars of M q̈, C q̇, N against τ; the energy gauge. */
  torqueBars: boolean;
  energyGauge: boolean;
  /** Show the keyboard legend on the viewport. */
  keys: boolean;
  /** Tuning of the controllers and the simulation (see [`TUNING`]): the same for every mechanism. */
  nullDamping: number;
  dlsEps: number;
  dlsMax: number;
  dlsLowerEps: number;
  dlsLowerMax: number;
  fadeLo: number;
  fadeHi: number;
  reachMargin: number;
  ikStep: number;
  ikDamping: number;
  pushMax: number;
  stopK: number;
  stopC: number;
  stopDepth: number;
  baumgarte: number;
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
  dims: false,
  velocities: false,
  forces: false,
  freeBody: false,
  torqueBars: false,
  energyGauge: false,
  keys: false,
  nullDamping: 8,
  dlsEps: 0.02,
  dlsMax: 0.01,
  dlsLowerEps: 0.1,
  dlsLowerMax: 0.05,
  fadeLo: 1e-4,
  fadeHi: 1e-3,
  reachMargin: 0.005,
  ikStep: 0.05,
  ikDamping: 0.01,
  pushMax: 30,
  stopK: 2e4,
  stopC: 300,
  stopDepth: 0.01,
  baumgarte: 20,
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
  ['nullDamping', 'NullDamping'],
  ['dlsEps', 'DlsEps'],
  ['dlsMax', 'DlsMax'],
  ['dlsLowerEps', 'DlsLowerEps'],
  ['dlsLowerMax', 'DlsLowerMax'],
  ['fadeLo', 'FadeLo'],
  ['fadeHi', 'FadeHi'],
  ['reachMargin', 'ReachMargin'],
  ['ikStep', 'IkStep'],
  ['ikDamping', 'IkDamping'],
  ['pushMax', 'PushMax'],
  ['stopK', 'StopK'],
  ['stopC', 'StopC'],
  ['stopDepth', 'StopDepth'],
  ['baumgarte', 'Baumgarte'],
];

export interface TuningSpec {
  key: Numeric;
  /** TeX label, slider range (always logarithmic: these are scales) and an explanation. */
  label: string;
  min: number;
  max: number;
  unit?: string;
  hint: string;
}

/**
 * Every number that shapes how the controllers and the simulation behave beyond the physics, as a setting:
 * no mechanism gets its own. `task` shapes the task-space controllers and the IK, `world` the contacts,
 * pushes and loops.
 */
export const TUNING: Record<'task' | 'world', TuningSpec[]> = {
  task: [
    {
      key: 'nullDamping',
      label: String.raw`d_{\text{null}}`,
      min: 0.5,
      max: 50,
      unit: '1/s',
      hint: 'joint damping in what the tool task does not serve: its null space, and the directions a singular pose lets go of',
    },
    {
      key: 'dlsEps',
      label: String.raw`\varepsilon_{1}`,
      min: 1e-3,
      max: 0.5,
      hint: 'tool level: below this relative size an eigen-direction is damped (damped least squares)',
    },
    {
      key: 'dlsMax',
      label: String.raw`\delta_{1}`,
      min: 1e-3,
      max: 0.5,
      hint: 'tool level: the damping reached at a singular direction',
    },
    {
      key: 'dlsLowerEps',
      label: String.raw`\varepsilon_{k}`,
      min: 1e-3,
      max: 0.5,
      hint: 'levels below the tool: where damping starts',
    },
    {
      key: 'dlsLowerMax',
      label: String.raw`\delta_{k}`,
      min: 1e-3,
      max: 0.5,
      hint: 'levels below the tool: the damping at a singular direction',
    },
    {
      key: 'fadeLo',
      label: String.raw`r_{\text{drop}}`,
      min: 1e-6,
      max: 1e-2,
      hint: 'a direction a level has used up below this relative size is dropped from the null space of the levels below',
    },
    {
      key: 'fadeHi',
      label: String.raw`r_{\text{keep}}`,
      min: 1e-5,
      max: 0.1,
      hint: 'above this relative size it is kept out of the levels below entirely (in between: a smooth fade)',
    },
    {
      key: 'reachMargin',
      label: String.raw`\rho_{\text{reach}}`,
      min: 1e-4,
      max: 0.1,
      hint: 'an unreachable target is aimed this fraction of the arm’s size inside the workspace, away from the singular boundary',
    },
    {
      key: 'ikStep',
      label: String.raw`\Delta q_{\max}`,
      min: 0.005,
      max: 0.5,
      unit: 'rad | m',
      hint: 'largest change of a coordinate per inverse-kinematics iteration',
    },
    {
      key: 'ikDamping',
      label: String.raw`\mu_{\text{IK}}`,
      min: 1e-4,
      max: 1,
      hint: 'least Levenberg–Marquardt damping of the inverse kinematics',
    },
  ],
  world: [
    {
      key: 'pushMax',
      label: String.raw`F_{\text{push}}^{\max}`,
      min: 1,
      max: 500,
      unit: 'N',
      hint: 'the Shift-drag push is capped at this force (a hand, not a winch)',
    },
    {
      key: 'stopK',
      label: String.raw`k_{\text{stop}}`,
      min: 100,
      max: 1e6,
      unit: 'N/m',
      hint: 'stiffness of the end stops of limited prismatic joints',
    },
    {
      key: 'stopC',
      label: String.raw`c_{\text{stop}}`,
      min: 1,
      max: 1e4,
      unit: 'N·s/m',
      hint: 'damping of the end stops at the typical penetration',
    },
    {
      key: 'stopDepth',
      label: String.raw`\delta_{0}`,
      min: 1e-4,
      max: 0.1,
      unit: 'm',
      hint: 'the typical penetration of an end stop (where its damping is the value above)',
    },
    {
      key: 'baumgarte',
      label: String.raw`\alpha_{\text{B}}`,
      min: 1,
      max: 200,
      unit: '1/s',
      hint: 'Baumgarte rate: how fast a loop that drifts open is pulled closed again',
    },
  ],
};

/** Change a numeric setting and the core flag that mirrors it (if any). */
export function setSetting(client: Pick<SimClient, 'flag'>, s: Settings, key: Numeric, v: number): void {
  s[key] = v;
  const flag = SETTING_FLAGS.find(([k]) => k === key)?.[1];
  if (flag) client.flag(flag, v);
}

export function pushSettings(client: Pick<SimClient, 'flag'>, s: Settings): void {
  for (const [key, flag] of SETTING_FLAGS) client.flag(flag, s[key]);
  client.flag('Coriolis', s.coriolis ? 1 : 0);
  client.flag('UseTool', s.useTool ? 1 : 0);
}

/** Can a tool target be followed? Needs a tool and a full actuator per DOF (IK + joint control) or a task controller. */
export const toolTargets = (d: Derivation) => d.tool !== null && d.nu > 0;
