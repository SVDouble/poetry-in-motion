import { LIMITS } from '../generated/contracts';
import { uniformRod, validPhysical } from './fields';

/**
 * The mechanism document edited by the builder, its presets, and its flat encoding for the wasm core
 * (`core/src/mech.rs::Desc::parse` — keep the two in sync).
 *
 * The mechanism is a graph: a spanning tree of bodies (each hanging from its parent by a revolute, prismatic
 * or fixed joint) plus loop-closing pins between points of two bodies (or a body and the ground).
 */

export type JointKind = 'revolute' | 'prismatic' | 'fixed';
export type Shape = 'link' | 'block';

export interface BodySpec {
  /** Index of the parent body, or -1 for the ground. Parents always come first. */
  parent: number;
  joint: JointKind;
  /** Joint position along the parent, as a fraction of its length: 0 = origin, 1 = tip. */
  mountS: number;
  /** Joint position for bodies on the ground (m). */
  mountX: number;
  mountY: number;
  /** Rotation of the joint frame relative to the parent frame (rad). */
  mountAngle: number;
  /** Sliding direction of a prismatic joint in its joint frame (rad). */
  axisAngle: number;
  actuated: boolean;
  m: number;
  l: number;
  r: number;
  I: number;
  beta: number;
  /** r = l/2 and I = m l²/12 (uniform slender rod). */
  uniform: boolean;
  q0: number;
  qTarget: number;
  /** Half range of a prismatic joint (end stops); 0 = unlimited. */
  limit: number;
  shape: Shape;
}

/** Loop closure between point `(sa · l_a, 0)` of body `a` (or ground point (gx, gy) if a = −1) and point
 * `(sb · l_b, 0)` of body `b`: a **pin** makes them coincide (2 constraints); a **slot** lets b slide along the
 * line through a with direction `angle` in a's frame (1 constraint). */
export interface Closure {
  kind: 'pin' | 'slot';
  angle: number;
  a: number;
  sa: number;
  b: number;
  sb: number;
  gx: number;
  gy: number;
}

/** A linear spring and/or viscous damper between point `(sa · l_a, 0)` of body `a` (or ground point (gx, gy)
 * if a = −1) and point `(sb · l_b, 0)` of body `b`: force `k (ℓ − ℓ₀) + d ℓ̇` along the line between them. */
export interface Element {
  a: number;
  sa: number;
  b: number;
  sb: number;
  gx: number;
  gy: number;
  /** Stiffness (N/m), rest length (m), damping (N·s/m). */
  k: number;
  l0: number;
  d: number;
}

export interface Mechanism {
  name: string;
  bodies: BodySpec[];
  closures: Closure[];
  elements: Element[];
  g: number;
  /** Body whose tip is the tool (end effector), or -1. */
  tool: number;
}

export const MAX_BODIES = LIMITS.maxBodies;
export const MAX_DOF = LIMITS.maxDof;
export const MAX_CLOSURES = LIMITS.maxClosures;
export const DESC_VERSION = LIMITS.version;
export const MAX_ELEMENTS = LIMITS.maxElements;
export const PER_BODY = LIMITS.perBody;
export const PARAMS_PER_BODY = LIMITS.paramsPerBody;

/** Keep r and I consistent with a uniform rod when requested. */
/** Nothing yet: the ground and gravity, to build on from scratch. */
export const emptyMechanism = (): Mechanism => ({
  name: 'Empty',
  g: 9.81,
  tool: -1,
  closures: [],
  elements: [],
  bodies: [],
});

/** Shelves of the preset gallery, in order. */
export const CATEGORIES = ['Arms', 'Pendulums', 'Carts & gantries', 'Linkages', 'Springs'] as const;
export type Category = (typeof CATEGORIES)[number];

/** Where a mechanism belongs, read off its structure (the same rule for presets and the user's builds). */
export function category(m: Mechanism): Category {
  if (m.closures.length) return 'Linkages';
  if (m.elements.length) return 'Springs';
  if (m.bodies.some((b) => b.parent < 0 && b.joint === 'prismatic')) return 'Carts & gantries';
  // an arm drives every joint between the base and its tool (a passive branch hanging off it is a load)
  const chain: BodySpec[] = [];
  for (let b = m.tool; b >= 0; b = m.bodies[b]!.parent) chain.push(m.bodies[b]!);
  const passive = (b: BodySpec) => b.joint !== 'fixed' && !b.actuated;
  return (chain.length ? chain.some(passive) : m.bodies.some(passive)) ? 'Pendulums' : 'Arms';
}

export function normalise(b: BodySpec): BodySpec {
  if (!b.uniform) return b;
  return { ...b, ...uniformRod(b.m, b.l) };
}

export const dof = (m: Mechanism) => m.bodies.filter((b) => b.joint !== 'fixed').length;

/** Coordinate index of every body (−1 for fixed joints). */
export function coordIndex(m: Mechanism): number[] {
  let j = 0;
  return m.bodies.map((b) => (b.joint === 'fixed' ? -1 : j++));
}

/** Actuator index of every body (−1 when not actuated). */
export function actuatorIndex(m: Mechanism): number[] {
  let a = 0;
  return m.bodies.map((b) => (b.joint !== 'fixed' && b.actuated ? a++ : -1));
}

export function encode(mech: Mechanism): number[] {
  const out = [
    DESC_VERSION,
    mech.bodies.length,
    mech.g,
    mech.tool,
    mech.closures.length,
    mech.elements.length,
  ];
  for (const raw of mech.bodies) {
    const b = normalise(raw);
    out.push(
      b.parent,
      b.joint === 'prismatic' ? 1 : b.joint === 'fixed' ? 2 : 0,
      b.mountS,
      b.mountX,
      b.mountY,
      b.mountAngle,
      b.axisAngle,
      b.actuated && b.joint !== 'fixed' ? 1 : 0,
      b.m,
      b.l,
      b.r,
      b.I,
      b.beta,
      b.q0,
      b.qTarget,
      b.limit,
      b.shape === 'block' ? 1 : 0,
      0,
    );
  }
  for (const c of mech.closures)
    out.push(c.kind === 'slot' ? 1 : 0, c.a, c.sa, c.b, c.sb, c.gx, c.gy, c.angle);
  for (const e of mech.elements) out.push(e.a, e.sa, e.b, e.sb, e.gx, e.gy, e.k, e.l0, e.d);
  return out;
}

/** Model parameter vector `[m₁ l₁ r₁ I₁ β₁ … g]` (per body, same order as the core). */
export function paramVector(mech: Mechanism): number[] {
  const p: number[] = [];
  for (const raw of mech.bodies) {
    const b = normalise(raw);
    p.push(b.m, b.l, b.r, b.I, b.beta);
  }
  p.push(mech.g);
  return p;
}

export function link(parent: number, joint: JointKind = 'revolute', patch: Partial<BodySpec> = {}): BodySpec {
  return normalise({
    parent,
    joint,
    mountS: parent >= 0 ? 1 : 0,
    mountX: 0,
    mountY: 0,
    mountAngle: 0,
    axisAngle: 0,
    actuated: joint !== 'fixed',
    m: 1,
    l: 1,
    r: 0.5,
    I: 1 / 12,
    beta: 0.05,
    uniform: true,
    q0: 0,
    qTarget: 0,
    limit: joint === 'prismatic' ? 2 : 0,
    shape: 'link',
    ...patch,
  });
}

const HALF_PI = Math.PI / 2;

/** A spring–damper between two points (k = 0: a damper, d = 0: a spring). */
export const spring = (
  a: number,
  sa: number,
  b: number,
  sb: number,
  patch: Partial<Element> = {},
): Element => ({
  a,
  sa,
  b,
  sb,
  gx: 0,
  gy: 0,
  k: 50,
  l0: 0.5,
  d: 2,
  ...patch,
});
export const pin = (a: number, sa: number, b: number, sb: number, gx = 0, gy = 0): Closure => ({
  kind: 'pin',
  angle: 0,
  a,
  sa,
  b,
  sb,
  gx,
  gy,
});
const cart = (patch: Partial<BodySpec> = {}) =>
  link(-1, 'prismatic', {
    m: 1,
    l: 0,
    uniform: false,
    r: 0,
    I: 0,
    beta: 0.1,
    limit: 2.4,
    shape: 'block',
    ...patch,
  });

export const PRESETS: Record<string, () => Mechanism> = {
  'Cart-pole': () => ({
    name: 'Cart-pole',
    g: 9.81,
    tool: 1,
    closures: [],
    elements: [],
    bodies: [
      cart(),
      link(0, 'revolute', {
        m: 0.3,
        l: 1,
        mountS: 0,
        mountAngle: HALF_PI,
        actuated: false,
        beta: 0.002,
        q0: 0.15,
      }),
    ],
  }),
  'Two-link arm': () => ({
    name: 'Two-link arm',
    g: 9.81,
    tool: 1,
    closures: [],
    elements: [],
    bodies: [link(-1, 'revolute', { q0: -HALF_PI + 0.6 }), link(0, 'revolute', { q0: 0.5 })],
  }),
  'Three-link arm': () => ({
    name: 'Three-link arm',
    g: 9.81,
    tool: 2,
    closures: [],
    elements: [],
    bodies: [
      link(-1, 'revolute', { l: 0.8, q0: -HALF_PI + 0.5 }),
      link(0, 'revolute', { l: 0.7, m: 0.8, q0: 0.5 }),
      link(1, 'revolute', { l: 0.5, m: 0.5, q0: 0.4 }),
    ],
  }),
  'Seven-joint arm': () => ({
    name: 'Seven-joint arm',
    g: 9.81,
    tool: 6,
    closures: [],
    elements: [],
    // shoulder to wrist, links shortening like a human arm: two task coordinates leave five for the
    // orientation, an elbow and posture goals
    bodies: [
      link(-1, 'revolute', { l: 0.5, m: 2, q0: -0.9 }),
      link(0, 'revolute', { l: 0.45, m: 1.6, q0: 0.5 }),
      link(1, 'revolute', { l: 0.4, m: 1.2, q0: 0.4 }),
      link(2, 'revolute', { l: 0.3, m: 0.9, q0: 0.3 }),
      link(3, 'revolute', { l: 0.25, m: 0.6, q0: 0.3 }),
      link(4, 'revolute', { l: 0.2, m: 0.4, q0: 0.2 }),
      link(5, 'revolute', { l: 0.15, m: 0.3, q0: 0.1 }),
    ],
  }),
  'Double pendulum': () => ({
    name: 'Double pendulum',
    g: 9.81,
    tool: 1,
    closures: [],
    elements: [],
    bodies: [
      link(-1, 'revolute', { mountAngle: -HALF_PI, actuated: false, beta: 0, q0: 2 }),
      link(0, 'revolute', { actuated: false, beta: 0, q0: 0.5 }),
    ],
  }),
  'Wall pendulum': () => ({
    name: 'Wall pendulum',
    g: 9.81,
    tool: 2,
    closures: [],
    elements: [],
    bodies: [
      link(-1, 'fixed', { mountX: -1, mountY: 1, l: 1.2, m: 2, shape: 'link' }),
      link(0, 'revolute', { mountS: 1, mountAngle: -HALF_PI, actuated: false, beta: 0.01, l: 0.9, q0: 0.8 }),
      link(1, 'revolute', { actuated: false, beta: 0.01, l: 0.7, m: 0.6, q0: 0.4 }),
    ],
  }),
  'Branching arm': () => ({
    name: 'Branching arm',
    g: 9.81,
    tool: 2,
    closures: [],
    elements: [],
    bodies: [
      // a driven arm raised a little above its target, carrying a passive pendulum on its middle (swung out)
      // and a driven forearm at its tip
      link(-1, 'revolute', { l: 1, q0: 0.4 }),
      link(0, 'revolute', {
        l: 0.7,
        m: 0.6,
        mountS: 0.5,
        mountAngle: -HALF_PI,
        actuated: false,
        beta: 0.02,
        q0: 0.6,
      }),
      link(0, 'revolute', { l: 0.7, m: 0.6, mountS: 1, mountAngle: -0.6, q0: 0 }),
    ],
  }),
  Acrobot: () => ({
    name: 'Acrobot',
    g: 9.81,
    tool: 1,
    closures: [],
    elements: [],
    bodies: [
      link(-1, 'revolute', { mountAngle: HALF_PI, actuated: false, q0: 0.02 }),
      link(0, 'revolute', { q0: 0 }),
    ],
  }),
  'Cart double pendulum': () => ({
    name: 'Cart double pendulum',
    g: 9.81,
    tool: 2,
    closures: [],
    elements: [],
    bodies: [
      cart(),
      link(0, 'revolute', {
        m: 0.3,
        l: 0.6,
        mountS: 0,
        mountAngle: HALF_PI,
        actuated: false,
        beta: 0.002,
        q0: 0.05,
      }),
      link(1, 'revolute', { m: 0.2, l: 0.6, actuated: false, beta: 0.002, q0: -0.05 }),
    ],
  }),
  'Four-bar linkage': () => ({
    name: 'Four-bar linkage',
    g: 9.81,
    tool: 1,
    closures: [pin(1, 1, 2, 1)],
    elements: [],
    bodies: [
      link(-1, 'revolute', { l: 0.5, m: 0.5, q0: HALF_PI, qTarget: 1.2, beta: 0.05 }),
      link(0, 'revolute', { l: 1.2, m: 1, actuated: false, q0: -1.2, beta: 0.01 }),
      link(-1, 'revolute', { l: 1, m: 0.8, mountX: 1, actuated: false, q0: 1.6, beta: 0.01 }),
    ],
  }),
  'Slider-crank': () => ({
    name: 'Slider-crank',
    g: 9.81,
    tool: 1,
    closures: [{ kind: 'slot', angle: 0, a: -1, sa: 0, b: 1, sb: 1, gx: 0, gy: 0 }],
    elements: [],
    bodies: [
      link(-1, 'revolute', { l: 0.4, m: 0.5, q0: 1, qTarget: 2, beta: 0.05 }),
      link(0, 'revolute', { l: 1.2, m: 0.6, actuated: false, q0: -1.3, beta: 0.01 }),
    ],
  }),
  'Gantry (PR)': () => ({
    name: 'Gantry (PR)',
    g: 9.81,
    tool: 1,
    closures: [],
    elements: [],
    bodies: [
      cart({ m: 2, beta: 0.5, limit: 2, mountY: 1 }),
      link(0, 'revolute', { m: 0.5, l: 0.9, mountS: 0, mountAngle: -HALF_PI, q0: 0.4 }),
    ],
  }),
  'Gripper arm': () => ({
    name: 'Gripper arm',
    g: 9.81,
    // the tool sits at the wrist's tip: the middle of the palm, between the fingers
    tool: 3,
    closures: [],
    elements: [],
    bodies: [
      // four joints carry the hand (two to spare for posture); its target posture differs a little from the
      // start, so a posture goal has something to do
      link(-1, 'revolute', { l: 0.6, m: 1.2, q0: 0.9, qTarget: 0.6 }),
      link(0, 'revolute', { l: 0.55, m: 0.9, q0: -0.6, qTarget: -0.3 }),
      link(1, 'revolute', { l: 0.45, m: 0.6, q0: -0.7, qTarget: -0.5 }),
      link(2, 'revolute', { l: 0.2, m: 0.3, q0: 0.1, qTarget: 0, beta: 0.01 }),
      // the palm: two welded halves across the wrist
      link(3, 'fixed', { l: 0.14, m: 0.05, mountAngle: HALF_PI }),
      link(3, 'fixed', { l: 0.14, m: 0.05, mountAngle: -HALF_PI }),
      // light fingers pointing forward from the palm's ends, closing a little (light friction too, or it
      // would dominate their tiny inertia)
      link(4, 'revolute', { l: 0.22, m: 0.08, mountAngle: -HALF_PI, q0: -0.25, qTarget: -0.25, beta: 0.002 }),
      link(5, 'revolute', { l: 0.22, m: 0.08, mountAngle: HALF_PI, q0: 0.25, qTarget: 0.25, beta: 0.002 }),
    ],
  }),
  '3-RPR platform': () => {
    // a planar parallel robot: three legs on passive floor hinges, each with a driven telescopic actuator,
    // carry a platform — 7 coordinates, 2 pins (4 constraints): 3 DOF, 3 actuators
    // the middle leg leans: with all three leg lines through one point (a symmetric layout puts the middle
    // one through the crossing of the outer two) the platform could turn about it unresisted — a singular
    // pose, where the actuators cannot control it
    const floor: [number, number][] = [
      [-1.4, 0],
      [1.4, 0],
      [0.7, 0],
    ];
    const top: [number, number][] = [
      [-0.6, 1.2],
      [0.6, 1.2],
      [0, 1.2],
    ];
    const sleeve = 0.6;
    const legs = floor.map(([px, py], k) => {
      const [ax, ay] = top[k]!;
      return { px, py, angle: Math.atan2(ay - py, ax - px), length: Math.hypot(ax - px, ay - py) };
    });
    const cylinder = (k: number) =>
      link(-1, 'revolute', {
        mountX: legs[k]!.px,
        mountY: legs[k]!.py,
        mountAngle: legs[k]!.angle,
        actuated: false,
        l: sleeve,
        m: 0.4,
        beta: 0.02,
      });
    const piston = (parent: number, k: number) =>
      link(parent, 'prismatic', {
        mountS: 0,
        l: sleeve,
        m: 0.4,
        limit: 1.2,
        q0: legs[k]!.length - sleeve,
        qTarget: legs[k]!.length - sleeve,
      });
    return {
      name: '3-RPR platform',
      g: 9.81,
      // the tool is the middle leg's tip: the centre of the platform
      tool: 6,
      closures: [pin(2, 1, 4, 1), pin(2, 0.5, 6, 1)],
      elements: [],
      bodies: [
        cylinder(0),
        piston(0, 0),
        link(1, 'revolute', {
          mountS: 1,
          mountAngle: -legs[0]!.angle,
          actuated: false,
          l: 1.2,
          m: 2,
          beta: 0.02,
        }),
        cylinder(1),
        piston(3, 1),
        cylinder(2),
        piston(5, 2),
      ],
    };
  },
  'Mass–spring–damper': () => ({
    name: 'Mass–spring–damper',
    g: 9.81,
    tool: -1,
    closures: [],
    elements: [spring(-1, 0, 0, 0, { gx: -1.6, gy: 0, k: 40, l0: 1.2, d: 1.5 })],
    bodies: [cart({ m: 1, beta: 0, limit: 1.4, q0: 0.5 })],
  }),
  'Sprung pendulum': () => ({
    name: 'Sprung pendulum',
    g: 9.81,
    tool: 1,
    closures: [],
    elements: [
      spring(-1, 0, 0, 1, { gx: 1.2, gy: 0, k: 25, l0: 0.6, d: 0 }),
      spring(-1, 0, 1, 1, { gx: -1.2, gy: -1.2, k: 0, l0: 0.8, d: 1.2 }),
    ],
    bodies: [
      link(-1, 'revolute', { mountAngle: -HALF_PI, actuated: false, beta: 0.01, q0: 0.9 }),
      link(0, 'revolute', { actuated: false, beta: 0.01, l: 0.8, q0: -0.4 }),
    ],
  }),
};

export const DEFAULT_PRESET = 'Cart-pole';

export function cloneMechanism(m: Mechanism): Mechanism {
  return {
    ...m,
    bodies: m.bodies.map((b) => ({ ...b })),
    closures: m.closures.map((c) => ({ ...c })),
    elements: m.elements.map((e) => ({ ...e })),
  };
}

/** Validate before traversing a document's graph. Empty scenes are valid editor documents. */
export function validateMechanism(raw: unknown): string | null {
  const record = (v: unknown): v is Record<string, unknown> =>
    typeof v === 'object' && v !== null && !Array.isArray(v);
  if (
    !record(raw) ||
    !Array.isArray(raw.bodies) ||
    !Array.isArray(raw.closures) ||
    !Array.isArray(raw.elements)
  )
    return 'Mechanism must contain body, closure and element arrays';
  if (typeof raw.name !== 'string' || typeof raw.g !== 'number' || !Number.isFinite(raw.g))
    return 'Invalid name or gravity';
  const nb = raw.bodies.length;
  if (nb > MAX_BODIES || raw.closures.length > MAX_CLOSURES || raw.elements.length > MAX_ELEMENTS)
    return 'Mechanism exceeds engine capacity';
  const index = (v: unknown, max: number, ground = false) =>
    typeof v === 'number' && Number.isInteger(v) && v >= (ground ? -1 : 0) && v < max;
  const finite = (r: Record<string, unknown>, keys: string[]) =>
    keys.every((k) => typeof r[k] === 'number' && Number.isFinite(r[k]));
  const fraction = (v: unknown) => typeof v === 'number' && v >= 0 && v <= 1;
  if (!index(raw.tool, nb, true)) return 'Tool must refer to an existing body, or ground';
  let moving = 0;
  for (const [i, b] of raw.bodies.entries()) {
    if (
      !record(b) ||
      !index(b.parent, i, true) ||
      !['revolute', 'prismatic', 'fixed'].includes(String(b.joint)) ||
      !['link', 'block'].includes(String(b.shape)) ||
      typeof b.actuated !== 'boolean' ||
      typeof b.uniform !== 'boolean' ||
      !finite(b, [
        'mountS',
        'mountX',
        'mountY',
        'mountAngle',
        'axisAngle',
        'm',
        'l',
        'r',
        'I',
        'beta',
        'q0',
        'qTarget',
        'limit',
      ]) ||
      !fraction(b.mountS) ||
      (['m', 'l', 'r', 'I', 'beta'] as const).some((k) => !validPhysical(k, b[k])) ||
      (b.limit as number) < 0
    )
      return `Invalid body ${i + 1}: check parent, joint, finite values and nonnegative physical coefficients`;
    if (b.joint !== 'fixed') moving++;
  }
  if (moving > MAX_DOF || (nb > 0 && moving === 0)) return `Mechanism needs 1–${MAX_DOF} moving joints`;
  const ends = (r: Record<string, unknown>) =>
    index(r.a, nb, true) &&
    index(r.b, nb) &&
    r.a !== r.b &&
    fraction(r.sa) &&
    fraction(r.sb) &&
    finite(r, ['gx', 'gy']);
  for (const c of raw.closures)
    if (!record(c) || !ends(c) || !['pin', 'slot'].includes(String(c.kind)) || !finite(c, ['angle']))
      return 'Invalid loop closure endpoints, kind or angle';
  for (const e of raw.elements)
    if (
      !record(e) ||
      !ends(e) ||
      !finite(e, ['k', 'l0', 'd']) ||
      ['k', 'l0', 'd'].some((k) => (e[k] as number) < 0)
    )
      return 'Invalid spring/damper endpoints or coefficients';
  return null;
}

/** Accept documents from older links (v1 had `mountTip` and no closures). */
export function migrateOrThrow(raw: unknown): Mechanism {
  type Legacy = Partial<BodySpec> & { mountTip?: boolean };
  const m = raw as Omit<Partial<Mechanism>, 'bodies'> & { bodies?: Legacy[] };
  if (!m || !Array.isArray(m.bodies)) throw new Error('Mechanism must contain a bodies array');
  const bodies = m.bodies.map((b) => {
    if (!b || typeof b !== 'object' || Array.isArray(b)) throw new Error('Invalid body record');
    for (const key of [
      'mountS',
      'mountX',
      'mountY',
      'mountAngle',
      'axisAngle',
      'm',
      'l',
      'r',
      'I',
      'beta',
      'q0',
      'qTarget',
      'limit',
    ] as const) {
      if (b[key] !== undefined && (typeof b[key] !== 'number' || !Number.isFinite(b[key])))
        throw new Error(`Invalid body value: ${key}`);
    }
    const { mountTip, ...rest } = b;
    return link(b.parent ?? -1, b.joint ?? 'revolute', {
      ...rest,
      mountS: b.mountS ?? (mountTip ? 1 : 0),
    } as Partial<BodySpec>);
  });
  const result = {
    name: m.name ?? 'Custom',
    g: m.g ?? 9.81,
    tool: m.tool ?? -1,
    closures: (m.closures ?? []).map((c) => ({ ...pin(c.a, c.sa, c.b, c.sb, c.gx, c.gy), ...c })),
    elements: (m.elements ?? []).map((e) => ({ ...spring(e.a ?? -1, e.sa ?? 1, e.b ?? 0, e.sb ?? 1), ...e })),
    bodies,
  };
  const error = validateMechanism(result);
  if (error) throw new Error(error);
  return result;
}

/** Compatibility helper for callers that do not need an import error message. */
export function migrate(raw: unknown): Mechanism | null {
  try {
    return migrateOrThrow(raw);
  } catch {
    return null;
  }
}

/** Is `d` a descendant of (or equal to) `i` in the tree? */
export function isDescendant(mech: Mechanism, d: number, i: number): boolean {
  for (let k = d; k >= 0; k = mech.bodies[k]!.parent) if (k === i) return true;
  return false;
}

/** Re-index bodies so parents precede children, remapping parents, tool and closures. */
function reorder(mech: Mechanism): { mech: Mechanism; remap: Map<number, number> } {
  const order: number[] = [];
  const visit = (p: number) => {
    mech.bodies.forEach((b, k) => {
      if (b.parent === p) {
        order.push(k);
        visit(k);
      }
    });
  };
  visit(-1);
  const remap = new Map(order.map((old, idx) => [old, idx]));
  const at = (k: number) => (k < 0 ? -1 : (remap.get(k) ?? -1));
  return {
    remap,
    mech: {
      ...mech,
      bodies: order.map((old) => {
        const b = mech.bodies[old]!;
        return { ...b, parent: at(b.parent) };
      }),
      tool: at(mech.tool),
      closures: mech.closures.map((c) => ({ ...c, a: at(c.a), b: at(c.b) })),
      elements: mech.elements.map((e) => ({ ...e, a: at(e.a), b: at(e.b) })),
    },
  };
}

/** Move body `i` (with its subtree) under `parent` (−1 = ground). Refuses cycles. Returns the new document
 * and the new index of the moved body. */
export function reparent(
  mech: Mechanism,
  i: number,
  parent: number,
): { mech: Mechanism; index: number } | null {
  if (parent >= 0 && isDescendant(mech, parent, i)) return null;
  const m = cloneMechanism(mech);
  const b = m.bodies[i]!;
  b.parent = parent;
  b.mountS = parent >= 0 ? 1 : 0;
  if (parent < 0) {
    b.mountX = 0;
    b.mountY = 0;
  }
  const { mech: out, remap } = reorder(m);
  return { mech: { ...out, name: 'Custom' }, index: remap.get(i) ?? 0 };
}

/** Remove body `i` and its descendants (and every loop that touches them). */
export function removeBody(mech: Mechanism, i: number): Mechanism {
  const doomed = new Set<number>();
  mech.bodies.forEach((_, k) => {
    if (isDescendant(mech, k, i)) doomed.add(k);
  });
  const remap = new Map<number, number>();
  const bodies: BodySpec[] = [];
  mech.bodies.forEach((b, k) => {
    if (doomed.has(k)) return;
    remap.set(k, bodies.length);
    bodies.push({ ...b, parent: b.parent < 0 ? -1 : (remap.get(b.parent) ?? -1) });
  });
  const closures = mech.closures
    .filter((c) => !doomed.has(c.b) && !(c.a >= 0 && doomed.has(c.a)))
    .map((c) => ({ ...c, a: c.a < 0 ? -1 : (remap.get(c.a) ?? -1), b: remap.get(c.b) ?? 0 }));
  const elements = mech.elements
    .filter((e) => !doomed.has(e.b) && !(e.a >= 0 && doomed.has(e.a)))
    .map((e) => ({ ...e, a: e.a < 0 ? -1 : (remap.get(e.a) ?? -1), b: remap.get(e.b) ?? 0 }));
  const tool = doomed.has(mech.tool) ? bodies.length - 1 : (remap.get(mech.tool) ?? -1);
  return { ...mech, bodies, closures, elements, tool };
}
