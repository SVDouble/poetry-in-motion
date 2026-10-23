/** Every preset can be built by hand: from nothing, with the palette, loops, springs and the inspector. */
import { describe, expect, it } from 'vitest';
import {
  assemblyIssue,
  type BodySpec,
  cloneMechanism,
  dof,
  emptyMechanism,
  encode,
  groundPoints,
  link,
  type Mechanism,
  migrateOrThrow,
  PRESETS,
  spring,
  validateMechanism,
} from '../model/description';
import { initialFrames } from '../scene/kinematics';
import {
  type Anchor,
  connectBody,
  detachBody,
  moveGroundPoint,
  type PartKind,
  place,
  placeAtJoint,
} from './parts';

/** Body fields the inspector or the side panel can set (the parent comes from where a part is dropped). */
const EDITABLE: (keyof BodySpec)[] = [
  'joint',
  'mountS',
  'mountX',
  'mountY',
  'mountAngle',
  'axisAngle',
  'actuated',
  'm',
  'l',
  'r',
  'I',
  'beta',
  'uniform',
  'q0',
  'qTarget',
  'limit',
  'shape',
];

const at = (body: number, s: number, x = 0, y = 0): Anchor => ({ body, s, x, y });

function rebuild(target: Mechanism): Mechanism {
  let m: Mechanism = { name: 'Custom', g: 9.81, tool: -1, bodies: [], closures: [], elements: [] };
  const take = (kind: PartKind, a: Anchor, b?: Anchor) => {
    const r = place(m, kind, a, b);
    if (typeof r === 'string') throw new Error(`${target.name}: ${r}`);
    m = r.mech;
  };
  // bodies in order: the part that matches the joint, dropped on the parent's port (or the ground)
  for (const b of target.bodies) {
    take(b.shape === 'block' ? 'block' : 'bar', at(-1, 0, b.mountX, b.mountY));
    const i = m.bodies.length - 1;
    if (b.parent < 0) take('ground', at(-1, 0, b.mountX, b.mountY));
    take(b.joint, b.parent < 0 ? at(-1, 0, b.mountX, b.mountY) : at(b.parent, b.mountS), at(i, 0));
    const edited = { ...m.bodies[i]! };
    for (const k of EDITABLE) (edited as Record<string, unknown>)[k] = b[k];
    m.bodies[i] = edited;
  }
  // loops: the moving point first, then what it is pinned to / slides on; the slot direction is editable
  for (const c of target.closures) {
    take(c.kind, at(c.b, c.sb), c.a < 0 ? at(-1, 0, c.gx, c.gy) : at(c.a, c.sa));
    m.closures[m.closures.length - 1]!.angle = c.angle;
  }
  // springs and dampers: both ends, then k, ℓ₀, c from the inspector
  for (const e of target.elements) {
    const kind: PartKind = e.k > 0 && e.d > 0 ? 'springDamper' : e.k > 0 ? 'spring' : 'damper';
    take(kind, e.a < 0 ? at(-1, 0, e.gx, e.gy) : at(e.a, e.sa), at(e.b, e.sb));
    const k = m.elements.length - 1;
    m.elements[k] = { ...m.elements[k]!, k: e.k, l0: e.l0, d: e.d, gx: e.gx, gy: e.gy };
  }
  // the tool and gravity (inspector, side panel)
  return { ...m, tool: target.tool, g: target.g };
}

describe('builder', () => {
  for (const [name, make] of Object.entries(PRESETS)) {
    it(`builds “${name}” by hand`, () => {
      const target = make();
      expect(encode(rebuild(target))).toEqual(encode(target));
    });
  }
});

it('keeps unconnected bodies editable and serializable, but out of the solver', () => {
  const m = take(place(emptyMechanism(), 'bar', at(-1, 0, 1, 2)));
  expect(validateMechanism(m)).toBeNull();
  expect(assemblyIssue(m)).toMatch(/unconnected/);
  expect(() => encode(m)).toThrow(/Connect every body/);
  expect(migrateOrThrow(JSON.parse(JSON.stringify(m)))).toEqual(m);
  expect(initialFrames(m)[0]).toEqual([1, 2, 0]);
  expect(connectBody(m, 'revolute', at(-1, 0), 0)).toMatch(/Ground anchor/);
});

it('attaches at an arbitrary point along a bar and keeps actuation independent of mobility', () => {
  let m = take(place(emptyMechanism(), 'ground', at(-1, 0)));
  m = take(placeAtJoint(m, 'bar', 'fixed', at(-1, 0), at(-1, 0, 2, 0)));
  expect(assemblyIssue(m)).toMatch(/Fixed assembly/);
  m = take(placeAtJoint(m, 'bar', 'revolute', at(0, 0.3, 0.6, 0), at(-1, 0, 0.6, 1)));
  expect(assemblyIssue(m)).toBeNull();
  expect(initialFrames(m)[1]![0]).toBeCloseTo(0.6);
  expect(initialFrames(m)[1]![2]).toBeCloseTo(Math.PI / 2);
  expect(dof(m)).toBe(1);
  m.bodies[1]!.actuated = false;
  expect(dof(m)).toBe(1);
  expect(validateMechanism(m)).toBeNull();
});

it('remaps branches, loops, springs and the tool when attaching to a later body', () => {
  let m = take(place(emptyMechanism(), 'bar', at(-1, 0, 1, 1)));
  m = take(place(m, 'bar', at(-1, 0, 2, 1)));
  m.bodies[0]!.m = 2;
  m.bodies[1]!.m = 3;
  m.tool = 0;
  m = take(place(m, 'spring', at(0, 1, 1.8, 1), at(1, 1, 2.8, 1)));
  m = take(place(m, 'pin', at(0, 1), at(1, 1)));
  m = take(connectBody(m, 'revolute', at(1, 0.25, 2.2, 1), 0));
  expect(m.bodies.map((b) => [b.m, b.parent])).toEqual([
    [3, -1],
    [2, 0],
  ]);
  expect(m.tool).toBe(1);
  expect([m.elements[0]!.a, m.elements[0]!.b]).toEqual([1, 0]);
  expect([m.closures[0]!.a, m.closures[0]!.b]).toEqual([0, 1]);
  expect(connectBody(m, 'fixed', at(1, 1), 0)).toMatch(/cycle/);
  expect(validateMechanism(m)).toBeNull();
});

it('disconnects a joint without moving the body or its descendants', () => {
  const m = PRESETS['Two-link arm']!();
  const frames = initialFrames(m);
  const draft = detachBody(m, 0);
  expect(initialFrames(draft)).toEqual(frames);
  expect(draft.bodies[1]!.parent).toBe(0);
  expect(assemblyIssue(draft)).toMatch(/unconnected/);
  expect(validateMechanism(draft)).toBeNull();
});

it('copies ground points independently and rejects invalid draft metadata', () => {
  const m = take(place(emptyMechanism(), 'ground', at(-1, 0, 1, 2)));
  const copy = cloneMechanism(m);
  copy.anchors![0]![0] = 9;
  expect(m.anchors).toEqual([[1, 2]]);
  expect(validateMechanism({ ...m, anchors: [[Infinity, 0]] })).toMatch(/anchors/);
  const loose = take(place(m, 'block', at(-1, 0)));
  loose.bodies[0]!.joint = 'prismatic';
  expect(validateMechanism(loose)).toMatch(/Invalid body/);
});

function take(result: ReturnType<typeof place>): Mechanism {
  if (typeof result === 'string') throw new Error(result);
  return result.mech;
}

it('moves a shared ground point with its rooted components, spring and loop ends', () => {
  const m = PRESETS['Cart-pole']!();
  m.anchors = [
    [0, 0],
    [3, 0],
  ];
  m.bodies.push(link(-1, 'fixed', { detached: true, mountX: 0, mountY: 0 }));
  m.closures = [{ kind: 'pin', a: -1, sa: 0, b: 1, sb: 1, gx: 0, gy: 0, angle: 0 }];
  m.elements = [spring(-1, 0, 1, 1, { gx: 0, gy: 0 }), spring(-1, 0, 1, 0, { gx: 3, gy: 0 })];
  const before = initialFrames(m);
  const moved = moveGroundPoint(m, 0, [1, -2]);
  if (typeof moved === 'string') throw new Error(moved);
  expect(m.anchors[0]).toEqual([0, 0]);
  expect(moved.anchors).toEqual([
    [1, -2],
    [3, 0],
  ]);
  expect([moved.closures[0]!.gx, moved.closures[0]!.gy]).toEqual([1, -2]);
  expect([moved.elements[0]!.gx, moved.elements[0]!.gy]).toEqual([1, -2]);
  expect(moved.elements[1]).toEqual(m.elements[1]);
  const after = initialFrames(moved);
  for (const i of [0, 1]) {
    expect(after[i]![0] - before[i]![0]).toBeCloseTo(1);
    expect(after[i]![1] - before[i]![1]).toBeCloseTo(-2);
    expect(after[i]![2]).toBe(before[i]![2]);
  }
  expect(after[2]).toEqual(before[2]); // A loose body at the same coordinates is not grounded.
  expect(validateMechanism(moved)).toBeNull();
});

it('edits implicit preset supports without adding metadata and rejects merging distinct supports', () => {
  const m = PRESETS['Four-bar linkage']!();
  const points = groundPoints(m);
  expect(moveGroundPoint(m, 0, points[1]!)).toMatch(/already there/);
  expect(moveGroundPoint(m, 0, [NaN, 0])).toMatch(/finite/);
  expect(moveGroundPoint(m, -1, [0, 0])).toMatch(/valid/);
  const moved = moveGroundPoint(m, 0, [-0.25, 0.1]);
  if (typeof moved === 'string') throw new Error(moved);
  expect(moved.anchors).toBeUndefined();
  expect(groundPoints(moved)).toEqual([[-0.25, 0.1], points[1]]);
  expect(validateMechanism(moved)).toBeNull();
});
