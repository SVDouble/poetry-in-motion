/** Every preset can be built by hand: from nothing, with the palette, loops, springs and the inspector. */
import { describe, expect, it } from 'vitest';
import { type BodySpec, encode, type Mechanism, PRESETS } from '../model/description';
import { type Anchor, type PartKind, place } from './parts';

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
    const kind: PartKind =
      b.joint === 'fixed' ? 'weld' : b.joint === 'revolute' ? 'bar' : b.shape === 'block' ? 'cart' : 'slider';
    take(kind, b.parent < 0 ? at(-1, 0, b.mountX, b.mountY) : at(b.parent, b.mountS));
    const i = m.bodies.length - 1;
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
