/**
 * The free-body diagram's joint loads, against the core. The moment a joint passes is computed from the
 * angular-momentum balance of the subtree it carries; at a hinge that must equal the drive less the
 * friction, which the core knows independently. Likewise the force a slide passes along its axis. Run
 * `make wasm` first.
 */
import { beforeAll, describe, expect, it } from 'vitest';
import { coordIndex, encode, normalise, PRESETS, paramVector } from '../model/description';
import { TestCore } from '../sim/testing';
import { poseBounds } from './kinematics';
import { Figure } from './schematic';

let tc: TestCore;
beforeAll(async () => {
  tc = await TestCore.load();
});

interface Loads {
  pointForces(...a: unknown[]): unknown[];
  heldMoment(...a: unknown[]): number;
  reaction(...a: unknown[]): [number, number];
}

describe('free-body diagram', () => {
  it('balances every joint with the drive and friction the core applies', () => {
    for (const name of ['Two-link arm', 'Double pendulum', 'Branching arm', 'Sprung pendulum', 'Cart-pole']) {
      const mech = PRESETS[name]!();
      const d = tc.build(encode(mech));
      tc.send({ type: 'kick', i: 0, v: 1.5 });
      tc.core.step(250);
      const snap = tc.snapshot();
      const params = paramVector(mech);
      const fig = new Figure(mech, d, poseBounds(mech)) as unknown as Loads;
      const forces = fig.pointForces(snap, params);
      const coord = coordIndex(mech);
      let a = 0;
      mech.bodies.forEach((raw, k) => {
        const b = normalise(raw);
        const j = coord[k]!;
        if (j < 0) return;
        const u = b.actuated ? snap.v('U', a++) : 0;
        const drive = u + snap.v('TAU_EXT', j) - b.beta * snap.v('QD', j);
        const scale = 1 + Math.abs(drive);
        if (b.joint === 'revolute') {
          const held = fig.heldMoment(k, snap, params, forces);
          expect(Math.abs(held - drive), `${name} joint ${k + 1}: ${held} vs ${drive}`).toBeLessThan(
            1e-6 * scale,
          );
        } else {
          const [rx, ry] = fig.reaction(k, snap, params, forces);
          const parent = b.parent < 0 ? 0 : snap.frame(b.parent + 1)[2];
          const axis = parent + b.mountAngle + b.axisAngle;
          const along = rx * Math.cos(axis) + ry * Math.sin(axis);
          expect(Math.abs(along - drive), `${name} slide ${k + 1}: ${along} vs ${drive}`).toBeLessThan(
            1e-6 * scale,
          );
        }
      });
    }
  });
});
