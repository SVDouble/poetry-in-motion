/**
 * Tool targets on every preset that offers them: the mechanism must start where its actuators control it
 * (not at a singular pose), and every joint-space controller — the reference followers and the regulators
 * designed at an equilibrium (LQR, MPC), whose equilibrium follows the inverse kinematics — must reach a
 * target inside the workspace. Run `make wasm` first.
 */
import { beforeAll, describe, expect, it } from 'vitest';
import { toolTargets } from '../app/settings';
import { encode, PRESETS } from '../model/description';
import { TestCore } from '../sim/testing';

let tc: TestCore;
beforeAll(async () => {
  tc = await TestCore.load();
});

describe('tool targets', () => {
  it('are reached by every joint-space controller on every preset that offers them', {
    timeout: 120_000,
  }, () => {
    const failed: string[] = [];
    for (const [name, make] of Object.entries(PRESETS)) {
      const d = tc.build(encode(make()));
      if (!toolTargets(d)) continue;
      tc.core.step(10);
      const s0 = tc.snapshot();
      if (s0.v('EQ_RESIDUAL', 2) !== 1) {
        failed.push(`${name}: starts where the actuators cannot control it`);
        continue;
      }
      // with one degree of freedom the tool runs along a curve: a point off it is out of reach by design
      if (s0.v('EQ_RESIDUAL', 1) < 2) continue;
      const reg = tc.controllers();
      for (const key of ['pd_gravity', 'computed_torque', 'lqr', 'mpc']) {
        const spec = reg.specs.find((s) => s.key === key)!;
        if (reg.applicable[spec.id] !== null) continue;
        tc.send({ type: 'reset' });
        tc.send({ type: 'controller', slot: 0, kind: spec.id });
        // from rest at the equilibrium (a linear regulator is not made for the way there from afar)
        tc.core.step(20);
        const s = tc.snapshot();
        for (let i = 0; i < d.n; i++) tc.send({ type: 'state', i, q: s.v('Q_TARGET', i), qd: 0 });
        tc.core.step(500);
        tc.snapshot(s);
        // a little towards the first joint, and up: inside the workspace
        const [bx, by] = s.frame(1);
        const [x0, y0] = [s.v('TOOL_P'), s.v('TOOL_P', 1)];
        const [tx, ty] = [x0 + 0.15 * (bx - x0), y0 + 0.15 * (by - y0) + 0.05];
        tc.flag('TargetX', tx);
        tc.flag('TargetY', ty);
        tc.flag('UseTool', 1);
        tc.core.step(15000);
        tc.snapshot(s);
        if (s.v('TARGET', 3) !== 1) {
          // out of this mechanism's reach: nothing to check here (the nearest point is aimed at instead)
          tc.flag('UseTool', 0);
          continue;
        }
        const err = Math.hypot(s.v('TOOL_P') - tx, s.v('TOOL_P', 1) - ty);
        if (!(err < 5e-3)) failed.push(`${name} · ${key}: ends ${err.toExponential(1)} m from the target`);
        tc.flag('UseTool', 0);
      }
    }
    expect(failed).toEqual([]);
  });

  it('are held by the regulators from wherever a fully actuated mechanism starts', {
    timeout: 120_000,
  }, () => {
    const failed: string[] = [];
    for (const [name, make] of Object.entries(PRESETS)) {
      const d = tc.build(encode(make()));
      tc.core.step(10);
      if (tc.snapshot().v('EQ_RESIDUAL', 2) !== 1) continue; // underactuated: only q* itself can be held
      const reg = tc.controllers();
      for (const key of ['lqr', 'mpc']) {
        const spec = reg.specs.find((s) => s.key === key)!;
        if (reg.applicable[spec.id] !== null) continue;
        tc.send({ type: 'reset' });
        tc.send({ type: 'controller', slot: 0, kind: spec.id });
        tc.core.step(20000);
        const err = tc.snapshot().slots[0]!.v('KIND', 2);
        if (!(err < 1e-2)) failed.push(`${name} · ${key}: ends ${err.toExponential(1)} from q*`);
      }
      void d;
    }
    expect(failed).toEqual([]);
  });
});
