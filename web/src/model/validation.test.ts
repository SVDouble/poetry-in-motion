import { describe, expect, it } from 'vitest';
import { emptyMechanism, migrate, migrateOrThrow, PRESETS, validateMechanism } from './description';

describe('mechanism imports', () => {
  it.each(Object.keys(PRESETS))('accepts the %s preset', (name) => {
    expect(validateMechanism(PRESETS[name]!())).toBeNull();
    expect(migrateOrThrow(PRESETS[name]!())).toEqual(PRESETS[name]!());
  });
  it('accepts an empty shared scene and migrates a legacy mount', () => {
    expect(migrateOrThrow(emptyMechanism())).toEqual(emptyMechanism());
    const raw = { bodies: [{ parent: -1 }, { parent: 0, mountTip: true }] };
    expect(migrateOrThrow(raw).bodies[1]?.mountS).toBe(1);
  });
  it('rejects invalid graphs, tags, domains and record types before traversal', () => {
    for (const patch of [
      { parent: 0 },
      { parent: 0.5 },
      { parent: 12 },
      { joint: 'spatial' },
      { m: -1 },
      { m: Number.NaN },
      { l: Number.POSITIVE_INFINITY },
      { I: -1, uniform: false },
      { actuated: 'yes' },
    ]) {
      const raw = PRESETS['Two-link arm']!();
      Object.assign(raw.bodies[0]!, patch);
      expect(migrate(raw), JSON.stringify(patch)).toBeNull();
    }
    expect(migrate({ bodies: [1] })).toBeNull();
    expect(migrate({ bodies: [null] })).toBeNull();
    const loop = PRESETS['Two-link arm']!();
    loop.bodies[0]!.parent = 1;
    loop.bodies[1]!.parent = 0;
    expect(() => migrateOrThrow(loop)).toThrow(/parent/);
    const closure = PRESETS['Four-bar linkage']!();
    closure.closures[0]!.b = 20;
    expect(migrate(closure)).toBeNull();
  });
});
