/**
 * The builder encodes mechanisms before the wasm core is loaded, so it keeps its own copy of the
 * description's limits and record sizes; the core publishes the originals in its layout. They must agree.
 * Run `make wasm` first.
 */
import { beforeAll, describe, expect, it } from 'vitest';
import { TestCore } from '../sim/testing';
import {
  DESC_VERSION,
  encode,
  MAX_BODIES,
  MAX_CLOSURES,
  MAX_DOF,
  MAX_ELEMENTS,
  PARAMS_PER_BODY,
  PER_BODY,
  PRESETS,
  paramVector,
} from './description';

let tc: TestCore;
beforeAll(async () => {
  tc = await TestCore.load();
});

describe('the description', () => {
  it('has the core’s limits and record sizes', () => {
    const l = tc.layout.limits;
    expect({
      DESC_VERSION,
      MAX_BODIES,
      MAX_CLOSURES,
      MAX_DOF,
      MAX_ELEMENTS,
      PARAMS_PER_BODY,
      PER_BODY,
    }).toEqual({
      DESC_VERSION: l.version,
      MAX_BODIES: l.maxBodies,
      MAX_CLOSURES: l.maxClosures,
      MAX_DOF: l.maxDof,
      MAX_ELEMENTS: l.maxElements,
      PARAMS_PER_BODY: l.paramsPerBody,
      PER_BODY: l.perBody,
    });
    for (const [name, make] of Object.entries(PRESETS)) {
      const m = make();
      const size =
        l.header +
        m.bodies.length * l.perBody +
        m.closures.length * l.perClosure +
        m.elements.length * l.perElement;
      expect(encode(m).length, name).toBe(size);
      expect(paramVector(m).length, name).toBe(m.bodies.length * l.paramsPerBody + 1);
    }
  });
});
