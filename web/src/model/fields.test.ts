import { expect, it } from 'vitest';
import { link, normalise } from './description';
import { physicalControl, validPhysical } from './fields';

it('keeps valid idealizations independent of slider ranges', () => {
  expect(validPhysical('m', 0)).toBe(true);
  expect(validPhysical('I', 0)).toBe(true);
  expect(validPhysical('g', -9.81)).toBe(true);
  expect(validPhysical('r', -100)).toBe(true);
  expect(validPhysical('m', -1)).toBe(false);
  expect(validPhysical('g', Number.NaN)).toBe(false);
  expect(physicalControl('m', 'slider').min).toBeGreaterThan(0);
  expect(physicalControl('m', 'inspector').min).toBe(0);
});

it('updates uniform dependencies together when authoring mass or length', () => {
  const b = link(-1, 'revolute', { m: 4, l: 3, uniform: true });
  expect(b.r).toBe(1.5);
  expect(b.I).toBe(3);
  expect(normalise({ ...b, l: 6 })).toMatchObject({ r: 3, I: 12 });
  expect(normalise({ ...b, uniform: false, l: 6 })).toMatchObject({ r: 1.5, I: 3 });
});
