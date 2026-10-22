import { expect, it } from 'vitest';
import { HISTORY } from '../generated/contracts';
import type { CheckpointFrame } from '../sim/protocol';
import { Checkpoints } from './checkpoints';

const frame = (id: number, oldest = 1, generation = 1, time = id): CheckpointFrame => ({
  params: [],
  push: null,
  checkpoint: id,
  oldest,
  generation,
  data: new Float64Array([0, time, Math.PI]),
});
it('retains exact transferred doubles, evicts expired IDs and branches without a discarded future', () => {
  const b = new Checkpoints(1);
  for (let id = 1; id <= HISTORY.frames + 3; id++) b.record(frame(id, Math.max(1, id - 10)));
  expect(b.count).toBe(11);
  expect(b.get(0)?.checkpoint).toBe(HISTORY.frames - 7);
  expect(b.get(0)?.data[2]).toBe(Math.PI);
  b.truncate(2);
  expect(b.count).toBe(3);
  const last = b.get(2)!;
  b.record(frame(HISTORY.frames + 4, last.checkpoint, 1, last.data[1]! + 0.1));
  expect(b.count).toBe(2);
  b.record(frame(1000, 1000, 2, 0));
  expect(b.count).toBe(1);
});
it('prunes expired IDs correctly across unsigned wrap and clears observations on reset', () => {
  const b = new Checkpoints(1);
  b.record(frame(0xfffffffe, 0xfffffffe, 1, 1));
  b.record(frame(0xffffffff, 0xfffffffe, 1, 2));
  b.record(frame(1, 0xffffffff, 1, 3));
  expect(b.get(0)?.checkpoint).toBe(0xffffffff);
  b.record(frame(2, 2, 1, 0));
  expect(b.count).toBe(1);
});

it('keeps the bounded tail and releases the superseded future when branching', () => {
  const b = new Checkpoints(1);
  for (let id = 1; id <= HISTORY.frames + 3; id++) b.record(frame(id));
  expect(b.count).toBe(HISTORY.frames);
  expect(b.get(0)?.checkpoint).toBe(4);
  expect(b.get(b.count - 1)?.checkpoint).toBe(HISTORY.frames + 3);
  b.truncate(2);
  expect(b.count).toBe(3);
  expect(b.get(3)).toBeUndefined();
  b.record(frame(HISTORY.frames + 4, 4, 1, 6.1));
  expect(b.count).toBe(4);
  expect(b.get(2)?.checkpoint).toBe(6);
  expect(b.get(3)?.checkpoint).toBe(HISTORY.frames + 4);
});
