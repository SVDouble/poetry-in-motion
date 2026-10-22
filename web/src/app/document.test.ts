import { expect, it, vi } from 'vitest';
import { emptyMechanism, PRESETS } from '../model/description';
import type { BuildResult, Built } from '../sim/client';
import { DocumentSession } from './document';

function session() {
  let done: (result: BuildResult) => void;
  let current: () => boolean;
  const client = {
    build: vi.fn((_desc: number[], check = () => true) => {
      current = check;
      return new Promise<BuildResult>((resolve) => {
        done = resolve;
      });
    }),
    install: vi.fn(),
  };
  const doc = new DocumentSession(PRESETS['Two-link arm']!(), client);
  return { doc, client, current: () => current(), resolve: (r: BuildResult) => done(r) };
}
const built = { ok: true, generation: 1 } as Built;

it('invalidates a pending candidate at edit time and retains the installed document on failure', async () => {
  const s = session();
  const initial = s.doc.draft;
  const first = s.doc.rebuild();
  s.resolve(built);
  expect((await first).kind).toBe('installed');
  const pending = s.doc.rebuild();
  s.doc.edit(PRESETS['Cart-pole']!());
  expect(s.current()).toBe(false);
  s.resolve({ ok: false, error: 'superseded' });
  expect((await pending).kind).toBe('stale');
  expect(s.doc.installed).toEqual(initial);
  const failed = s.doc.rebuild();
  s.resolve({ ok: false, error: 'invalid model' });
  expect(await failed).toEqual({ kind: 'error', error: 'invalid model' });
  expect(s.doc.installed).toEqual(initial);
});

it('installs the captured accepted document when a later edit arrives during commit', async () => {
  const s = session();
  const original = s.doc.draft;
  const captured = structuredClone(original);
  const pending = s.doc.rebuild();
  expect(s.current()).toBe(true); // worker accepts prepare here
  original.bodies.pop(); // the editor may mutate the previous draft before replacing it
  s.doc.edit(PRESETS['Cart-pole']!());
  s.resolve(built); // accepted commit acknowledgment
  expect(await pending).toEqual({ kind: 'installed', document: captured, result: built });
  expect(s.client.install).toHaveBeenCalledWith(built);
  expect(s.doc.installed).toEqual(captured);
  expect(s.doc.draft.name).toBe('Cart-pole');
});

it('never renders an accepted in-flight model over a subsequently emptied document', async () => {
  const s = session();
  const pending = s.doc.rebuild();
  s.doc.edit(emptyMechanism());
  s.resolve(built);
  expect((await pending).kind).toBe('empty');
  expect(s.client.install).toHaveBeenCalledWith(built);
  expect(s.doc.installed).toBeNull();
});
