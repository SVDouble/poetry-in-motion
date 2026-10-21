/**
 * What a controller page is made of, and the context its sections are written against. Every page follows
 * the same arc: the idea, the generic derivation, the same derivation for the built mechanism (with live
 * numbers from the slot running this controller), why it is stable, and where it breaks.
 */
import type { SectionBuilder, SectionSpec } from '../eq/panel';
import { texMat } from '../eq/tex';
import type { Derivation } from '../model/derivation';
import type { ControllerSpec } from '../sim/controllers';
import type { Snapshot } from '../sim/layout';

export interface PageContext {
  d: Derivation;
  snap: Snapshot;
  spec: ControllerSpec;
  n: number;
  nu: number;
  /** Degrees of freedom: n − (number of constraint equations). */
  dof: number;
  loops: boolean;
  /** Telemetry value `name[i]` of the slot running this controller (NaN when none does). */
  tele(name: string, i?: number): number;
  /** Is this controller running in some slot? */
  running(): boolean;
  /** Current value of this controller's parameter `key`. */
  p(key: string): number;
}

export interface Page {
  /** One-sentence summary shown above the parameters. */
  lead: string;
  idea(b: SectionBuilder): void;
  generic(b: SectionBuilder): void;
  concrete(b: SectionBuilder): void;
  stability(b: SectionBuilder): void;
  limits(b: SectionBuilder): void;
  /** Literature reference shown with the generic derivation. */
  cite?: string;
}

/** Settings an experiment may change (mirrored by core flags). */
export type SettingKey = 'umax' | 'posture' | 'manipulability' | 'pushK' | 'refWn' | 'orient' | 'elbow';

/** One step of an experiment's setup. The page's own controller is made primary first. */
export type Action =
  | { do: 'param'; key: string; value: number }
  | { do: 'shadow'; key: string }
  | { do: 'setting'; key: SettingKey; value: number }
  | { do: 'coriolis'; on: boolean }
  | { do: 'reset' }
  /** Reset, then make the tool's current position its target (a well-posed task-space set-point). */
  | { do: 'hold' }
  | { do: 'hang' }
  /** Reset, then start exactly at the equilibrium q* (at rest): where linear controllers are designed. */
  | { do: 'atTarget' }
  /** Aim the tool at the base plus `factor` times the vector from the base to the tool (beyond reach for
   * factor ≳ 2). */
  | { do: 'reach'; factor: number }
  /** Switch the secondary tasks on (natural frequency ω), with targets relative to the current pose:
   * the tool turned by `dphi`, the elbow moved by `delbow` (omit a field to leave that task off). */
  | { do: 'stack'; w: number; dphi?: number; delbow?: [number, number] }
  | { do: 'kick'; v: number }
  /** Push the tool with the spring towards tool + (dx, dy), or `inward` metres towards the base. */
  | { do: 'push'; dx?: number; dy?: number; inward?: number; ms: number }
  | { do: 'wait'; ms: number };

export interface Experiment {
  title: string;
  /** What is being tested, one sentence. */
  goal: string;
  actions: Action[];
  /** Plots shown while it runs (signal ids, see `scope/signals.ts`). */
  plots: { ids: string[] }[];
  /** What to notice, and why. HTML with `$…$` maths. */
  notice: string;
}

/** The lab part of a page: the plots that matter for this controller and guided experiments. */
export interface Lab {
  watch: { ids: string[] }[];
  experiments: Experiment[];
}

export const range = (k: number, f: (i: number) => string) => Array.from({ length: k }, (_, i) => f(i));

/** Joints that move the tool point (columns of its Jacobian that are not identically zero): joints beyond
 * the tool — a gripper's fingers — add no redundancy for the tool's tasks. */
export function toolChainDof(d: Derivation): number {
  return toolChain(d).filter(Boolean).length;
}

/** Per coordinate: does the joint move the tool point? (All of them when there is no tool.) */
export function toolChain(d: Derivation): boolean[] {
  const jp = d.tool?.Jp;
  if (!jp?.length) return d.coords.map(() => true);
  return d.coords.map((_, j) => jp.some((row) => row[j] !== '0'));
}

export function pageContext(d: Derivation, snap: Snapshot, spec: ControllerSpec, p: (i: number) => number) {
  const n = d.n;
  const ctx: PageContext = {
    d,
    snap,
    spec,
    n,
    nu: d.nu,
    dof: n - d.phi.length,
    loops: d.phi.length > 0,
    tele(name, i = 0) {
      const slot = snap.slotOf(spec.id);
      const field = spec.telemetry[name];
      return slot && field && i < field[1] ? slot.tele(field[0] + i) : Number.NaN;
    },
    running: () => snap.slotOf(spec.id) !== undefined,
    p: (key) => p(spec.params.findIndex((x) => x.key === key)),
  };
  return ctx;
}

export function sections(key: string, page: Page): SectionSpec[] {
  return [
    { id: `c-${key}-idea`, title: 'Idea', build: (b) => page.idea(b) },
    {
      id: `c-${key}-generic`,
      title: 'Derivation',
      ...(page.cite ? { cite: page.cite } : {}),
      build: (b) => page.generic(b),
    },
    { id: `c-${key}-concrete`, title: 'For this mechanism', build: (b) => page.concrete(b) },
    { id: `c-${key}-stability`, title: 'Stability', open: false, build: (b) => page.stability(b) },
    { id: `c-${key}-limits`, title: 'Limitations', open: false, build: (b) => page.limits(b) },
  ];
}

export const col = (xs: string[]) => texMat(xs.map((x) => [x]));

/** The coordinate vector q as TeX. */
export const qVec = (d: Derivation) => col(d.coords.map((c) => c.tex));

/** Rows of the symbolic matrix restricted to columns `cols`. */
export const columns = (m: string[][], cols: number[]) => m.map((r) => cols.map((j) => r[j] ?? '0'));

export const actuatedIdx = (d: Derivation) => d.coords.flatMap((c, i) => (c.actuated ? [i] : []));
export const passiveIdx = (d: Derivation) => d.coords.flatMap((c, i) => (c.actuated ? [] : [i]));

/** "inputs u solve B u − Aᵀλ = f" — the step shared by every law on a constrained or underactuated system. */
export function realise(b: SectionBuilder, c: PageContext, f: string): void {
  if (c.loops) {
    b.text(
      String.raw`With loop closures the actuators act together with the constraint forces. The inputs are the least-squares solution of`,
    );
    b.math(String.raw`B\,u - A^{T}\lambda = ${f}`);
  } else if (c.nu < c.n) {
    b.text(
      String.raw`Only the actuated rows of $B u = f$ can be realised; the inputs are its least-squares solution (exact when $f$ lies in the range of $B$).`,
    );
    b.math(String.raw`B\,u = ${f}`);
  } else {
    b.math(String.raw`u = B^{-1}\Big(${f}\Big)`);
  }
}

/** A live note when the controller is not running anywhere. */
export function idleNote(b: SectionBuilder, c: PageContext): void {
  b.text(
    `<span class="muted">Numbers marked live update only while ${c.spec.name} runs as the primary or as a shadow.</span>`,
  );
}
