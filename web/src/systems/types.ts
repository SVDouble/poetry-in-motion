import type { SectionSpec } from '../eq/panel';
import type { SectionContext } from '../eq/sections';
import type { SymbolicModel } from '../generated';
import type { PlotSpec } from '../plots/plots';
import type { Stage } from '../scene/stage';
import type { SimClient } from '../sim/client';
import type { Flag, Snapshot } from '../sim/layout';

/** Values of the UI parameters (physical sliders and controller settings) by key. */
export type UiParams = Record<string, number>;

export interface ParamSpec {
  key: string;
  /** LaTeX label. */
  tex: string;
  unit: string;
  min: number;
  max: number;
  value: number;
  step?: number;
  log?: boolean;
  group: 'physical' | 'control';
  hint?: string;
}

export interface ModeSpec {
  id: number;
  label: string;
  hint: string;
}

export interface KeyHint {
  keys: string;
  action: string;
}

/** Narrow handle the system code uses to talk to the simulation. */
export interface SimApi {
  readonly client: SimClient;
  readonly snap: Snapshot;
  readonly ui: UiParams;
  mode(): number;
  flag(f: Flag, v: number): void;
  input(i: number, v: number): void;
  kick(i: number, v: number): void;
  state(i: number, q: number, qd: number): void;
}

export interface SceneHandle {
  /** Called every animation frame with the newest snapshot. */
  update(): void;
  /** Pointer in world coordinates. */
  pointer?(x: number, y: number, kind: 'down' | 'move' | 'up'): void;
  /** Clear trails etc. after a reset. */
  reset(): void;
}

export interface SystemDef {
  id: number;
  key: string;
  title: string;
  model: SymbolicModel;
  /** Symbol of the generalised coordinates in the equations. */
  q: string;
  /** Plain-text coordinate names for plots. */
  qNames: string[];
  /** Region of the plane (m) that must stay visible. */
  view: { cx: number; cy: number; width: number; height: number };
  modes: ModeSpec[];
  defaultMode: number;
  params: ParamSpec[];
  keys: KeyHint[];
  /** Model parameter vector, in generated order, from the UI parameters. */
  modelParams(ui: UiParams): number[];
  /** Flags derived from the UI parameters. */
  flags(ui: UiParams): [Flag, number][];
  /** Handle a key; return true if consumed. */
  onKey(e: KeyboardEvent, down: boolean, api: SimApi): boolean;
  sections(ctx: SectionContext, api: SimApi): SectionSpec[];
  plots(api: SimApi): PlotSpec[];
  scene(stage: Stage, api: SimApi): SceneHandle;
}
