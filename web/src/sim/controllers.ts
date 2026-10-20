/** The controller registry as described by the core (`control::specs_json`), plus applicability. */

export interface ParamSpec {
  key: string;
  tex: string;
  default: number;
  min: number;
  max: number;
  log: boolean;
  unit: string;
}

export interface ControllerSpec {
  id: number;
  key: string;
  name: string;
  family: 'none' | 'joint' | 'energy' | 'interaction' | 'optimal' | 'underactuated';
  params: ParamSpec[];
  /** Telemetry field → [offset, length] inside a slot's telemetry block. */
  telemetry: Record<string, [number, number]>;
}

export interface Controllers {
  specs: ControllerSpec[];
  /** Per kind: null when it can run on the built mechanism, else the reason it cannot. */
  applicable: (string | null)[];
  /** Current parameter values per kind. */
  params: number[][];
}

export const parseControllers = (json: string) => JSON.parse(json) as Controllers;
