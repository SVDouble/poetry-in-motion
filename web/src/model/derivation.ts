/** Shape of the derivation JSON produced by the core (`core/src/doc.rs`). All strings are LaTeX. */

export type TexMatrix = string[][];

export interface CoordInfo {
  tex: string;
  dot: string;
  ddot: string;
  kind: 'revolute' | 'prismatic';
  actuated: boolean;
  /** Body moved by this coordinate. */
  body: number;
}

export interface ParamInfo {
  id: string;
  tex: string;
  body: number;
  kind: 'm' | 'l' | 'r' | 'I' | 'beta' | 'g';
  /** Index into the model parameter vector. */
  index: number;
  value: number;
}

export interface BodyInfo {
  label: string;
  parent: string;
  kind: 'revolute' | 'prismatic' | 'fixed';
  actuated: boolean;
  /** Orientation φᵢ of the body frame. */
  phi: string;
  gLocal: TexMatrix;
  gWorld: TexMatrix;
  com: string[];
  vcom: string[];
  omega: string;
  T: string;
  V: string;
}

export interface ToolInfo {
  body: string;
  p: string[];
  Jp: TexMatrix;
  Js: TexMatrix;
  twists: TexMatrix;
  gst0: TexMatrix;
  phi: string;
}

export interface ClosureInfo {
  /** Body label, or "s" for a ground point. */
  a: string;
  b: string;
  sa: number;
  sb: number;
  pa: string[];
  pb: string[];
}

export interface Derivation {
  n: number;
  nu: number;
  coords: CoordInfo[];
  params: ParamInfo[];
  bodies: BodyInfo[];
  T: string;
  V: string;
  M: TexMatrix;
  C: TexMatrix;
  /** Non-zero Christoffel symbols Γᵢⱼₖ with j ≤ k (1-based indices). */
  gamma: { i: number; j: number; k: number; tex: string }[];
  dV: string[];
  N: string[];
  B: TexMatrix;
  /** Scalar equations of motion, left-hand side of row i (= τᵢ). */
  eom: string[];
  tool: ToolInfo | null;
  closures: ClosureInfo[];
  /** Constraints φ(q) = 0 (two per closure), their Jacobian A = ∂φ/∂q and Ȧ q̇. */
  phi: string[];
  A: TexMatrix;
  adot: string[];
  /** [shorthand, meaning]. */
  trig: [string, string][];
}
