/**
 * Task-space quantities of the tool computed from the published M(q) and J(q): the task inertia
 * Λ = (J M⁻¹ Jᵀ)⁻¹, the dynamically consistent inverse J̄ = M⁻¹ Jᵀ Λ and the null-space projector
 * N = I − Jᵀ J̄ᵀ. Small dense algebra (n ≤ 6), recomputed once per snapshot.
 */
import type { Snapshot } from '../sim/layout';

export interface TaskSpace {
  n: number;
  /** Λ, row-major 2 × 2. */
  lambda: number[];
  /** J̄, row-major n × 2. */
  jbar: number[];
  /** N, row-major n × n. */
  proj: number[];
  /** rank-deficiency guard: false near singular poses. */
  ok: boolean;
}

/** Solve M X = B for X (n × k) by Gaussian elimination with partial pivoting; null if singular. */
export function solve(n: number, m: number[], b: number[], k: number): number[] | null {
  const a = m.slice(0, n * n);
  const x = b.slice(0, n * k);
  for (let c = 0; c < n; c++) {
    let piv = c;
    for (let r = c + 1; r < n; r++) if (Math.abs(a[r * n + c]!) > Math.abs(a[piv * n + c]!)) piv = r;
    if (Math.abs(a[piv * n + c]!) < 1e-14) return null;
    for (let j = 0; j < n; j++) [a[c * n + j], a[piv * n + j]] = [a[piv * n + j]!, a[c * n + j]!];
    for (let j = 0; j < k; j++) [x[c * k + j], x[piv * k + j]] = [x[piv * k + j]!, x[c * k + j]!];
    for (let r = 0; r < n; r++) {
      if (r === c) continue;
      const f = a[r * n + c]! / a[c * n + c]!;
      if (f === 0) continue;
      for (let j = c; j < n; j++) a[r * n + j]! -= f * a[c * n + j]!;
      for (let j = 0; j < k; j++) x[r * k + j]! -= f * x[c * k + j]!;
    }
  }
  for (let r = 0; r < n; r++) for (let j = 0; j < k; j++) x[r * k + j]! /= a[r * n + r]!;
  return x;
}

export function taskSpace(snap: Snapshot): TaskSpace {
  const n = snap.n;
  const m = Array.from({ length: n * n }, (_, i) => snap.v('M', i));
  const jp = Array.from({ length: 2 * n }, (_, i) => snap.v('JP', i));
  // M⁻¹ Jᵀ (n × 2)
  const jt = Array.from({ length: 2 * n }, (_, i) => jp[(i % 2) * n + Math.floor(i / 2)]!);
  const minvJt = solve(n, m, jt, 2);
  const eye = Array.from({ length: n * n }, (_, i) => (i % (n + 1) === 0 ? 1 : 0));
  if (!minvJt) return { n, lambda: [0, 0, 0, 0], jbar: new Array(2 * n).fill(0), proj: eye, ok: false };
  const li = (a: number, b: number) => {
    let s = 0;
    for (let c = 0; c < n; c++) s += jp[a * n + c]! * minvJt[c * 2 + b]!;
    return s;
  };
  const [a, b, d] = [li(0, 0), li(0, 1), li(1, 1)];
  const det = a * d - b * b;
  const ok = Math.abs(det) > 1e-10 * Math.max(1, a * d);
  const lambda = ok ? [d / det, -b / det, -b / det, a / det] : [0, 0, 0, 0];
  const jbar = Array.from({ length: 2 * n }, (_, i) => {
    const [c, r] = [Math.floor(i / 2), i % 2];
    return minvJt[c * 2]! * lambda[r]! + minvJt[c * 2 + 1]! * lambda[2 + r]!;
  });
  const proj = Array.from({ length: n * n }, (_, idx) => {
    const [i, j] = [Math.floor(idx / n), idx % n];
    return eye[idx]! - (jp[i]! * jbar[j * 2]! + jp[n + i]! * jbar[j * 2 + 1]!);
  });
  return { n, lambda, jbar, proj, ok };
}

/** `taskSpace`, recomputed only when the snapshot changed (time, pose or — while paused — the inertia). */
export function cachedTaskSpace(snap: Snapshot): () => TaskSpace {
  let at = '';
  let value: TaskSpace | null = null;
  return () => {
    const stamp = `${snap.v('STEPS')}:${snap.v('Q')}:${snap.v('DET_M')}`;
    if (!value || stamp !== at) {
      value = taskSpace(snap);
      at = stamp;
    }
    return value;
  };
}
