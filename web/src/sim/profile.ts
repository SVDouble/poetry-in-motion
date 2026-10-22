/** Bounded, opt-in measurements of actual worker ticks, including publication and checkpoint retention. */
export const MAX_PROFILE_TICKS = 5000;
export const PHASES = ['integration', 'publication', 'transport', 'history', 'benchmark', 'total'] as const;
export type Phase = (typeof PHASES)[number];
export interface TickProfile {
  ticks: number;
  steps: number;
  shared: boolean;
  us: Record<Phase, { mean: number; p95: number; max: number }>;
}
export class TickProfiler {
  private samples: number[][] = PHASES.map(() => []);
  private steps = 0;
  constructor(
    readonly count: number,
    readonly shared: boolean,
  ) {}
  add(values: Record<Phase, number>, steps: number): TickProfile | null {
    this.steps += steps;
    PHASES.forEach((key, i) => {
      this.samples[i]!.push(values[key]);
    });
    if (this.samples[0]!.length < this.count) return null;
    const us = {} as TickProfile['us'];
    PHASES.forEach((key, i) => {
      const s = this.samples[i]!.sort((a, b) => a - b);
      us[key] = {
        mean: s.reduce((a, b) => a + b, 0) / s.length,
        p95: s[Math.ceil(s.length * 0.95) - 1]!,
        max: s.at(-1)!,
      };
    });
    return { ticks: this.count, steps: this.steps, shared: this.shared, us };
  }
}
