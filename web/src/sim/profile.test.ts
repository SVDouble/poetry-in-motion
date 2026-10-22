import { expect, it } from 'vitest';
import { PHASES, type Phase, TickProfiler } from './profile';

it('reports microsecond phase distributions and actual step counts over the requested ticks', () => {
  const p = new TickProfiler(20, false);
  for (let i = 1; i <= 20; i++) {
    const values = Object.fromEntries(PHASES.map((phase, k) => [phase, i * (k + 1)])) as Record<
      Phase,
      number
    >;
    const result = p.add(values, i);
    if (i < 20) expect(result).toBeNull();
    else {
      expect(result).toMatchObject({ ticks: 20, steps: 210, shared: false });
      expect(result!.us.integration).toEqual({ mean: 10.5, p95: 19, max: 20 });
      expect(result!.us.total).toEqual({ mean: 63, p95: 114, max: 120 });
    }
  }
});
