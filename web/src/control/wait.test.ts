import { afterEach, describe, expect, it, vi } from 'vitest';
import { waitSimTime } from './wait';

afterEach(() => vi.useRealTimers());

describe('simulation-time waits', () => {
  it.each([0.25, 1, 2])('follows simulated time at playback speed %s', async (speed) => {
    vi.useFakeTimers();
    let simulated = 0;
    const clock = setInterval(() => {
      simulated += speed / 1000;
    }, 1);
    const abort = new AbortController();
    let finishedAt = 0;
    const waiting = waitSimTime(100, () => simulated, abort.signal).then(() => {
      finishedAt = simulated;
    });
    await vi.advanceTimersByTimeAsync(420 / speed);
    await waiting;
    clearInterval(clock);
    expect(finishedAt).toBeGreaterThanOrEqual(0.1 - 1e-9);
    expect(finishedAt).toBeLessThan(0.1 + speed * 0.009);
  });
  it('remains pending while paused and aborts immediately without another snapshot', async () => {
    vi.useFakeTimers();
    const abort = new AbortController();
    let completed = false;
    const waiting = waitSimTime(100, () => 0, abort.signal).then(() => {
      completed = true;
    });
    const rejected = expect(waiting).rejects.toThrow('cancelled');
    await vi.advanceTimersByTimeAsync(1000);
    expect(completed).toBe(false);
    abort.abort(new Error('cancelled'));
    await rejected;
    expect(vi.getTimerCount()).toBe(0);
  });
  it('rejects a time reset instead of waiting against an obsolete epoch', async () => {
    vi.useFakeTimers();
    let t = 2;
    const waiting = waitSimTime(100, () => t, new AbortController().signal);
    const rejected = expect(waiting).rejects.toThrow(/reset/);
    t = 0;
    await vi.advanceTimersByTimeAsync(8);
    await rejected;
    expect(vi.getTimerCount()).toBe(0);
  });
});
