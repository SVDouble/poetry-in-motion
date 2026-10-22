/** Wait for simulated motion, independent of playback speed or wall-clock stalls. */
export function waitSimTime(ms: number, time: () => number, signal: AbortSignal): Promise<void> {
  return new Promise((resolve, reject) => {
    if (signal.aborted) {
      reject(signal.reason);
      return;
    }
    const start = time();
    const target = start + ms / 1000;
    const finish = (error?: unknown) => {
      clearInterval(timer);
      signal.removeEventListener('abort', abort);
      if (error !== undefined) reject(error);
      else resolve();
    };
    const abort = () => finish(signal.reason);
    const timer = setInterval(() => {
      const now = time();
      if (now < start - 1e-9) finish(new Error('Simulation reset during experiment wait'));
      else if (now >= target - 1e-9) finish();
    }, 8);
    signal.addEventListener('abort', abort, { once: true });
  });
}
