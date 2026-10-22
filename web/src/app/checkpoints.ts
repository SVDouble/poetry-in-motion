import { HISTORY } from '../generated/contracts';
import type { CheckpointFrame } from '../sim/protocol';

/** Lazy snapshot retention: slots hold transferred doubles only after an actual checkpoint arrives. */
export class Checkpoints {
  private frames: (CheckpointFrame | undefined)[] = Array(HISTORY.frames);
  constructor(private readonly timeOffset: number) {}
  private head = 0;
  count = 0;
  private generation = -1;
  clear(): void {
    this.frames = Array(HISTORY.frames);
    this.head = 0;
    this.count = 0;
  }
  get(index: number): CheckpointFrame | undefined {
    if (index < 0 || index >= this.count) return undefined;
    return this.frames[(this.head - this.count + index + HISTORY.frames) % HISTORY.frames];
  }
  record(frame: CheckpointFrame): void {
    const latest = this.get(this.count - 1);
    if (
      frame.generation !== this.generation ||
      (latest && frame.data[this.timeOffset]! < latest.data[this.timeOffset]!)
    )
      this.clear();
    this.generation = frame.generation;
    while (this.count && (this.get(0)!.checkpoint - frame.oldest) >>> 0 >= 0x80000000) {
      const index = (this.head - this.count + HISTORY.frames) % HISTORY.frames;
      this.frames[index] = undefined;
      this.count--;
    }
    this.frames[this.head] = frame;
    this.head = (this.head + 1) % HISTORY.frames;
    this.count = Math.min(HISTORY.frames, this.count + 1);
  }
  truncate(index: number): void {
    const keep = index + 1;
    for (let i = keep; i < this.count; i++)
      this.frames[(this.head - this.count + i + HISTORY.frames) % HISTORY.frames] = undefined;
    this.head = (this.head - this.count + keep + HISTORY.frames) % HISTORY.frames;
    this.count = keep;
  }
}
