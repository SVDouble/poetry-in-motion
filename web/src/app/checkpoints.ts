import { HISTORY } from '../generated/contracts';
import type { CheckpointFrame } from '../sim/protocol';

/** A bounded list of transferred observations; snapshot buffers are never copied here. */
export class Checkpoints {
  private frames: CheckpointFrame[] = [];
  constructor(private readonly timeOffset: number) {}
  get count(): number {
    return this.frames.length;
  }
  clear(): void {
    this.frames.length = 0;
  }
  get(index: number): CheckpointFrame | undefined {
    return this.frames[index];
  }
  record(frame: CheckpointFrame): void {
    const latest = this.frames.at(-1);
    if (
      latest &&
      (frame.generation !== latest.generation || frame.data[this.timeOffset]! < latest.data[this.timeOffset]!)
    )
      this.clear();
    while (this.frames.length && (this.frames[0]!.checkpoint - frame.oldest) >>> 0 >= 0x80000000)
      this.frames.shift();
    this.frames.push(frame);
    // At most 600 references move; the transferred snapshot data stays in place.
    if (this.frames.length > HISTORY.frames) this.frames.shift();
  }
  truncate(index: number): void {
    this.frames.splice(index + 1);
  }
}
