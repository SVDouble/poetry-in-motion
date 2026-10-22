/** Bounded seqlock reader. Failed reads never modify an accepted observation. */
import { DATA_OFFSET, STATS, STATS_OFFSET } from './protocol';

export class SharedReader {
  private readonly seq: Int32Array;
  private readonly source: Float64Array;
  private readonly sourceStats: Float64Array;
  readonly data: Float64Array<ArrayBuffer>;
  readonly stats = new Float64Array(STATS);
  sequence = 0;
  private lastSeq: number | null = null;

  constructor(buffer: SharedArrayBuffer, len: number) {
    this.seq = new Int32Array(buffer, 0, 2);
    this.source = new Float64Array(buffer, DATA_OFFSET, len);
    this.sourceStats = new Float64Array(buffer, STATS_OFFSET, STATS);
    this.data = new Float64Array(len);
  }

  reset(): void {
    this.lastSeq = null;
  }

  read(generation: number): boolean {
    for (let attempt = 0; attempt < 4; attempt++) {
      const seq = Atomics.load(this.seq, 0);
      if (seq & 1) continue;
      if (seq === this.lastSeq) return false;
      const gen = Atomics.load(this.seq, 1);
      this.data.set(this.source);
      this.stats.set(this.sourceStats);
      if (Atomics.load(this.seq, 0) === seq) {
        if (gen !== generation) return false;
        this.lastSeq = seq;
        this.sequence = seq >>> 0;
        return true;
      }
    }
    return false;
  }
}
