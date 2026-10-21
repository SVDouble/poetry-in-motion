/**
 * Rewind: the last 20 s of the simulation, kept as snapshots at 30 Hz (as Float32: about 5 MB). While
 * paused, a timeline under the figure scrubs through them — the figure, the gauges and the readouts show
 * that moment — steps frame by frame (at the live end, the physics advances one frame), and resumes the
 * simulation from any recorded moment.
 */
import { type Layout, Snapshot } from '../sim/layout';
import { button, el } from '../ui/widgets';

const HZ = 30;
const SECONDS = 20;
const CAP = HZ * SECONDS;

export interface TimelineEvents {
  /** Advance the physics by `steps` (1 ms each) while paused. */
  advance(steps: number): void;
  /** Continue the simulation from snapshot `s` (its q and q̇). */
  resume(s: Snapshot): void;
}

export class Timeline {
  readonly el = el('div', 'timeline overlay');
  private readonly frames: Float32Array[] = [];
  private head = 0;
  private count = 0;
  private lastT = Number.NaN;
  /** Index (0 = oldest) of the moment being shown, or −1: live. */
  private at = -1;
  private readonly replay: Snapshot;
  private readonly slider = el('input');
  private readonly time = el('span', 'timeline-time');

  constructor(
    layout: Layout,
    private readonly events: TimelineEvents,
  ) {
    this.replay = new Snapshot(layout);
    for (let k = 0; k < CAP; k++) this.frames.push(new Float32Array(layout.len));
    this.slider.type = 'range';
    this.slider.min = '0';
    this.slider.addEventListener('input', () => this.show(Number(this.slider.value)));
    this.el.append(
      button('⏮', () => this.show(0), 'btn small', 'the oldest recorded moment'),
      button('◀', () => this.step(-1), 'btn small', 'one frame back'),
      this.slider,
      button('▶', () => this.step(1), 'btn small', 'one frame on (at the end: simulate one frame)'),
      button('⏭', () => this.show(-1), 'btn small', 'back to now'),
      this.time,
      button(
        'Resume here',
        () => this.resumeHere(),
        'btn small primary',
        'continue the simulation from this moment',
      ),
    );
    this.el.hidden = true;
  }

  /** Forget everything (new mechanism, reset). */
  clear(): void {
    this.count = 0;
    this.head = 0;
    this.at = -1;
    this.lastT = Number.NaN;
  }

  /** Keep the live snapshot if 1/30 s of simulated time has passed (time going back: a reset). */
  record(snap: Snapshot): void {
    const t = snap.t;
    if (t < this.lastT) this.clear();
    if (!(t - this.lastT < 1 / HZ - 1e-9)) {
      this.frames[this.head]!.set(snap.data);
      this.head = (this.head + 1) % CAP;
      this.count = Math.min(CAP, this.count + 1);
      this.lastT = t;
    }
  }

  /** The snapshot to draw: the recorded moment being shown, or the live one. */
  view(live: Snapshot): Snapshot {
    return this.at < 0 ? live : this.replay;
  }

  get replaying(): boolean {
    return this.at >= 0;
  }

  /** Visible while paused in the simulation and there is something to look back at. */
  setVisible(on: boolean): void {
    const show = on && this.count > 1;
    if (!show) this.at = -1;
    if (show && this.el.hidden) this.sync();
    this.el.hidden = !show;
  }

  private frame(i: number): Float32Array {
    return this.frames[(this.head - this.count + i + CAP) % CAP]!;
  }

  private show(i: number): void {
    this.at = i < 0 || i >= this.count - 1 ? -1 : i;
    if (this.at >= 0) this.replay.data.set(this.frame(this.at));
    this.sync();
  }

  private step(dir: number): void {
    const i = this.at < 0 ? this.count - 1 : this.at;
    if (dir > 0 && this.at < 0) {
      this.events.advance(1000 / HZ);
      return;
    }
    this.show(Math.max(0, i + dir));
  }

  private resumeHere(): void {
    if (this.at < 0) return;
    // the moments after this one are a future that will not happen
    const keep = this.at + 1;
    this.head = (this.head - this.count + keep + CAP) % CAP;
    this.count = keep;
    this.lastT = this.replay.t;
    this.events.resume(this.replay);
    this.at = -1;
    this.sync();
  }

  private sync(): void {
    this.slider.max = String(Math.max(0, this.count - 1));
    this.slider.value = String(this.at < 0 ? this.count - 1 : this.at);
    const t = this.at < 0 ? this.lastT : this.replay.t;
    this.time.textContent = Number.isFinite(t) ? `t = ${t.toFixed(2)} s${this.at < 0 ? ' (now)' : ''}` : '';
  }

  /** Keep the slider at the end while live. */
  tick(): void {
    if (!this.el.hidden && this.at < 0) this.sync();
  }
}
