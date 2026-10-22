/** Exact rewind from worker-owned execution checkpoints, retained within a frame and memory budget. */
import { HISTORY } from '../generated/contracts';
import { type Layout, Snapshot } from '../sim/layout';
import type { CheckpointFrame } from '../sim/protocol';
import { button, el } from '../ui/widgets';
import { Checkpoints } from './checkpoints';

export interface TimelineEvents {
  /** Advance the physics by `steps` of the core’s fixed timestep while paused. */
  advance(steps: number): void;
  /** Restore the selected worker checkpoint. Expired checkpoints leave the live world untouched. */
  resume(checkpoint: number): Promise<boolean>;
}

export class Timeline {
  readonly el = el('div', 'timeline overlay');
  private readonly frames: Checkpoints;
  private busy = false;
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
    this.frames = new Checkpoints(layout.off.TIME);
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
        () => void this.resumeHere(),
        'btn small primary',
        'continue the simulation from this moment',
      ),
    );
    this.el.hidden = true;
  }

  /** Forget everything (new mechanism, reset). */
  clear(): void {
    this.frames.clear();
    this.at = -1;
    this.lastT = Number.NaN;
  }

  record(frame: CheckpointFrame): void {
    this.frames.record(frame);
    this.lastT = frame.data[this.replay.layout.off.TIME]!;
  }

  /** The snapshot to draw: the recorded moment being shown, or the live one. */
  view(live: Snapshot): Snapshot {
    return this.at < 0 ? live : this.replay;
  }

  viewParams(live: number[]): number[] {
    return this.at < 0 ? live : (this.frames.get(this.at)?.params ?? live);
  }

  viewPush(live: [number, number] | null): [number, number] | null {
    return this.at < 0 ? live : (this.frames.get(this.at)?.push ?? null);
  }

  get replaying(): boolean {
    return this.at >= 0;
  }

  /** Visible while paused in the simulation and there is something to look back at. */
  setVisible(on: boolean): void {
    const show = on && this.frames.count > 1;
    if (!show) this.at = -1;
    if (show && this.el.hidden) this.sync();
    this.el.hidden = !show;
  }

  private show(i: number): void {
    this.at = i < 0 || i >= this.frames.count - 1 ? -1 : i;
    if (this.at >= 0) this.replay.data.set(this.frames.get(this.at)!.data);
    this.sync();
  }

  private step(dir: number): void {
    const i = this.at < 0 ? this.frames.count - 1 : this.at;
    if (dir > 0 && this.at < 0) {
      this.events.advance(Math.max(1, Math.round(1 / (HISTORY.hz * this.replay.layout.dt))));
      return;
    }
    this.show(Math.max(0, i + dir));
  }

  private async resumeHere(): Promise<void> {
    if (this.at < 0 || this.busy) return;
    const at = this.at;
    const frame = this.frames.get(at);
    if (!frame) return;
    this.busy = true;
    try {
      if (!(await this.events.resume(frame.checkpoint))) return;
      this.frames.truncate(at);
      this.lastT = this.replay.t;
      this.at = -1;
      this.sync();
    } finally {
      this.busy = false;
    }
  }

  private sync(): void {
    this.slider.max = String(Math.max(0, this.frames.count - 1));
    this.slider.value = String(this.at < 0 ? this.frames.count - 1 : this.at);
    const t = this.at < 0 ? this.lastT : this.replay.t;
    this.time.textContent = Number.isFinite(t) ? `t = ${t.toFixed(2)} s${this.at < 0 ? ' (now)' : ''}` : '';
  }

  /** Keep the slider at the end while live. */
  tick(): void {
    if (!this.el.hidden && this.at < 0) this.sync();
  }
}
