/**
 * Plays an experiment: a clean slate (the page's controller as primary, no shadows, default parameters,
 * limits and goals), then its actions in order. It talks to the simulation through a `Port`, so the
 * browser (real time) and the verification tests (stepping the core directly) run the very same code.
 */
import { defaultSettings, pushSettings, SETTING_FLAGS, type Settings } from '../app/settings';
import type { Derivation } from '../model/derivation';
import type { Flag, Snapshot } from '../sim/layout';
import type { ToWorker } from '../sim/protocol';
import type { Action, Experiment } from './page';
import type { ControlStore } from './store';

export interface Port {
  d: Derivation;
  store: ControlStore;
  settings: Settings;
  send(msg: ToWorker): void;
  flag(f: Flag, v: number): void;
  /** The latest snapshot. */
  snap(): Snapshot;
  /** Let simulated time pass. */
  sleep(ms: number): Promise<void>;
  reset(): void;
  setUseTool(on: boolean): void;
  /** The push spring's anchor changed (for drawing it), or it was released. */
  setPush(at: [number, number] | null): void;
  /** Settings changed (views showing them may refresh). */
  settingsChanged?(): void;
  /** Timeline marks (kick, push, release, …) for whoever records the run. */
  mark?(event: string): void;
}

/** Joints a kick goes to: the passive ones (underactuated), else all. */
export function kickable(d: Derivation): number[] {
  const passive = d.coords.flatMap((c, i) => (c.actuated ? [] : [i]));
  return passive.length ? passive : d.coords.map((_, i) => i);
}

/** Set up `e` from a clean slate and play its actions; stops early when `alive()` turns false. */
export async function runExperiment(
  port: Port,
  kind: number,
  e: Experiment,
  alive: () => boolean,
): Promise<void> {
  const { store, settings } = port;
  port.setPush(null);
  port.flag('PushOn', 0);
  store.setPrimary(kind);
  store.clearShadows();
  store.resetParams(kind);
  const defaults = defaultSettings();
  for (const [key] of SETTING_FLAGS) settings[key] = defaults[key];
  settings.coriolis = true;
  pushSettings({ flag: (f, v) => port.flag(f, v) }, settings);
  port.setUseTool(false);
  port.settingsChanged?.();
  for (const a of e.actions) {
    if (!alive()) return;
    await act(port, a);
  }
  port.mark?.('done');
}

async function act(port: Port, a: Action): Promise<void> {
  const { store, settings, d } = port;
  switch (a.do) {
    case 'param': {
      const kind = store.primary;
      const i = store.spec(kind)?.params.findIndex((p) => p.key === a.key) ?? -1;
      if (i >= 0) store.setParam(kind, i, a.value);
      return;
    }
    case 'shadow': {
      const kind = store.byKey(a.key);
      if (kind >= 0 && store.reason(kind) === null && store.slotOf(kind) < 0) {
        store.resetParams(kind);
        store.toggleShadow(kind);
      }
      return;
    }
    case 'setting': {
      settings[a.key] = a.value;
      const flag = SETTING_FLAGS.find(([k]) => k === a.key)?.[1];
      if (flag) port.flag(flag, a.value);
      port.settingsChanged?.();
      return;
    }
    case 'coriolis':
      settings.coriolis = a.on;
      port.flag('Coriolis', a.on ? 1 : 0);
      return;
    case 'reset':
      port.reset();
      port.mark?.('reset');
      return;
    case 'reach': {
      const snap = port.snap();
      const [bx, by] = snap.frame(1);
      const [tx, ty] = [snap.v('TOOL_P'), snap.v('TOOL_P', 1)];
      port.flag('TargetX', bx + a.factor * (tx - bx));
      port.flag('TargetY', by + a.factor * (ty - by));
      port.setUseTool(true);
      port.mark?.('reach');
      return;
    }
    case 'stack': {
      // targets relative to where the tool and the elbow are now
      const snap = port.snap();
      const nb = d.bodies.length; // frames: s, bodies 1 … nb, tool nb + 1
      const set = (
        key: 'orient' | 'orientTarget' | 'elbow' | 'elbowX' | 'elbowY',
        flag: Parameters<Port['flag']>[0],
        v: number,
      ) => {
        settings[key] = v;
        port.flag(flag, v);
      };
      if (a.dphi !== undefined) {
        set('orientTarget', 'OrientTarget', snap.frame(nb + 1)[2] + a.dphi);
        set('orient', 'Orient', a.w);
      }
      if (a.delbow) {
        const [x, y] = snap.frame(Math.round(snap.v('GOALS', 5)) + 1);
        set('elbowX', 'ElbowX', x + a.delbow[0]);
        set('elbowY', 'ElbowY', y + a.delbow[1]);
        set('elbow', 'Elbow', a.w);
      }
      port.settingsChanged?.();
      port.mark?.('stack');
      return;
    }
    case 'hold': {
      port.reset();
      await port.sleep(20); // a fresh snapshot: the bend is relative to the reset pose, not the last one
      // bend the arm by 0.6 rad in all, shared by the joints after the first: a stretched arm is singular
      // and no task-space law holds a singular pose well (one sharp elbow for two links, a gentle curl for
      // seven — bending every joint fully would fold a long arm onto itself)
      if (!d.closures.length) {
        const s0 = port.snap();
        const first = d.coords.findIndex((c) => c.kind === 'revolute');
        const bend = d.coords.filter((c, i) => c.kind === 'revolute' && i > first).length;
        d.coords.forEach((c, i) => {
          if (c.kind === 'revolute' && i > first)
            port.send({ type: 'state', i, q: s0.v('Q', i) + 0.6 / bend, qd: 0 });
        });
      }
      await port.sleep(120); // the next snapshot shows the new pose
      const s = port.snap();
      port.flag('TargetX', s.v('TOOL_P'));
      port.flag('TargetY', s.v('TOOL_P', 1));
      port.setUseTool(true);
      port.mark?.('hold');
      return;
    }
    case 'hang': {
      port.reset();
      await port.sleep(20); // a fresh snapshot after the reset
      const s = port.snap();
      const first = d.coords.findIndex((c) => c.kind === 'revolute');
      d.coords.forEach((_, i) => {
        const q = s.v('Q_TARGET', i) + (i === first ? Math.PI : 0);
        port.send({ type: 'state', i, q, qd: 0 });
      });
      port.mark?.('hang');
      return;
    }
    case 'atTarget': {
      port.reset();
      await port.sleep(20);
      const s = port.snap();
      for (let i = 0; i < d.n; i++) port.send({ type: 'state', i, q: s.v('Q_TARGET', i), qd: 0 });
      port.mark?.('atTarget');
      return;
    }
    case 'kick':
      for (const i of kickable(d)) port.send({ type: 'kick', i, v: a.v });
      port.mark?.('kick');
      return;
    case 'push': {
      await port.sleep(300); // let the setup settle so the push starts from the current pose
      const s = port.snap();
      const [tx, ty] = [s.v('TOOL_P'), s.v('TOOL_P', 1)];
      // inward: towards the first joint, where a compliant arm has room (outwards it soon stretches singular)
      const [bx, by] = s.frame(1);
      const r = Math.hypot(bx - tx, by - ty) || 1;
      const at: [number, number] = a.inward
        ? [tx + (a.inward * (bx - tx)) / r, ty + (a.inward * (by - ty)) / r]
        : [tx + (a.dx ?? 0), ty + (a.dy ?? 0)];
      port.setPush(at);
      port.flag('PushX', at[0]);
      port.flag('PushY', at[1]);
      port.mark?.('push');
      await port.sleep(a.ms);
      port.setPush(null);
      port.flag('PushOn', 0);
      port.mark?.('release');
      return;
    }
    case 'wait':
      await port.sleep(a.ms);
      return;
  }
}
