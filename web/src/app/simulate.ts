/**
 * The Simulate stage: the controller dock and the shadow ghosts on the viewport, the side tabs (Control,
 * Goals, Equations, Model), the scope, pointer and keyboard interaction, and the experiment runner.
 * `main.ts` owns the app shell (stages, building, the frame loop) and delegates here while simulating.
 */
import { LABS } from '../control/lab';
import { type Experiment, pageContext } from '../control/page';
import { shortName } from '../control/pages';
import { kickable, type Port, runExperiment } from '../control/runner';
import type { ControlStore } from '../control/store';
import { liveSections } from '../eq/live';
import type { EquationPanel } from '../eq/panel';
import { tex } from '../eq/tex';
import type { Derivation } from '../model/derivation';
import { coordIndex, type Mechanism, normalise } from '../model/description';
import type { RobotView } from '../scene/robot';
import { presets } from '../scope/presets';
import { type PlotDef, Scope } from '../scope/scope';
import { Signals } from '../scope/signals';
import type { SimClient } from '../sim/client';
import type { Controllers } from '../sim/controllers';
import { segmented } from '../ui/widgets';
import { ControlTab, dock } from './control';
import { goalsTab, modelTab } from './goals';
import { ACTUATOR_KEYS, ACTUATOR_LABELS } from './keys';
import { pushSettings, type Settings, setSetting, toolTargets } from './settings';

type SideTab = 'control' | 'goals' | 'eq' | 'model';

const PLOT_HZ = 30;
const KICK = 1.5;

const $ = (id: string) => document.getElementById(id) as HTMLElement;
const sleep = (ms: number) => new Promise<void>((r) => window.setTimeout(r, ms));

/** What the app shell provides. */
export interface Shell {
  client: SimClient;
  /** Forget the tool paths (the shadows changed or the simulation restarted). */
  resetTrails(): void;
  /** Light up body i in the figure (−1: none). */
  highlight(body: number): void;
  panel: EquationPanel;
  store: ControlStore;
  settings: Settings;
  robot(): RobotView | null;
  mech(): Mechanism;
  params(): number[];
  running(): boolean;
  setRunning(on: boolean): void;
  flash(message: string): void;
}

export class SimulateStage {
  private d: Derivation | null = null;
  private signals: Signals | null = null;
  private tab: ControlTab | null = null;
  private readonly scope = new Scope($('plots'));
  private side: SideTab = 'control';
  private active = false;
  private dockCollapsed = false;
  private pushAt: [number, number] | null = null;
  private readonly held = new Set<string>();
  private runToken = 0;
  private lastPlot = 0;

  constructor(private readonly s: Shell) {
    s.store.onSlots(() => this.slotsChanged());
  }

  private get snap() {
    return this.s.client.snap;
  }

  /** A new mechanism was built (the robot view already exists). */
  load(d: Derivation, controllers: Controllers): void {
    const { store } = this.s;
    this.d = d;
    this.signals = new Signals(d, this.snap, store);
    this.tab = new ControlTab({
      store,
      d,
      snap: this.snap,
      panel: this.s.panel,
      run: (kind, e) => void this.run(kind, e),
      watch: (kind) => this.watch(kind),
    });
    if (!toolTargets(d)) this.s.settings.useTool = false;
    pushSettings(this.s.client, this.s.settings);
    store.load(controllers, defaultController(controllers));
    this.scope.clear();
  }

  // ---------------------------------------------------------------- mounting

  enter(head: HTMLElement): void {
    this.active = true;
    head.append(
      segmented(
        [
          ['control', 'Control'],
          ['goals', 'Goals'],
          ['eq', 'Equations'],
          ['model', 'Model'],
        ] as [SideTab, string][],
        this.side,
        (v) => {
          this.side = v;
          this.mountSide();
        },
        false,
      ),
    );
    this.mountSide();
    this.mountScope();
    this.renderDock();
    this.renderKeys();
  }

  leave(): void {
    this.active = false;
    this.release();
    $('dock').replaceChildren();
    $('keys').replaceChildren();
  }

  private mountSide(): void {
    const d = this.d;
    if (!d || !this.active) return;
    const { panel, client, settings } = this.s;
    panel.clear();
    const side = $('side');
    if (this.side === 'eq') panel.mount(liveSections({ d, snap: this.snap }));
    else if (this.side === 'model')
      side.replaceChildren(modelTab(d, this.s.params(), client, settings, (i) => this.s.highlight(i)));
    else if (this.side === 'goals') {
      side.replaceChildren(
        goalsTab({
          d,
          mech: this.s.mech(),
          snap: this.snap,
          client,
          settings,
          panel,
          onJointTarget: () => this.setUseTool(false),
          setUseTool: (on) => this.setUseTool(on),
          refresh: () => this.mountSide(),
        }),
      );
    } else if (this.tab) side.replaceChildren(this.tab.render());
  }

  private mountScope(prefer?: string): void {
    const d = this.d;
    if (!d || !this.signals) return;
    const { store } = this.s;
    const spec = store.spec(store.primary);
    const lab = spec && LABS[spec.key]?.(pageContext(d, this.snap, spec, (i) => store.param(spec.id, i)));
    this.scope.mount(this.signals, presets(d, store, lab?.watch ?? [], shortName(spec)), prefer);
  }

  private renderDock(): void {
    if (!this.active) return;
    $('dock').replaceChildren(
      dock(this.s.store, this.dockCollapsed, (c) => {
        this.dockCollapsed = c;
        this.renderDock();
      }),
    );
  }

  renderKeys(): void {
    const d = this.d;
    const keys = $('keys');
    keys.replaceChildren();
    if (!d || !this.active || !this.s.settings.keys) return;
    const parts = d.coords
      .filter((c) => c.actuated)
      .slice(0, ACTUATOR_LABELS.length)
      .map(
        (c, a) =>
          `<span><kbd>${ACTUATOR_LABELS[a]}</kbd><span data-tex="${encodeURIComponent(c.tex)}"></span></span>`,
      );
    if (toolTargets(d)) parts.push('<span><kbd>drag</kbd>tool target</span>');
    else if (this.s.mech().bodies.some((b) => b.actuated && b.joint === 'prismatic')) {
      parts.push('<span><kbd>click</kbd>rail target</span>');
    }
    if (d.tool) parts.push('<span><kbd>Shift</kbd>+<kbd>drag</kbd>push</span>');
    parts.push(
      '<span><kbd>1–9</kbd>primary</span><span><kbd>Shift</kbd>+<kbd>1–9</kbd>shadow</span><span><kbd>Space</kbd>kick</span><span><kbd>R</kbd>reset</span><span><kbd>P</kbd>pause</span>',
    );
    keys.innerHTML = parts.join('');
    for (const e of keys.querySelectorAll<HTMLElement>('[data-tex]'))
      tex(e, decodeURIComponent(e.dataset.tex ?? ''));
  }

  private setUseTool(on: boolean): void {
    const { settings, client } = this.s;
    if (settings.useTool === on) return;
    settings.useTool = on;
    client.flag('UseTool', on ? 1 : 0);
    if (this.side === 'control') this.mountSide();
  }

  /** The slot assignment changed: dock, plots, ghosts and the Control tab follow. */
  private slotsChanged(): void {
    if (!this.active) return;
    this.renderDock();
    this.mountScope();
    this.s.resetTrails();
    if (this.side === 'control') this.tab?.render();
  }

  // ---------------------------------------------------------------- experiments

  private watch(kind: number): void {
    const d = this.d;
    const spec = this.s.store.spec(kind);
    if (!d || !spec) return;
    const lab = LABS[spec.key]?.(pageContext(d, this.snap, spec, (i) => this.s.store.param(kind, i)));
    if (lab?.watch.length) this.scope.show(`watch:${spec.key}`, lab.watch, shortName(spec));
  }

  /** Set up an experiment from a clean slate, then play its steps. A newer run cancels this one. */
  private async run(kind: number, e: Experiment): Promise<void> {
    const d = this.d;
    const why = this.s.store.reason(kind);
    if (!d) return;
    if (why) return this.s.flash(why);
    const token = ++this.runToken;
    this.scope.show('experiment', e.plots as PlotDef[], e.title);
    this.scope.clearMarks();
    this.tab?.ran(kind, e.title);
    if (this.side !== 'control') {
      this.side = 'control';
      this.mountSide();
    }
    this.s.setRunning(true);
    const { client, store, settings } = this.s;
    const port: Port = {
      d,
      store,
      settings,
      send: (m) => client.send(m),
      flag: (f, v) => client.flag(f, v),
      snap: () => this.snap,
      sleep,
      reset: () => this.reset(),
      setUseTool: (on) => this.setUseTool(on),
      setPush: (at) => {
        this.pushAt = at;
      },
      settingsChanged: () => {
        if (this.side === 'goals') this.mountSide();
      },
      // the events a student would look for, on every plot; stamped a moment later so a fresh snapshot
      // (after a reset) carries the time
      mark: (event) => {
        if (!MARKED.has(event)) return;
        window.setTimeout(() => {
          if (token === this.runToken) this.scope.mark(event, this.snap.t);
        }, 30);
      },
    };
    await runExperiment(port, kind, e, () => token === this.runToken);
  }

  // ---------------------------------------------------------------- interaction

  reset(): void {
    this.s.client.send({ type: 'reset' });
    this.s.resetTrails();
  }

  private release(): void {
    if (!this.pushAt) return;
    this.pushAt = null;
    this.s.client.flag('PushOn', 0);
  }

  pointer(
    x: number,
    y: number,
    kind: 'down' | 'move' | 'up',
    pressed: boolean,
    shift: boolean,
    alt = false,
  ): void {
    const d = this.d;
    const robot = this.s.robot();
    const { client } = this.s;
    if (!d || !robot) return;
    // Shift-drag: push the tool with a spring towards the pointer
    if (this.pushAt || (shift && kind === 'down' && d.tool)) {
      if (kind === 'up' || !pressed) this.release();
      else {
        this.runToken++; // a manual push takes over from a running experiment
        this.pushAt = [x, y];
        client.flag('PushX', x);
        client.flag('PushY', y);
      }
      return;
    }
    if (!pressed || kind === 'up') return;
    // Alt-drag: the elbow target (switching the elbow goal on)
    if (alt && d.tool && !d.closures.length) {
      const { settings } = this.s;
      setSetting(client, settings, 'elbowX', x);
      setSetting(client, settings, 'elbowY', y);
      if (!(settings.elbow > 0)) {
        setSetting(client, settings, 'elbow', 6);
        if (this.side === 'goals') this.mountSide();
      }
      return;
    }
    if (toolTargets(d)) {
      client.flag('TargetX', x);
      client.flag('TargetY', y);
      this.setUseTool(true);
      return;
    }
    // project onto the rail of the first actuated prismatic joint
    const mech = this.s.mech();
    const i = mech.bodies.findIndex((b) => b.actuated && b.joint === 'prismatic');
    if (i < 0 || kind !== 'down') return;
    const b = normalise(mech.bodies[i]!);
    const [jx, jy, jp] = robot.jointPose(i, this.snap, this.s.params());
    const axis = jp + b.axisAngle;
    let target = (x - jx) * Math.cos(axis) + (y - jy) * Math.sin(axis);
    if (b.limit > 0) target = Math.max(-b.limit * 0.9, Math.min(b.limit * 0.9, target));
    client.send({ type: 'target', i: coordIndex(mech)[i]!, v: target });
  }

  key(e: KeyboardEvent, down: boolean): void {
    const d = this.d;
    const { client, store, settings } = this.s;
    const k = e.key.toLowerCase();
    if (down && e.repeat) {
      if (this.held.has(k)) e.preventDefault();
      return;
    }
    if (down) this.held.add(k);
    else this.held.delete(k);
    const a = ACTUATOR_KEYS.findIndex(([neg, pos]) => k === neg || k === pos);
    if (a >= 0 && d && a < d.nu) {
      // left = negative for prismatic joints, counter-clockwise (positive) for revolute ones
      const kind = d.coords.filter((c) => c.actuated)[a]?.kind;
      const sign = (k === ACTUATOR_KEYS[a]![0] ? -1 : 1) * (kind === 'revolute' ? -1 : 1);
      client.send({ type: 'input', i: a, v: down ? sign * settings.key : 0 });
      e.preventDefault();
      return;
    }
    if (!down || !d) return;
    if (k === ' ') {
      for (const i of kickable(d)) client.send({ type: 'kick', i, v: (Math.random() < 0.5 ? -1 : 1) * KICK });
      e.preventDefault();
    } else if (k === 'r') this.reset();
    else if (k === 'p') this.s.setRunning(!this.s.running());
    else if (/^digit[1-9]$/.test(e.code.toLowerCase())) {
      const spec = store.usable()[Number(e.code.slice(5)) - 1];
      if (spec && e.shiftKey) store.toggleShadow(spec.id);
      else if (spec) store.setPrimary(spec.id);
    }
  }

  /** Release every held actuator key (window blur). */
  releaseKeys(): void {
    for (let a = 0; a < ACTUATOR_KEYS.length; a++) this.s.client.send({ type: 'input', i: a, v: 0 });
    this.held.clear();
  }

  /** The theme changed: redraw what carries slot inks (dock, plots, comparison). */
  restyle(): void {
    if (!this.active) return;
    this.renderDock();
    this.mountScope();
    if (this.side === 'control') this.tab?.render();
  }

  /** Show the recorded moment `t` on the plots (rewinding), or null: now. */
  setCursor(t: number | null): void {
    this.scope.cursor = t;
  }

  /** Where the tool is being pushed towards (Shift-drag), or null. */
  get pushPoint(): [number, number] | null {
    return this.active ? this.pushAt : null;
  }

  // ---------------------------------------------------------------- frame

  frame(now: number, refresh: boolean): void {
    if (!this.active) return;
    if (now - this.lastPlot >= 1000 / PLOT_HZ) {
      this.scope.sample(this.snap.t);
      this.scope.draw();
      this.lastPlot = now;
    }
    if (refresh && this.side === 'control') this.tab?.update();
  }
}

/** Experiment events drawn on the plots. */
const MARKED = new Set(['kick', 'push', 'release', 'reach', 'stack']);

/** Computed torque where every DOF is actuated, LQR for underactuated mechanisms, else passive. */
function defaultController(reg: Controllers): number {
  for (const key of ['computed_torque', 'lqr']) {
    const spec = reg.specs.find((s) => s.key === key);
    if (spec && reg.applicable[spec.id] === null) return spec.id;
  }
  return 0;
}
