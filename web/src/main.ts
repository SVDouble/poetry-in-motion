import 'katex/dist/katex.min.css';
import './styles.css';

import { ControlView } from './app/control';
import { defaultSettings, goalsTab, modelTab, pushSettings, type Settings, toolTargets } from './app/goals';
import { History } from './app/history';
import { ACTUATOR_KEYS, ACTUATOR_LABELS } from './app/keys';
import { closeLoop, type DragContext, dragJoint, dragTip } from './app/manipulate';
import { plotSpecs } from './app/plotspecs';
import { Editor } from './builder/editor';
import { Gizmos } from './builder/gizmos';
import { ControlStore, SLOT_COLORS } from './control/store';
import { derivationSections } from './eq/derivation';
import { liveSections } from './eq/live';
import { EquationPanel } from './eq/panel';
import { tex } from './eq/tex';
import type { Derivation } from './model/derivation';
import {
  cloneMechanism,
  coordIndex,
  DEFAULT_PRESET,
  encode,
  type JointKind,
  MAX_CLOSURES,
  type Mechanism,
  migrate,
  normalise,
  PRESETS,
  paramVector,
} from './model/description';
import { Plots } from './plots/plots';
import { GhostView } from './scene/ghost';
import { RobotView } from './scene/robot';
import { Stage } from './scene/stage';
import { SimClient } from './sim/client';
import type { Controllers } from './sim/controllers';
import { button, portLabel, segmented } from './ui/widgets';

type StageName = 'build' | 'derive' | 'simulate';
type SideTab = 'control' | 'goals' | 'eq' | 'model';
type ConnectKind = 'pin' | 'slot';

const EQ_HZ = 30;
const PLOT_HZ = 30;
const TIP_OFFSET_PX = 22;

const $ = <T extends HTMLElement = HTMLElement>(id: string) => document.getElementById(id) as T;

const encodeHash = (m: Mechanism) => `#m=${btoa(encodeURIComponent(JSON.stringify(m)))}`;

function decodeHash(): Mechanism | null {
  const match = /^#m=(.+)$/.exec(location.hash);
  if (!match?.[1]) return null;
  try {
    return migrate(JSON.parse(decodeURIComponent(atob(match[1]))));
  } catch {
    return null;
  }
}

class App {
  private stageName: StageName = 'build';
  private mech: Mechanism;
  private params: number[];
  private derivation: Derivation | null = null;
  private robot: RobotView | null = null;
  private ghosts: GhostView[] = [];
  private control: ControlView | null = null;
  private readonly store: ControlStore;
  private side: SideTab = 'control';
  private running = false;
  private dragging = false;
  private pushAt: [number, number] | null = null;
  private popover: HTMLElement | null = null;
  private readonly settings: Settings = defaultSettings();
  private readonly history = new History<Mechanism>(cloneMechanism);
  private readonly editor: Editor;
  private readonly gizmos: Gizmos;
  private readonly panel = new EquationPanel($('side'));
  private readonly plots = new Plots($('plots'));
  private readonly held = new Set<string>();
  private buildTimer = 0;
  private buildSeq = 0;
  private lastBuildMs = 0;
  private readonly clock = { eq: 0, plot: 0, hud: 0, frames: 0 };

  constructor(
    private readonly client: SimClient,
    private readonly stage: Stage,
  ) {
    this.mech = decodeHash() ?? PRESETS[DEFAULT_PRESET]!();
    this.params = paramVector(this.mech);
    this.store = new ControlStore(client, client.layout.slots);
    this.store.onSlots(() => this.slotsChanged());
    this.editor = new Editor(document.createElement('div'), {
      change: (m) => this.edited(m),
      select: (i) => {
        this.robot?.select(i);
        this.gizmos.select(i);
        this.closePopover();
      },
      connect: (body, s, kind) => this.startConnect(body, s, kind),
      flash: (msg) => this.flash(msg),
    });
    this.gizmos = new Gizmos($('viewport'), stage, {
      port: (body, s, at) => this.openPortMenu(body, s, at),
      connect: (body, s, world) => this.finishConnect(body, s, world),
      dragStart: () => {
        this.dragging = true;
        this.history.record(this.mech, true);
      },
      dragJoint: (w, free) => this.applyDrag(dragJoint(this.dragContext(), this.editor.selection, w, free)),
      dragTip: (w, free, initial) =>
        this.applyDrag(dragTip(this.dragContext(), this.editor.selection, w, free, initial)),
      dragEnd: () => {
        this.dragging = false;
        this.editor.load(this.mech);
      },
    });
  }

  private get snap() {
    return this.client.snap;
  }

  async start(): Promise<void> {
    this.chrome();
    this.editor.load(this.mech);
    await this.rebuild();
    this.setStage('build');
    const loop = (now: number) => {
      this.frame(now);
      requestAnimationFrame(loop);
    };
    requestAnimationFrame(loop);
  }

  // ---------------------------------------------------------------- building & history

  /** A new document from the editor or a drag. Drags record history once, at their start. */
  private edited(m: Mechanism): void {
    if (!this.dragging) this.history.record(this.mech);
    this.mech = m;
    window.clearTimeout(this.buildTimer);
    this.buildTimer = window.setTimeout(() => void this.rebuild(), this.dragging ? 10 : 60);
  }

  private undo(redo = false): void {
    const m = this.history.step(this.mech, redo);
    if (!m) return;
    this.mech = m;
    this.editor.load(m);
    void this.rebuild();
  }

  private dragContext(): DragContext {
    return {
      mech: this.mech,
      robot: this.robot!,
      snap: this.snap,
      params: this.params,
      tipOffset: TIP_OFFSET_PX * this.stage.metresPerPixel,
    };
  }

  private applyDrag(m: Mechanism | null): void {
    if (m && this.robot) this.edited(m);
  }

  private async rebuild(): Promise<void> {
    const seq = ++this.buildSeq;
    const res = await this.client.build(encode(this.mech));
    if (seq !== this.buildSeq) return; // a newer edit superseded this build
    if (!res.ok) return this.status(res.error, 'build');
    if ($('status').dataset.kind === 'build') $('status').hidden = true;
    this.derivation = res.derivation;
    this.lastBuildMs = res.ms;
    this.params = paramVector(this.mech);
    history.replaceState(null, '', encodeHash(this.mech));
    if (!toolTargets(res.derivation)) this.settings.useTool = false;
    pushSettings(this.client, this.settings);

    this.stage.clear();
    this.robot = new RobotView(this.stage, this.mech);
    this.ghosts = SLOT_COLORS.slice(1).map((c) => new GhostView(this.stage, this.mech, c));
    this.stage.castShadows();
    this.robot.showFrames = this.settings.frames;
    this.robot.showTrail = this.settings.trail;
    this.control = new ControlView(this.store, res.derivation, this.snap, this.panel);
    this.store.load(res.controllers, defaultController(res.controllers));
    if (!this.dragging) {
      const b = this.robot.bounds();
      this.stage.setView(b.cx, b.cy, b.width, b.height);
    }
    const sel = this.stageName === 'build' ? this.editor.selection : -1;
    this.robot.select(sel);
    this.gizmos.select(sel);
    // the loop closures are solved numerically from q(0): report when they cannot close
    window.setTimeout(() => {
      const gap = this.snap.v('CONSTRAINT');
      if (this.snap.v('NC') > 0 && gap > 1e-3) {
        this.status(
          `A loop cannot close in this pose (gap ${gap.toFixed(3)} m): adjust lengths, ports or initial angles.`,
          'build',
        );
      }
    }, 80);
    if (!this.dragging) this.refreshStage();
  }

  private status(message: string, kind: 'build' | 'flash'): void {
    const st = $('status');
    st.dataset.kind = kind;
    st.textContent = message;
    st.hidden = false;
  }

  private flash(message: string): void {
    this.status(message, 'flash');
    window.setTimeout(() => {
      if ($('status').textContent === message) $('status').hidden = true;
    }, 2600);
  }

  // ---------------------------------------------------------------- ports, loops & popover

  private closePopover(): void {
    this.popover?.remove();
    this.popover = null;
  }

  private openPortMenu(body: number, s: number, [x, y]: [number, number]): void {
    this.closePopover();
    const pop = document.createElement('div');
    pop.className = 'popover overlay';
    Object.assign(pop.style, { left: `${x + 14}px`, top: `${y - 10}px` });
    const title = document.createElement('div');
    title.className = 'popover-title';
    title.textContent = `Body ${body + 1} · ${portLabel(s)}`;
    pop.append(title);
    const act = (label: string, run: () => void, disabled = false) => {
      const b = button(label, () => {
        this.closePopover();
        run();
      });
      b.disabled = disabled;
      pop.append(b);
    };
    for (const kind of ['revolute', 'prismatic', 'fixed'] as JointKind[]) {
      act(`+ ${kind} child here`, () => this.editor.addChild(body, kind, s));
    }
    const full = this.mech.closures.length >= MAX_CLOSURES;
    act('pin a loop from here…', () => this.startConnect(body, s, 'pin'), full);
    act('slide in a slot from here…', () => this.startConnect(body, s, 'slot'), full);
    if (s === 1) {
      act(this.mech.tool === body ? 'tool is here' : 'make this the tool', () => {
        const m = cloneMechanism(this.mech);
        m.tool = body;
        this.editor.commit(m);
      });
    }
    $('viewport').append(pop);
    this.popover = pop;
  }

  private connectKind: ConnectKind = 'pin';

  private startConnect(body: number, s: number, kind: ConnectKind): void {
    this.closePopover();
    this.connectKind = kind;
    this.gizmos.connectFrom = { body, s };
    const what = kind === 'pin' ? 'pin it to' : 'let it slide in a slot on';
    this.flash(`Click a port of another body to ${what}, or empty space for the ground. Esc cancels.`);
  }

  private finishConnect(body: number, s: number, world: [number, number]): void {
    const from = this.gizmos.connectFrom;
    this.gizmos.connectFrom = null;
    if (from) this.editor.commit(closeLoop(this.mech, { ...from, kind: this.connectKind }, body, s, world));
  }

  // ---------------------------------------------------------------- stages

  private setStage(s: StageName): void {
    this.stageName = s;
    for (const b of $('stages').querySelectorAll<HTMLButtonElement>('button')) {
      b.classList.toggle('on', b.dataset.stage === s);
    }
    document.body.dataset.stage = s;
    this.setRunning(s === 'simulate');
    if (s !== 'simulate') this.reset();
    const sel = s === 'build' ? this.editor.selection : -1;
    this.robot?.select(sel);
    this.gizmos.select(sel);
    this.gizmos.visible = s === 'build';
    this.gizmos.connectFrom = null;
    this.closePopover();
    this.refreshStage();
  }

  /** Re-render the side panel and overlays for the current stage. */
  private refreshStage(): void {
    const d = this.derivation;
    const head = $('side-head');
    head.replaceChildren();
    this.panel.clear();
    if (!d) return;
    const next = (label: string, to: StageName) => button(label, () => this.setStage(to), 'btn primary');
    const title = (t: string, sub: string) => {
      const el = document.createElement('div');
      el.className = 'side-title';
      el.innerHTML = `<b>${t}</b><span>${sub}</span>`;
      return el;
    };
    const constraints = d.phi.length;
    const ms = `${this.lastBuildMs.toFixed(1)} ms`;

    if (this.stageName === 'build') {
      const tools = document.createElement('div');
      tools.className = 'head-tools';
      const undo = button('↶', () => this.undo(), 'btn small', 'undo (Ctrl+Z)');
      const redo = button('↷', () => this.undo(true), 'btn small', 'redo (Ctrl+Shift+Z)');
      undo.disabled = !this.history.canUndo;
      redo.disabled = !this.history.canRedo;
      tools.append(undo, redo, next('Derive equations →', 'derive'));
      const dof = constraints ? `${d.n} − ${constraints} = ${d.n - constraints} DOF` : `${d.n} DOF`;
      head.append(
        title('Build', `${this.mech.bodies.length} bodies · ${dof} · ${d.nu} actuated · derived in ${ms}`),
        tools,
      );
      $('side').replaceChildren(this.editor.host);
    } else if (this.stageName === 'derive') {
      head.append(
        title('Derivation', `Lagrange's method for this topology · ${ms}`),
        next('Simulate →', 'simulate'),
      );
      this.panel.mount(derivationSections(this.mech, d));
    } else {
      head.append(title('Simulation', `${d.n - constraints} DOF · 1 kHz RK4`), this.sideTabs());
      this.mountSide();
      this.plots.mount(plotSpecs(d, this.snap, this.store));
    }
    this.buildOverlay();
    this.buildToggles();
    this.buildKeys();
  }

  private sideTabs(): HTMLElement {
    const tabs: [SideTab, string][] = [
      ['control', 'Control'],
      ['goals', 'Goals'],
      ['eq', 'Equations'],
      ['model', 'Model'],
    ];
    return segmented(
      tabs,
      this.side,
      (v) => {
        this.side = v;
        this.mountSide();
      },
      false,
    );
  }

  private mountSide(): void {
    const d = this.derivation;
    if (!d || this.stageName !== 'simulate') return;
    this.panel.clear();
    const side = $('side');
    if (this.side === 'eq') this.panel.mount(liveSections({ d, snap: this.snap }));
    else if (this.side === 'model')
      side.replaceChildren(modelTab(d, this.params, this.client, this.settings));
    else if (this.side === 'goals') {
      side.replaceChildren(
        goalsTab({
          d,
          mech: this.mech,
          snap: this.snap,
          client: this.client,
          settings: this.settings,
          panel: this.panel,
          onJointTarget: () => this.setUseTool(false),
          setUseTool: (on) => this.setUseTool(on),
        }),
      );
    } else if (this.control) side.replaceChildren(this.control.tab());
  }

  private setUseTool(on: boolean): void {
    if (this.settings.useTool === on) return;
    this.settings.useTool = on;
    this.client.flag('UseTool', on ? 1 : 0);
    if (this.side === 'control') this.mountSide();
  }

  /** The slot assignment changed: overlay, plots, ghosts and the Control tab follow. */
  private slotsChanged(): void {
    const d = this.derivation;
    if (!d || this.stageName !== 'simulate') return;
    this.buildOverlay();
    this.plots.mount(plotSpecs(d, this.snap, this.store));
    for (const g of this.ghosts) g.resetTrail();
    if (this.side === 'control') this.mountSide();
  }

  // ---------------------------------------------------------------- chrome

  private buildOverlay(): void {
    const [chips, quick, hint] = [$('modes'), $('quick'), $('hint')];
    chips.replaceChildren();
    quick.replaceChildren();
    if (!this.derivation || this.stageName !== 'simulate' || !this.control) {
      hint.textContent =
        this.stageName === 'build'
          ? 'Click a body to select it. Steel hinges are actuated, amber ones passive; blue pins close loops.'
          : 'Every formula on the right was derived symbolically for exactly this structure.';
      return;
    }
    this.control.overlay(chips, quick, hint);
  }

  private buildToggles(): void {
    const host = $('toggles');
    host.replaceChildren();
    const t = this.settings;
    const items: ['frames' | 'trail' | 'coriolis' | 'ghosts', string, () => void][] = [
      [
        'frames',
        'Frames',
        () => {
          if (this.robot) this.robot.showFrames = t.frames;
        },
      ],
      [
        'trail',
        'Trail',
        () => {
          if (!this.robot) return;
          this.robot.showTrail = t.trail;
          this.robot.resetTrail();
        },
      ],
      [
        'ghosts',
        'Shadows',
        () => {
          for (const g of this.ghosts) g.visible = t.ghosts;
        },
      ],
      ['coriolis', 'Coriolis', () => this.client.flag('Coriolis', t.coriolis ? 1 : 0)],
    ];
    for (const [key, label, apply] of items) {
      if ((key === 'coriolis' || key === 'ghosts') && this.stageName !== 'simulate') continue;
      const b = button(
        label,
        () => {
          t[key] = !t[key];
          b.classList.toggle('on', t[key]);
          apply();
        },
        'chip',
      );
      b.classList.toggle('on', t[key]);
      if (key === 'coriolis')
        b.title = 'Switch off C(q, q̇)q̇ in the simulation (non-physical: watch the energy drift)';
      host.append(b);
    }
  }

  private buildKeys(): void {
    const d = this.derivation;
    const keys = $('keys');
    if (!d || this.stageName !== 'simulate') {
      keys.innerHTML = '';
      return;
    }
    const parts = d.coords
      .filter((c) => c.actuated)
      .slice(0, ACTUATOR_LABELS.length)
      .map(
        (c, a) =>
          `<span><kbd>${ACTUATOR_LABELS[a]}</kbd><span data-tex="${encodeURIComponent(c.tex)}"></span></span>`,
      );
    if (toolTargets(d)) parts.push('<span><kbd>drag</kbd>tool target</span>');
    else if (this.mech.bodies.some((b) => b.actuated && b.joint === 'prismatic')) {
      parts.push('<span><kbd>click</kbd>rail target</span>');
    }
    if (d.tool) parts.push('<span><kbd>Shift</kbd>+<kbd>drag</kbd>push</span>');
    parts.push(
      '<span><kbd>1–9</kbd>primary</span><span><kbd>Shift</kbd>+<kbd>1–9</kbd>shadow</span><span><kbd>Space</kbd>kick</span><span><kbd>R</kbd>reset</span>',
    );
    keys.innerHTML = parts.join('');
    for (const el of keys.querySelectorAll<HTMLElement>('[data-tex]'))
      tex(el, decodeURIComponent(el.dataset.tex ?? ''));
  }

  private setRunning(on: boolean): void {
    this.running = on;
    this.client.send({ type: 'run', running: on });
    $('run').textContent = on ? 'Pause' : 'Run';
  }

  private reset(): void {
    this.client.send({ type: 'reset' });
    this.robot?.resetTrail();
    for (const g of this.ghosts) g.resetTrail();
  }

  private chrome(): void {
    const nav = $('stages');
    (['build', 'derive', 'simulate'] as const).forEach((s, k) => {
      const b = button('', () => this.setStage(s), '');
      b.dataset.stage = s;
      b.innerHTML = `<span class="num">${k + 1}</span>${s[0]!.toUpperCase()}${s.slice(1)}`;
      nav.append(b);
    });
    $('run').addEventListener('click', () => this.setRunning(!this.running));
    $('reset').addEventListener('click', () => this.reset());
    $<HTMLSelectElement>('speed').addEventListener('change', (e) => {
      this.client.send({ type: 'speed', speed: Number((e.target as HTMLSelectElement).value) });
    });

    const viewport = $('viewport');
    const pointer = (kind: 'down' | 'move' | 'up') => (e: PointerEvent) => {
      if ((e.target as Element).closest('.overlay, .gizmos *')) return;
      if (kind === 'down') {
        this.closePopover();
        viewport.setPointerCapture(e.pointerId);
      }
      const [x, y] = this.stage.toWorld(e.clientX, e.clientY);
      this.pointer(x, y, kind, e.buttons !== 0 || kind === 'down', e.shiftKey);
    };
    for (const kind of ['down', 'move', 'up'] as const)
      viewport.addEventListener(`pointer${kind}`, pointer(kind));
    window.addEventListener('keydown', (e) => this.key(e, true));
    window.addEventListener('keyup', (e) => this.key(e, false));
    window.addEventListener('blur', () => {
      for (let a = 0; a < ACTUATOR_KEYS.length; a++) this.client.send({ type: 'input', i: a, v: 0 });
      this.held.clear();
    });
  }

  private pointer(x: number, y: number, kind: 'down' | 'move' | 'up', pressed: boolean, shift = false): void {
    const d = this.derivation;
    if (!d || !this.robot) return;
    if (this.stageName === 'build') {
      if (kind !== 'down') return;
      const i = this.robot.pick(x, y, this.snap, this.params);
      if (i >= 0) this.editor.select(i);
      return;
    }
    if (this.stageName !== 'simulate') return;
    // Shift-drag: push the tool with a spring towards the pointer
    if (this.pushAt || (shift && kind === 'down' && d.tool)) {
      if (kind === 'up' || !pressed) {
        this.pushAt = null;
        this.client.flag('PushOn', 0);
      } else {
        this.pushAt = [x, y];
        this.client.flag('PushX', x);
        this.client.flag('PushY', y);
      }
      return;
    }
    if (!pressed || kind === 'up') return;
    if (toolTargets(d)) {
      this.client.flag('TargetX', x);
      this.client.flag('TargetY', y);
      this.setUseTool(true);
      return;
    }
    // project onto the rail of the first actuated prismatic joint
    const i = this.mech.bodies.findIndex((b) => b.actuated && b.joint === 'prismatic');
    if (i < 0 || kind !== 'down') return;
    const b = normalise(this.mech.bodies[i]!);
    const [jx, jy, jp] = this.robot.jointPose(i, this.snap, this.params);
    const axis = jp + b.axisAngle;
    let target = (x - jx) * Math.cos(axis) + (y - jy) * Math.sin(axis);
    if (b.limit > 0) target = Math.max(-b.limit * 0.9, Math.min(b.limit * 0.9, target));
    this.client.send({ type: 'target', i: coordIndex(this.mech)[i]!, v: target });
  }

  private key(e: KeyboardEvent, down: boolean): void {
    if ((e.target as HTMLElement).closest('input, select, textarea')) return;
    const k = e.key.toLowerCase();
    if (this.stageName === 'build') {
      if (down) this.buildKey(e, k);
      return;
    }
    if (e.metaKey || e.ctrlKey || this.stageName !== 'simulate') return;
    if (down && e.repeat) {
      if (this.held.has(k)) e.preventDefault();
      return;
    }
    if (down) this.held.add(k);
    else this.held.delete(k);
    const d = this.derivation;
    const a = ACTUATOR_KEYS.findIndex(([neg, pos]) => k === neg || k === pos);
    if (a >= 0 && d && a < d.nu) {
      // left = negative for prismatic joints, counter-clockwise (positive) for revolute ones
      const kind = d.coords.filter((c) => c.actuated)[a]?.kind;
      const sign = (k === ACTUATOR_KEYS[a]![0] ? -1 : 1) * (kind === 'revolute' ? -1 : 1);
      this.client.send({ type: 'input', i: a, v: down ? sign * this.settings.key : 0 });
      e.preventDefault();
      return;
    }
    if (!down || !d) return;
    if (k === ' ') {
      const passive = d.coords.flatMap((c, i) => (c.actuated ? [] : [i]));
      for (const i of passive.length ? passive : d.coords.map((_, i) => i)) {
        this.client.send({ type: 'kick', i, v: (Math.random() < 0.5 ? -1 : 1) * 1.5 });
      }
      e.preventDefault();
    } else if (k === 'r') this.reset();
    else if (k === 'p') this.setRunning(!this.running);
    else if (/^digit[1-9]$/.test(e.code.toLowerCase())) {
      const spec = this.store.usable()[Number(e.code.slice(5)) - 1];
      if (spec && e.shiftKey) this.store.toggleShadow(spec.id);
      else if (spec) this.store.setPrimary(spec.id);
    }
  }

  private buildKey(e: KeyboardEvent, k: string): void {
    const mod = e.ctrlKey || e.metaKey;
    if (mod && (k === 'z' || k === 'y')) {
      this.undo(k === 'y' || e.shiftKey);
    } else if (k === 'escape') {
      this.gizmos.connectFrom = null;
      this.closePopover();
      $('status').hidden = true;
      return;
    } else if (k === 'delete' || k === 'backspace') {
      this.editor.remove(this.editor.selection);
    } else {
      return;
    }
    e.preventDefault();
  }

  // ---------------------------------------------------------------- frame loop

  private frame(now: number): void {
    const c = this.clock;
    this.client.pull();
    const simulating = this.stageName === 'simulate';
    this.robot?.update(this.snap, this.params, simulating);
    this.robot?.push(simulating ? this.pushAt : null, this.snap);
    if (simulating && this.settings.ghosts) {
      const length = (i: number) => this.robot?.length(i, this.params) ?? 0;
      this.ghosts.forEach((g, k) => {
        g.update(this.snap.slots[k + 1]!, length, this.running);
      });
    } else for (const g of this.ghosts) g.visible = false;
    this.stage.render();
    if (this.stageName === 'build' && this.robot)
      this.gizmos.update(this.mech, this.robot, this.snap, this.params);
    c.frames++;
    if (simulating && now - c.plot >= 1000 / PLOT_HZ) {
      this.plots.sample(this.snap.t);
      this.plots.draw();
      c.plot = now;
    }
    if (now - c.eq >= 1000 / EQ_HZ) {
      this.panel.update();
      if (simulating && this.side === 'control') this.control?.update();
      c.eq = now;
    }
    if (now - c.hud >= 500) {
      const fps = (c.frames * 1000) / (now - c.hud);
      c.frames = 0;
      c.hud = now;
      const s = this.client.stats;
      $('stats').innerHTML =
        `<span><b>${fps.toFixed(0)}</b> fps</span>` +
        `<span><b>${(s.stepsPerSecond / 1000).toFixed(2)}</b> kHz</span>` +
        `<span><b>${s.stepUs.toFixed(2)}</b> µs/step</span>` +
        `<span><b>t</b> ${this.snap.t.toFixed(1)} s</span>` +
        `<span class="muted">${this.stage.backend}${s.shared ? ' · shared memory' : ''}</span>`;
    }
  }
}

/** Computed torque where every DOF is actuated, LQR for underactuated mechanisms, else passive. */
function defaultController(reg: Controllers): number {
  for (const key of ['computed_torque', 'lqr']) {
    const spec = reg.specs.find((s) => s.key === key);
    if (spec && reg.applicable[spec.id] === null) return spec.id;
  }
  return 0;
}

async function main(): Promise<void> {
  const stage = new Stage($('viewport'));
  const [client] = await Promise.all([SimClient.start(), stage.init()]);
  await new App(client, stage).start();
}

main().catch((err: unknown) => {
  console.error(err);
  $('viewport').insertAdjacentHTML(
    'beforeend',
    `<div class="fatal"><h2>Could not start</h2><pre>${String(err)}</pre></div>`,
  );
});
