import 'katex/dist/katex.min.css';
import './styles.css';

import { History } from './app/history';
import { closeLoop, type DragContext, dragJoint, dragTip } from './app/manipulate';
import { defaultSettings, type Settings } from './app/settings';
import { SimulateStage } from './app/simulate';
import { Editor } from './builder/editor';
import { Gizmos } from './builder/gizmos';
import { ControlStore } from './control/store';
import { derivationSections } from './eq/derivation';
import { EquationPanel } from './eq/panel';
import type { Derivation } from './model/derivation';
import {
  cloneMechanism,
  DEFAULT_PRESET,
  encode,
  type JointKind,
  MAX_CLOSURES,
  type Mechanism,
  migrate,
  PRESETS,
  paramVector,
} from './model/description';
import { RobotView } from './scene/robot';
import { Stage } from './scene/stage';
import { SimClient } from './sim/client';
import { closePopover } from './ui/popover';
import { button, portLabel } from './ui/widgets';

type StageName = 'build' | 'derive' | 'simulate';
type ConnectKind = 'pin' | 'slot';

const EQ_HZ = 30;
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
  private readonly sim: SimulateStage;
  private running = false;
  private dragging = false;
  private popover: HTMLElement | null = null;
  private readonly settings: Settings = defaultSettings();
  private readonly history = new History<Mechanism>(cloneMechanism);
  private readonly editor: Editor;
  private readonly gizmos: Gizmos;
  private readonly panel = new EquationPanel($('side'));
  private buildTimer = 0;
  private buildSeq = 0;
  private lastBuildMs = 0;
  private readonly clock = { eq: 0, hud: 0, frames: 0 };

  constructor(
    private readonly client: SimClient,
    private readonly stage: Stage,
  ) {
    this.mech = decodeHash() ?? PRESETS[DEFAULT_PRESET]!();
    this.params = paramVector(this.mech);
    this.sim = new SimulateStage({
      client,
      stage,
      panel: this.panel,
      store: new ControlStore(client, client.layout.slots),
      settings: this.settings,
      robot: () => this.robot,
      mech: () => this.mech,
      params: () => this.params,
      running: () => this.running,
      setRunning: (on) => this.setRunning(on),
      flash: (m) => this.flash(m),
    });
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
    this.stage.clear();
    this.robot = new RobotView(this.stage, this.mech);
    this.sim.load(res.derivation, res.controllers);
    this.stage.castShadows();
    this.robot.showFrames = this.settings.frames;
    this.robot.showTrail = this.settings.trail;
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
    if (s !== 'simulate') {
      this.sim.leave();
      this.reset();
    }
    const sel = s === 'build' ? this.editor.selection : -1;
    this.robot?.select(sel);
    this.gizmos.select(sel);
    this.gizmos.visible = s === 'build';
    this.gizmos.connectFrom = null;
    this.closePopover();
    closePopover();
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
      head.append(title('Simulation', `${d.n - constraints} DOF · 1 kHz RK4`));
      this.sim.enter(head);
    }
    $('hint').textContent =
      this.stageName === 'build'
        ? 'Click a body to select it. Steel hinges are actuated, amber ones passive; blue pins close loops.'
        : this.stageName === 'derive'
          ? 'Every formula on the right was derived symbolically for exactly this structure.'
          : '';
    this.buildToggles();
  }

  private buildToggles(): void {
    const host = $('toggles');
    host.replaceChildren();
    const t = this.settings;
    const items: ['frames' | 'trail' | 'coriolis' | 'ghosts' | 'keys', string, () => void][] = [
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
      ['ghosts', 'Shadows', () => undefined],
      ['coriolis', 'Coriolis', () => this.client.flag('Coriolis', t.coriolis ? 1 : 0)],
      ['keys', 'Keys', () => this.sim.renderKeys()],
    ];
    for (const [key, label, apply] of items) {
      if (key !== 'frames' && key !== 'trail' && this.stageName !== 'simulate') continue;
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

  private setRunning(on: boolean): void {
    this.running = on;
    this.client.send({ type: 'run', running: on });
    $('run').textContent = on ? 'Pause' : 'Run';
  }

  private reset(): void {
    this.client.send({ type: 'reset' });
    this.robot?.resetTrail();
    this.sim.resetTrails();
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
    window.addEventListener('blur', () => this.sim.releaseKeys());
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
    if (this.stageName === 'simulate') this.sim.pointer(x, y, kind, pressed, shift);
  }

  private key(e: KeyboardEvent, down: boolean): void {
    if ((e.target as HTMLElement).closest('input, select, textarea')) return;
    const k = e.key.toLowerCase();
    if (this.stageName === 'build') {
      if (down) this.buildKey(e, k);
      return;
    }
    if (e.metaKey || e.ctrlKey || this.stageName !== 'simulate') return;
    this.sim.key(e, down);
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
    const refresh = now - c.eq >= 1000 / EQ_HZ;
    this.sim.frame(now, refresh);
    this.stage.render();
    if (this.stageName === 'build' && this.robot)
      this.gizmos.update(this.mech, this.robot, this.snap, this.params);
    c.frames++;
    if (refresh) {
      this.panel.update();
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
