import 'katex/dist/katex.min.css';
import './styles.css';

import { DocumentSession } from './app/document';
import { Gauges } from './app/gauges';
import { History } from './app/history';
import { closeLoop, type DragContext, dragJoint, dragTip } from './app/manipulate';
import { defaultSettings, type Settings } from './app/settings';
import { SimulateStage } from './app/simulate';
import { Timeline } from './app/timeline';
import { Editor } from './builder/editor';
import { Gizmos } from './builder/gizmos';
import { type EditContext, Inspector, removeSelection, type Selection } from './builder/inspector';
import { Palette } from './builder/palette';
import { type Anchor, place, placeAtJoint, placeLoop } from './builder/parts';
import { preview } from './builder/preview';
import { ControlStore } from './control/store';
import { derivationSections } from './eq/derivation';
import { EquationPanel } from './eq/panel';
import type { Derivation } from './model/derivation';
import {
  assemblyIssue,
  cloneMechanism,
  DEFAULT_PRESET,
  type JointKind,
  MAX_CLOSURES,
  type Mechanism,
  migrateOrThrow,
  PRESETS,
  paramVector,
  reparent,
} from './model/description';
import { poseBounds } from './scene/kinematics';
import { param, RobotView } from './scene/robot';
import { type Focus, parseFocus, type SchematicFrame } from './scene/schematic';
import { SchematicView } from './scene/schematic-view';
import { Stage } from './scene/stage';
import { setDark } from './scene/theme';
import { SimClient } from './sim/client';
import type { Snapshot } from './sim/layout';
import { closePopover, popover } from './ui/popover';
import { button, portLabel } from './ui/widgets';

type StageName = 'build' | 'derive' | 'simulate';
type ConnectKind = 'pin' | 'slot';

const EQ_HZ = 30;
const TIP_OFFSET_PX = 22;

const $ = <T extends HTMLElement = HTMLElement>(id: string) => document.getElementById(id) as T;

const isDarkDoc = () => document.documentElement.dataset.theme === 'dark';

function savedTheme(): string | null {
  try {
    return localStorage.getItem('theme');
  } catch {
    return null;
  }
}

const encodeHash = (m: Mechanism) => `#m=${btoa(encodeURIComponent(JSON.stringify(m)))}`;

function decodeHash(): { mech: Mechanism | null; error?: string } {
  const match = /^#m=(.+)$/.exec(location.hash);
  if (!match?.[1]) return { mech: null };
  try {
    return { mech: migrateOrThrow(JSON.parse(decodeURIComponent(atob(match[1])))) };
  } catch (error) {
    return { mech: null, error: `Could not import mechanism: ${String(error)}` };
  }
}

class App {
  private stageName: StageName = 'build';
  private readonly document: DocumentSession;
  private get mech(): Mechanism {
    return this.document.draft;
  }
  private get installed(): Mechanism | null {
    return this.document.installed;
  }
  private draftPreview: Snapshot | null = null;
  private get displayed(): Mechanism {
    return this.robot?.mech ?? this.mech;
  }
  private params: number[];
  private derivation: Derivation | null = null;
  private robot: RobotView | null = null;
  private schematic: SchematicView | null = null;
  private readonly sim: SimulateStage;
  private running = false;
  private dragging = false;
  private popover: HTMLElement | null = null;
  private readonly settings: Settings;
  private readonly history = new History<Mechanism>(cloneMechanism);
  private readonly editor: Editor;
  private readonly gizmos: Gizmos;
  private readonly panel = new EquationPanel($('side'));
  private lastBuildMs = 0;
  private lastBuildReused = false;
  private readonly clock = { eq: 0, hud: 0, frames: 0 };

  constructor(
    private readonly client: SimClient,
    private readonly stage: Stage,
  ) {
    this.settings = defaultSettings(client.layout);
    client.onParamRejected(({ i, value }) => {
      this.params[i] = value;
      this.sim.refreshModel();
    });
    client.onError((message) => {
      this.sim.cancelRun();
      this.setRunning(false);
      this.status(message, 'sim');
    });
    const shared = decodeHash();
    this.document = new DocumentSession(shared.mech ?? PRESETS[DEFAULT_PRESET]!(), client);
    if (shared.error) window.setTimeout(() => this.flash(shared.error!), 0);
    this.params = paramVector(this.mech);
    this.sim = new SimulateStage({
      client,
      resetTrails: () => this.schematic?.resetTrail(),
      highlight: (i) => this.schematic?.select(i),
      panel: this.panel,
      store: new ControlStore(client, client.layout.slots),
      settings: this.settings,
      robot: () => this.robot,
      mech: () => this.installed ?? this.mech,
      params: () => this.params,
      running: () => this.running,
      setRunning: (on) => this.setRunning(on),
      flash: (m) => this.flash(m),
      note: (m) => {
        if (m) this.status(m, 'sim');
        else if ($('status').dataset.kind === 'sim') $('status').hidden = true;
      },
    });
    this.editor = new Editor(document.createElement('div'), {
      change: (m, discrete) => this.edited(m, discrete),
      inspect: (kind, index) => this.inspector.show({ kind, index }),
      select: (i) => {
        this.schematic?.select(i);
        this.gizmos.select(i);
        this.closePopover();
      },
      connect: (body, s, kind) => this.startConnect(body, s, kind),
      flash: (msg) => this.flash(msg),
      selectElement: (k) => {
        this.schematic?.selectElement(k);
        if (k >= 0) this.gizmos.select(-1);
      },
      elementLength: (k) => this.elementLength(k),
      startPart: (kind) => this.gizmos.beginPlace(kind),
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
      snapTip: (body, to) => this.snapTip(body, to),
      snapJoint: (body, to) => this.snapJoint(body, to),
      place: (kind, a, b, joint) => {
        const r =
          joint && (kind === 'bar' || kind === 'block')
            ? placeAtJoint(this.mech, kind, joint.kind, joint.from, a)
            : place(this.mech, kind, a, b);
        if (typeof r === 'string') {
          this.flash(r);
          return;
        }
        this.adopt(r.mech, r.select.kind !== 'element' ? r.select.index : this.editor.selection);
        if (r.select.kind === 'element') this.editor.selectElement(r.select.index);
        if (r.select.index >= 0) this.inspector.show({ kind: r.select.kind, index: r.select.index });
        else this.inspector.hide();
      },
      placing: (state, kind) => {
        if (state === 'done') return;
        this.inspector.hide();
        const joint = kind === 'revolute' || kind === 'prismatic' || kind === 'fixed';
        this.flash(
          joint
            ? state === 'first'
              ? 'Choose a Ground point or any point along a body for the joint. Esc cancels.'
              : 'Click an unconnected body, or choose Bar/Block and place it to complete this joint. Esc cancels.'
            : kind === 'ground'
              ? 'Click to place a fixed Ground attachment point. Ground is not a collision floor.'
              : kind === 'bar' || kind === 'block'
                ? state === 'second'
                  ? 'Click to draw the new body from the chosen joint anchor.'
                  : 'Click to place an unconnected body, then add a joint to attach it.'
                : 'Choose the two endpoints on bodies or ground. Esc cancels.',
        );
      },
    });
    this.palette = new Palette({ start: (kind) => this.gizmos.beginPlace(kind) });
    $('viewport').append(this.palette.el);
    this.gauges = new Gauges(this.settings);
    this.timeline = new Timeline(client.layout, {
      advance: (steps) => client.send({ type: 'advance', steps }),
      resume: async (checkpoint) => {
        this.sim.cancelRun();
        const state = await client.restore(checkpoint);
        if (!state) {
          this.flash('This checkpoint is no longer available.');
          return false;
        }
        this.params = [...state.params];
        this.sim.restore(state);
        this.setRunning(true);
        return true;
      },
    });
    client.onCheckpoint((frame) => this.timeline.record(frame));
    this.readout.className = 'readout overlay';
    this.readout.hidden = true;
    $('viewport').append(this.gauges.el, this.timeline.el, this.readout);
    // pointing at an equation lights up what it is about
    const side = $('side');
    side.addEventListener('pointerover', (e) => {
      const el = e.target instanceof Element ? e.target.closest<HTMLElement>('[data-focus]') : null;
      this.focus = parseFocus(el?.dataset.focus);
    });
    side.addEventListener('pointerleave', () => {
      this.focus = null;
    });
    this.inspector = new Inspector($('viewport'), {
      context: () => this.editContext(),
      remove: (sel) => this.removeSelected(sel),
      tool: (i, on) => {
        const m = cloneMechanism(this.mech);
        m.tool = on ? i : m.tool === i ? -1 : m.tool;
        this.adopt(m);
        this.inspector.render();
      },
      close: () => this.closeInspector(),
    });
  }

  private readonly palette: Palette;
  private readonly inspector: Inspector;
  private readonly gauges: Gauges;
  private readonly timeline: Timeline;
  /** Body shown as a free-body diagram (−1: none), and what an equation being pointed at is about. */
  private freeBody = -1;
  private focus: Focus | null = null;
  private readonly readout = document.createElement('div');

  /** The snapshot to draw: live, or the moment being looked back at. */
  private get view() {
    return this.draftPreview ?? this.timeline.view(this.snap);
  }

  /** Take a new document from the view (palette, inspector, snapping): history, panel, rebuild. */
  private adopt(m: Mechanism, selected = this.editor.selection, discrete = true): void {
    this.editor.commit(m, selected, discrete);
  }

  private editContext(): EditContext {
    const app = this;
    return {
      get mech() {
        return app.mech;
      },
      commit: (m, structural = false) => {
        this.adopt(m, this.editor.selection, structural);
        if (structural) this.inspector.render();
      },
      refuse: (msg) => this.flash(msg),
      length: (k) => this.elementLength(k),
    };
  }

  /** Hovering a joint or a centre of mass: its numbers, next to the pointer. */
  private hover(x: number, y: number): void {
    const d = this.derivation;
    const robot = this.robot;
    const el = this.readout;
    if (!d || !robot) return;
    const s = this.view;
    const reach = 9 * this.stage.metresPerPixel;
    let html = '';
    let best = reach;
    (this.installed ?? this.mech).bodies.forEach((b, k) => {
      const [fx, fy] = s.frame(k + 1);
      const j = d.coords.findIndex((c) => c.body === k);
      if (j >= 0 && Math.hypot(x - fx, y - fy) < best) {
        best = Math.hypot(x - fx, y - fy);
        const ang = b.joint === 'revolute';
        const [q, qd, tau] = [s.v('Q', j), s.v('QD', j), s.v('TAU', j)];
        html = `<b>${ang ? 'θ' : 'd'}<sub>${k + 1}</sub></b> = ${q.toFixed(3)} ${ang ? 'rad' : 'm'}<br>rate ${qd.toFixed(3)} ${ang ? 'rad/s' : 'm/s'}<br>${ang ? 'torque' : 'force'} τ = ${tau.toFixed(2)} ${ang ? 'N·m' : 'N'}`;
      }
      const [cx, cy] = robot.com(k, s, this.params);
      const m = param(this.params, k, 'm') ?? 0;
      if (m > 0 && Math.hypot(x - cx, y - cy) < best) {
        best = Math.hypot(x - cx, y - cy);
        const [vx, vy] = [s.v('VCOM', 2 * k), s.v('VCOM', 2 * k + 1)];
        const w = s.v('OMEGA', k);
        const inertia = param(this.params, k, 'I') ?? 0;
        const t = 0.5 * m * (vx * vx + vy * vy) + 0.5 * inertia * w * w;
        const total = s.v('ENERGY');
        html = `<b>body ${k + 1}</b>: m = ${m.toFixed(2)} kg<br>|v<sub>c</sub>| = ${Math.hypot(vx, vy).toFixed(3)} m/s, ω = ${w.toFixed(3)} rad/s<br>T<sub>${k + 1}</sub> = ½mv² + ½Iω² = ${t.toFixed(3)} J${total > 1e-9 ? ` (${((100 * t) / total).toFixed(0)} % of T)` : ''}`;
      }
    });
    el.hidden = !html;
    if (!html) return;
    el.innerHTML = html;
    const [sx, sy] = this.stage.toScreen(x, y);
    el.style.left = `${sx + 16}px`;
    el.style.top = `${sy + 12}px`;
  }

  /** Distance between the ends of spring/damper k at the current pose. */
  private elementLength(k: number): number {
    if (!this.robot || !this.displayed.elements[k]) return 0;
    const [[ax, ay], [bx, by]] = this.robot.elementEnds(k, this.snap, this.params);
    return Math.hypot(bx - ax, by - ay);
  }

  private closeInspector(): void {
    this.inspector.hide();
    this.editor.selectElement(-1);
  }

  private removeSelected(sel: Selection): void {
    const r = removeSelection(this.mech, sel);
    if (typeof r === 'string') {
      this.flash(r);
      return;
    }
    this.inspector.hide();
    if (sel.kind === 'element') this.editor.selectElement(-1);
    this.adopt(r, sel.kind === 'body' ? Math.max(0, sel.index - 1) : this.editor.selection);
  }

  /** The tip of `body` was dropped on a port: put it exactly there and pin it. */
  private snapTip(body: number, to: Anchor): void {
    const moved = dragTip(this.dragContext(), body, [to.x, to.y], true, false) ?? this.mech;
    const r = placeLoop(moved, 'pin', { body, s: 1, x: to.x, y: to.y }, to);
    if (typeof r === 'string') {
      this.flash(r);
      return;
    }
    this.adopt(r.mech, body);
    this.flash(
      `Pinned the tip of body ${body + 1} to ${to.body < 0 ? 'the ground' : `body ${to.body + 1}`}: a closed loop.`,
    );
  }

  /** The joint of `body` was dropped on a port of another body: hang it there. */
  private snapJoint(body: number, to: Anchor): void {
    if (to.body < 0 || this.mech.bodies[body]?.detached) return;
    const r = reparent(this.mech, body, to.body);
    if (!r) {
      this.flash('a body cannot hang from its own descendant — close a loop instead');
      return;
    }
    const m = cloneMechanism(r.mech);
    m.bodies[r.index] = { ...m.bodies[r.index]!, mountS: to.s };
    this.adopt(m, r.index);
    this.flash(`Body ${body + 1} now hangs from body ${to.body + 1}.`);
  }

  private get snap() {
    return this.draftPreview ?? this.client.snap;
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
  private edited(m: Mechanism, discrete = false): void {
    if (!this.dragging) this.history.record(this.mech, discrete);
    this.gizmos.cancel();
    this.document.edit(m);
    this.sim.cancelRun();
    this.document.schedule(() => void this.rebuild(), this.dragging ? 10 : 60);
  }

  private undo(redo = false): void {
    const m = this.history.step(this.mech, redo);
    if (!m) return;
    this.gizmos.cancel();
    this.document.edit(m);
    this.sim.cancelRun();
    this.editor.load(m);
    this.inspector.render();
    void this.rebuild();
  }

  /** Show the authored pose of an empty, incomplete or invalid assembly without stale dynamics. */
  private showDraft(): void {
    if ($('status').dataset.kind === 'build') $('status').hidden = true;
    this.derivation = null;
    this.draftPreview = preview(this.mech, this.client.layout);
    this.timeline.clear();
    this.params = paramVector(this.mech);
    history.replaceState(null, '', encodeHash(this.mech));
    this.robot = new RobotView(this.mech);
    this.schematic?.host.remove();
    const bounds =
      this.mech.bodies.length || this.mech.anchors?.length
        ? poseBounds(this.mech)
        : { cx: 0, cy: 0, width: 4, height: 3 };
    this.schematic = new SchematicView(this.mech, { coords: [] } as unknown as Derivation, bounds);
    $('viewport').prepend(this.schematic.host);
    this.freeBody = -1;
    if (this.stageName !== 'build') this.setStage('build');
    if (!this.dragging && !this.userView)
      this.stage.setView(bounds.cx, bounds.cy, bounds.width, bounds.height);
    this.schematic.select(this.editor.selection);
    this.gizmos.select(this.editor.selection);
    this.refreshStage();
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
    const outcome = await this.document.rebuild();
    if (outcome.kind === 'empty') {
      this.showDraft();
      return;
    }
    if (outcome.kind === 'error') {
      this.showDraft();
      this.status(outcome.error, 'build');
      return;
    }
    if (outcome.kind !== 'installed') return;
    const { document: mech, result: res } = outcome;
    if ($('status').dataset.kind === 'build') $('status').hidden = true;
    this.draftPreview = null;
    this.derivation = res.derivation;
    this.lastBuildMs = res.ms;
    this.lastBuildReused = res.reused;
    this.params = paramVector(mech);
    history.replaceState(null, '', encodeHash(mech));
    this.robot = new RobotView(mech);
    this.schematic?.host.remove();
    this.schematic = new SchematicView(mech, res.derivation, this.robot.bounds());
    $('viewport').prepend(this.schematic.host);
    this.sim.load(res.derivation, res.controllers);
    if (assemblyIssue(this.mech)) {
      this.showDraft();
      return;
    }
    this.gauges.load(res.derivation);
    this.timeline.clear();
    this.freeBody = -1;
    // a different mechanism is framed afresh; edits keep the view unless the user has not touched it
    if (mech.name !== this.framedName || mech.name === 'Custom') {
      if (mech.name !== this.framedName) this.userView = false;
      this.framedName = mech.name;
    }
    if (!this.dragging && !this.userView) this.fitView();
    const sel = this.stageName === 'build' ? this.editor.selection : -1;
    this.schematic.select(sel);
    this.gizmos.select(sel);
    // the loop closures are solved numerically from q(0): report when they cannot close
    const generation = this.client.generation;
    window.setTimeout(() => {
      if (this.client.generation !== generation) return;
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

  private status(message: string, kind: 'build' | 'flash' | 'sim'): void {
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
      act(
        `+ ${kind === 'revolute' ? 'rotational' : kind === 'prismatic' ? 'translational' : 'fixed'} joint here…`,
        () => this.gizmos.beginJoint(kind, body, s),
      );
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
    if (s !== 'build' && !this.derivation) return; // nothing built yet
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
    this.schematic?.select(sel);
    this.gizmos.select(sel);
    this.gizmos.visible = s === 'build';
    this.gizmos.connectFrom = null;
    if (s !== 'build') this.inspector.hide();
    this.gauges.apply(s === 'simulate');
    this.readout.hidden = true;
    if (!this.userView) this.fitView();
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
    for (const b of $('stages').querySelectorAll<HTMLButtonElement>('button')) {
      b.disabled = !d && b.dataset.stage !== 'build';
    }
    if (!d) {
      // an empty scene: the builder alone
      const title = document.createElement('div');
      title.className = 'side-title';
      title.innerHTML = '<b>Build</b>';
      const sub = document.createElement('span');
      sub.textContent =
        assemblyIssue(this.mech) ?? 'Place bodies and ground anchors, then connect them with joints.';
      title.append(sub);
      const undo = button('↶', () => this.undo(), 'btn small', 'undo (Ctrl+Z)');
      const redo = button('↷', () => this.undo(true), 'btn small', 'redo (Ctrl+Shift+Z)');
      undo.disabled = !this.history.canUndo;
      redo.disabled = !this.history.canRedo;
      head.append(title, undo, redo);
      $('side').replaceChildren(this.editor.host);
      $('hint').textContent =
        'Place a Ground point and a Bar or Block. Choose a joint, click its ground or body anchor, then the unconnected body. Click graph nodes to edit bodies and joints separately.';
      this.buildToggles();
      return;
    }
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
        title(
          'Build',
          `${(this.installed ?? this.mech).bodies.length} bodies · ${dof} · ${d.nu} actuated · ${this.lastBuildReused ? 'updated' : 'derived'} in ${ms}`,
        ),
        tools,
      );
      $('side').replaceChildren(this.editor.host);
    } else if (this.stageName === 'derive') {
      head.append(
        title('Derivation', `Lagrange's method for this topology · ${ms}`),
        next('Simulate →', 'simulate'),
      );
      this.panel.mount(derivationSections(this.installed ?? this.mech, d), undefined, true);
    } else {
      head.append(title('Simulation', `${d.n - constraints} DOF · 1 kHz RK4`));
      this.sim.enter(head);
    }
    $('hint').textContent =
      this.stageName === 'build'
        ? 'Place bodies and Ground points, then join them with rotational, translational or fixed joints. Click a body or joint in the graph to edit it. Motor/passive is a joint setting. Drag ○ to rotate or stretch a body; ◆ moves its attachment.'
        : this.stageName === 'derive'
          ? 'Every formula on the right was derived symbolically for exactly this structure.'
          : '';
    this.buildToggles();
  }

  /** The view toolbar: a View menu (what the figure shows), zoom and fit, and an Export menu. */
  private buildToggles(): void {
    const host = $('toggles');
    host.replaceChildren();
    const t = this.settings;
    const sim = this.stageName === 'simulate';
    const redraw = () => {
      if (this.schematic) this.schematic.dirty = true;
    };
    type Key =
      | 'frames'
      | 'labels'
      | 'dims'
      | 'velocities'
      | 'forces'
      | 'freeBody'
      | 'torqueBars'
      | 'energyGauge'
      | 'trail'
      | 'coriolis'
      | 'ghosts'
      | 'keys';
    const groups: [string, [Key, string, string, () => void][]][] = [
      [
        'Figure',
        [
          ['labels', 'Labels', 'joint coordinates, masses and gravity', redraw],
          ['frames', 'Frames', 'coordinate frames {s}, {1} … {n} and {t}', redraw],
          ['dims', 'Dimensions', 'the length lᵢ of every bar', redraw],
          ...(sim
            ? ([
                ['velocities', 'Velocities', 'of the centres of mass', redraw],
                ['forces', 'Forces', 'weights, springs and dampers, loops, the push', redraw],
              ] as [Key, string, string, () => void][])
            : []),
          ...(sim
            ? ([
                [
                  'trail',
                  'Trail',
                  "the tool's path",
                  () => {
                    this.schematic?.resetTrail();
                    redraw();
                  },
                ],
                ['ghosts', 'Shadows', 'the shadow controllers, in their colours', redraw],
              ] as [Key, string, string, () => void][])
            : []),
        ],
      ],
      ...(sim
        ? ([
            [
              'Simulation',
              [
                [
                  'coriolis',
                  'Coriolis forces',
                  'off is non-physical: watch the energy drift',
                  () => this.client.flag('Coriolis', t.coriolis ? 1 : 0),
                ],
                [
                  'freeBody',
                  'Free-body diagram',
                  'click a body: every force on it, and m a',
                  () => {
                    if (!t.freeBody) this.freeBody = -1;
                    redraw();
                  },
                ],
                ['torqueBars', 'Torque bars', 'M q̈ + C q̇ + N = τ, joint by joint', () => this.gauges.apply()],
                [
                  'energyGauge',
                  'Energy gauge',
                  'kinetic, potential, lost and supplied',
                  () => this.gauges.apply(),
                ],
                ['keys', 'Keyboard legend', 'the keys that drive the joints', () => this.sim.renderKeys()],
              ],
            ],
          ] as [string, [Key, string, string, () => void][]][])
        : []),
    ];
    const viewMenu = () => {
      const menu = document.createElement('div');
      menu.className = 'check-menu';
      for (const [title, items] of groups) {
        const h = document.createElement('h5');
        h.textContent = title;
        menu.append(h);
        for (const [key, label, hint, apply] of items) {
          const b = button(
            '',
            () => {
              t[key] = !t[key];
              b.classList.toggle('on', t[key]);
              apply();
            },
            `menu-check${t[key] ? ' on' : ''}`,
          );
          b.innerHTML = `<span class="tick"></span><span>${label}<small>${hint}</small></span>`;
          menu.append(b);
        }
      }
      return menu;
    };
    const view = button(
      'View ▾',
      () => popover(view, viewMenu(), 'right'),
      'chip action',
      'what the figure shows',
    );
    const zoom = (f: number) => () => {
      const m = this.stage.mapping();
      this.stage.zoomAt(f, m.w / 2, m.h / 2);
      this.userView = true;
    };
    const exportMenu = () => {
      const menu = document.createElement('div');
      menu.className = 'check-menu';
      menu.append(
        button('SVG file — the picture as it is now', () => this.exportSchematic('svg'), 'menu-item'),
        button('TikZ code — copied, ready for LaTeX', () => this.exportSchematic('tikz'), 'menu-item'),
      );
      return menu;
    };
    const exp = button(
      'Export ▾',
      () => popover(exp, exportMenu(), 'right'),
      'chip action',
      'save the figure',
    );
    host.append(
      view,
      button('−', zoom(1 / 1.25), 'chip action zoom', 'zoom out (wheel)'),
      button(
        'Fit',
        () => this.fitView(),
        'chip action zoom',
        'frame the mechanism (right- or middle-drag pans)',
      ),
      button('+', zoom(1.25), 'chip action zoom', 'zoom in (wheel)'),
      exp,
    );
  }

  /** The user zoomed or panned: keep their view until they ask to fit again. */
  private userView = false;
  private framedName = '';

  /** Frame the mechanism: its pose while building and deriving, everywhere it can reach while simulating. */
  private fitView(): void {
    if (!this.robot) return;
    if (!this.mech.bodies.length && !this.mech.anchors?.length) {
      this.stage.setView(0, 0, 4, 3);
      this.userView = false;
      return;
    }
    const b = this.stageName === 'simulate' ? this.robot.bounds() : poseBounds(this.displayed);
    this.stage.setView(b.cx, b.cy, b.width, b.height);
    this.userView = false;
  }

  private ghostSlots: SchematicFrame['ghosts'] | null = null;

  private schematicFrame(): SchematicFrame {
    const simulating = this.stageName === 'simulate';
    this.ghostSlots ??= this.client.snap.slots.slice(1);
    return {
      snap: this.view,
      params: this.timeline.viewParams(this.params),
      running: simulating,
      push: this.timeline.viewPush(this.sim.pushPoint),
      ghosts:
        simulating && this.settings.ghosts
          ? this.timeline.replaying
            ? this.view.slots.slice(1)
            : this.ghostSlots
          : [],
      annotate: this.settings.labels,
      frames: this.settings.frames,
      dims: this.settings.dims,
      velocities: simulating && this.settings.velocities,
      forces: simulating && this.settings.forces,
      freeBody: simulating && this.settings.freeBody ? this.freeBody : -1,
      focus: this.focus,
      trail: this.settings.trail,
    };
  }

  private exportSchematic(kind: 'svg' | 'tikz'): void {
    const view = this.schematic;
    if (!view) return;
    const f = this.schematicFrame();
    const m = this.stage.mapping();
    const name = this.mech.name.toLowerCase().replace(/[^a-z0-9]+/g, '-');
    if (kind === 'svg') {
      const url = URL.createObjectURL(new Blob([view.fig.svg(m, f)], { type: 'image/svg+xml' }));
      const a = document.createElement('a');
      a.href = url;
      a.download = `${name}.svg`;
      a.click();
      window.setTimeout(() => URL.revokeObjectURL(url), 1000);
      return;
    }
    navigator.clipboard.writeText(view.fig.tikz(m, f)).then(
      () => this.flash('TikZ code copied to the clipboard.'),
      () => this.flash('Could not access the clipboard.'),
    );
  }

  /** Ink on paper, or chalk on a board; `save` remembers an explicit choice. */
  private applyTheme(dark: boolean, save = false): void {
    document.documentElement.dataset.theme = dark ? 'dark' : 'light';
    setDark(dark);
    $('theme').textContent = dark ? 'Light' : 'Dark';
    if (save) {
      try {
        localStorage.setItem('theme', dark ? 'dark' : 'light');
      } catch {
        // storage may be unavailable (private mode): the choice lasts for this page
      }
    }
    this.schematic?.restyle();
    this.sim.restyle();
    if (this.stageName === 'build') this.editor.load(this.mech);
  }

  private setRunning(on: boolean): void {
    this.running = on;
    this.client.send({ type: 'run', running: on });
    $('run').textContent = on ? 'Pause' : 'Run';
  }

  private reset(): void {
    this.client.send({ type: 'reset' });
    this.schematic?.resetTrail();
    this.timeline.clear();
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
    this.applyTheme(document.documentElement.dataset.theme === 'dark');
    $('theme').addEventListener('click', () => this.applyTheme(!isDarkDoc(), true));
    matchMedia('(prefers-color-scheme: dark)').addEventListener('change', (e) => {
      if (!savedTheme()) this.applyTheme(e.matches);
    });
    $('reset').addEventListener('click', () => this.reset());
    $<HTMLSelectElement>('speed').addEventListener('change', (e) => {
      this.client.send({ type: 'speed', speed: Number((e.target as HTMLSelectElement).value) });
    });

    const viewport = $('viewport');
    // the wheel zooms about the pointer; the right or middle button pans
    viewport.addEventListener(
      'wheel',
      (e) => {
        if (e.target instanceof Element && e.target.closest('.overlay')) return;
        e.preventDefault();
        const r = viewport.getBoundingClientRect();
        this.stage.zoomAt(Math.exp(-e.deltaY * 0.0015), e.clientX - r.left, e.clientY - r.top);
        this.userView = true;
      },
      { passive: false },
    );
    viewport.addEventListener('contextmenu', (e) => e.preventDefault());
    let panFrom: [number, number] | null = null;
    viewport.addEventListener(
      'pointerdown',
      (e) => {
        if (e.button !== 1 && e.button !== 2) return;
        e.stopPropagation();
        e.preventDefault();
        panFrom = [e.clientX, e.clientY];
        viewport.setPointerCapture(e.pointerId);
        viewport.classList.add('panning');
      },
      true,
    );
    viewport.addEventListener('pointermove', (e) => {
      if (!panFrom) return;
      this.stage.panBy(e.clientX - panFrom[0], e.clientY - panFrom[1]);
      panFrom = [e.clientX, e.clientY];
      this.userView = true;
    });
    const endPan = () => {
      panFrom = null;
      viewport.classList.remove('panning');
    };
    viewport.addEventListener('pointerup', endPan);
    viewport.addEventListener('pointercancel', endPan);
    const pointer = (kind: 'down' | 'move' | 'up') => (e: PointerEvent) => {
      if (panFrom) return;
      if (e.target instanceof Element && e.target.closest('.overlay, .gizmos *')) return;
      if (kind === 'down') {
        this.closePopover();
        viewport.setPointerCapture(e.pointerId);
      }
      const [x, y] = this.stage.toWorld(e.clientX, e.clientY);
      this.pointer(x, y, kind, e.buttons !== 0 || kind === 'down', e.shiftKey, e.altKey);
    };
    for (const kind of ['down', 'move', 'up'] as const)
      viewport.addEventListener(`pointer${kind}`, pointer(kind));
    window.addEventListener('keydown', (e) => this.key(e, true));
    window.addEventListener('keyup', (e) => this.key(e, false));
    window.addEventListener('blur', () => this.sim.releaseKeys());
  }

  private pointer(
    x: number,
    y: number,
    kind: 'down' | 'move' | 'up',
    pressed: boolean,
    shift = false,
    alt = false,
  ): void {
    const d = this.derivation;
    if (!this.robot) return;
    if (this.stageName === 'build') {
      if (kind !== 'down') return;
      // springs and dampers first: they are thin and often lie over bodies
      const k = this.robot.pickElement(x, y, this.snap, this.params, 7 * this.stage.metresPerPixel);
      if (k >= 0) {
        this.editor.selectElement(k);
        this.inspector.show({ kind: 'element', index: k });
        return;
      }
      const i = this.robot.pick(x, y, this.snap, this.params);
      if (i >= 0) {
        this.editor.selectElement(-1);
        this.editor.select(i);
        this.inspector.show({ kind: 'body', index: i });
      } else this.closeInspector();
      return;
    }
    if (this.stageName !== 'simulate' || !d) return;
    if (!pressed && kind === 'move') {
      this.hover(x, y);
      return;
    }
    this.readout.hidden = true;
    // free-body diagram: a click on a body picks it (anywhere else still moves the target)
    if (this.settings.freeBody && kind === 'down') {
      const i = this.robot.pick(x, y, this.view, this.params);
      if (i >= 0) {
        this.freeBody = i;
        return;
      }
    }
    if (this.timeline.replaying) return;
    this.sim.pointer(x, y, kind, pressed, shift, alt);
  }

  private key(e: KeyboardEvent, down: boolean): void {
    // key events may target the window or document (not an element): only skip typing into form fields
    if (e.target instanceof Element && e.target.closest('input, select, textarea')) return;
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
      this.gizmos.cancel();
      this.closeInspector();
      this.closePopover();
      $('status').hidden = true;
      return;
    } else if (k === 'delete' || k === 'backspace') {
      const sel = this.inspector.selection;
      if (sel) this.removeSelected(sel);
      else if (this.editor.elementSelection >= 0)
        this.removeSelected({ kind: 'element', index: this.editor.elementSelection });
      else this.editor.remove(this.editor.selection);
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
    this.timeline.setVisible(simulating && !this.running);
    this.timeline.tick();
    this.sim.setCursor(this.timeline.replaying ? this.view.t : null);
    const refresh = now - c.eq >= 1000 / EQ_HZ;
    this.sim.frame(now, refresh);
    this.schematic?.draw(this.stage.mapping(), this.schematicFrame());
    if (simulating) this.gauges.draw(this.view, now);
    if (this.stageName === 'build' && this.robot)
      this.gizmos.update(this.displayed, this.robot, this.snap, this.params);
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
      const khz = s.stepsPerSecond / 1000;
      // the time for everyone; the machinery (frame rate, step rate and cost) on hover
      const realtimeKhz = s.realtime / (this.snap.layout.dt * 1000);
      const slow = this.running && this.stageName === 'simulate' && khz < 0.95 * realtimeKhz;
      const el = $('stats');
      el.innerHTML =
        `<span><i>t</i> = <b>${this.snap.t.toFixed(1)}</b> s</span>` +
        (slow ? '<span class="slow">slower than real time</span>' : '');
      el.title = `${fps.toFixed(0)} frames/s · physics ${khz.toFixed(2)} kHz (${realtimeKhz.toFixed(2)} kHz requested) · ${s.stepUs.toFixed(2)} µs per step${s.shared ? ' · shared memory' : ''}`;
    }
  }
}

async function main(): Promise<void> {
  const stage = new Stage($('viewport'));
  const client = await SimClient.start();
  await new App(client, stage).start();
}

main().catch((err: unknown) => {
  console.error(err);
  $('viewport').insertAdjacentHTML(
    'beforeend',
    `<div class="fatal"><h2>Could not start</h2><pre>${String(err)}</pre></div>`,
  );
});
