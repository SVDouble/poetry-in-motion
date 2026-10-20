import 'katex/dist/katex.min.css';
import './styles.css';

import { Editor, portLabel, segmented, slider } from './builder/editor';
import { Gizmos } from './builder/gizmos';
import { derivationSections } from './eq/derivation';
import { liveSections, MODE } from './eq/live';
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
import { type PlotSpec, Plots } from './plots/plots';
import { RobotView } from './scene/robot';
import { Stage } from './scene/stage';
import { css, palette } from './scene/theme';
import { SimClient } from './sim/client';
import type { Flag } from './sim/layout';

type StageName = 'build' | 'derive' | 'simulate';

const EQ_HZ = 30;
const PLOT_HZ = 30;
const DEG = 180 / Math.PI;
const ACTUATOR_KEYS: [string, string][] = [
  ['arrowleft', 'arrowright'],
  ['a', 'd'],
  ['j', 'l'],
  ['u', 'o'],
  ['z', 'c'],
  ['v', 'n'],
];
const ACTUATOR_LABELS = ['← →', 'A D', 'J L', 'U O', 'Z C', 'V N'];

const $ = <T extends HTMLElement = HTMLElement>(id: string) => document.getElementById(id) as T;

interface Tuning {
  umax: number;
  push: number;
  pdWn: number;
  ctWn: number;
  refWn: number;
  lqrQx: number;
  lqrQv: number;
  lqrR: number;
  swing: number;
  coriolis: boolean;
  frames: boolean;
  trail: boolean;
}

const MODES: {
  id: number;
  label: string;
  hint: string;
  when: (n: number, nu: number, closed: boolean) => boolean;
}[] = [
  {
    id: MODE.PASSIVE,
    label: 'Passive',
    hint: 'No controller: gravity, friction and your inputs only.',
    when: () => true,
  },
  { id: MODE.PD, label: 'PD', hint: 'Independent joint PD. Gravity makes it sag.', when: (_n, nu) => nu > 0 },
  {
    id: MODE.PD_GRAVITY,
    label: 'PD + gravity',
    hint: 'PD plus ∂V/∂q: no steady-state error, transients still depend on M(q).',
    when: (_n, nu) => nu > 0,
  },
  {
    id: MODE.COMPUTED_TORQUE,
    label: 'Computed torque',
    hint: 'Feedback linearisation: identical linear error dynamics everywhere.',
    when: (n, nu, closed) => nu === n && !closed,
  },
  {
    id: MODE.LQR,
    label: 'LQR',
    hint: 'Linear-quadratic regulator about the target q*.',
    when: (_n, nu, closed) => nu > 0 && !closed,
  },
  {
    id: MODE.SWING_UP,
    label: 'Swing-up',
    hint: 'Energy pumping through the actuated joints, then an LQR catch at q*.',
    when: (n, nu, closed) => nu > 0 && nu < n && !closed,
  },
];

function defaultMode(d: Derivation): number {
  if (d.nu === 0) return MODE.PASSIVE;
  if (d.closures.length) return MODE.PD_GRAVITY;
  if (d.nu === d.n) return MODE.COMPUTED_TORQUE;
  return MODE.LQR;
}

const availableModes = (d: Derivation) => MODES.filter((m) => m.when(d.n, d.nu, d.closures.length > 0));
const SNAP_ANGLE = Math.PI / 12;
const SNAP_LEN = 0.05;
const snap = (v: number, step: number) => Math.round(v / step) * step;
const wrapAngle = (a: number) => Math.atan2(Math.sin(a), Math.cos(a));
const TIP_OFFSET_PX = 22;

function encodeHash(m: Mechanism): string {
  return `#m=${btoa(encodeURIComponent(JSON.stringify(m)))}`;
}

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
  private mode = MODE.PASSIVE as number;
  private running = false;
  private ik = false;
  private readonly editor: Editor;
  private readonly gizmos: Gizmos;
  private readonly undoStack: Mechanism[] = [];
  private readonly redoStack: Mechanism[] = [];
  private lastHistory = 0;
  private dragging = false;
  private popover: HTMLElement | null = null;
  private readonly panel = new EquationPanel($('side'));
  private readonly plots = new Plots($('plots'));
  private readonly held = new Set<string>();
  private buildTimer = 0;
  private buildSeq = 0;
  private lastBuildMs = 0;
  private lastEq = 0;
  private lastPlot = 0;
  private lastHud = 0;
  private frames = 0;
  private readonly tuning: Tuning = {
    umax: 80,
    push: 20,
    pdWn: 6,
    ctWn: 12,
    refWn: 4,
    lqrQx: 10,
    lqrQv: 1,
    lqrR: 1,
    swing: 12,
    coriolis: true,
    frames: true,
    trail: true,
  };

  constructor(
    private readonly client: SimClient,
    private readonly stage: Stage,
  ) {
    this.mech = decodeHash() ?? PRESETS[DEFAULT_PRESET]!();
    this.params = paramVector(this.mech);
    this.editor = new Editor(document.createElement('div'), {
      change: (m) => this.edited(m),
      select: (i) => {
        this.robot?.select(i);
        this.gizmos.select(i);
        this.closePopover();
      },
      connect: (body, s) => this.startConnect(body, s),
      flash: (msg) => this.flash(msg),
    });
    this.gizmos = new Gizmos($('viewport'), stage, {
      port: (body, s, at) => this.openPortMenu(body, s, at),
      connect: (body, s, world) => this.finishConnect(body, s, world),
      dragStart: () => {
        this.dragging = true;
        this.record(true);
      },
      dragJoint: (w, free) => this.dragJoint(w, free),
      dragTip: (w, free) => this.dragTip(w, free),
      dragEnd: () => {
        this.dragging = false;
        this.editor.load(this.mech);
      },
    });
  }

  // ---------------------------------------------------------------- history & direct manipulation

  /** Remember the current document before a change (slider bursts within 600 ms coalesce). */
  private record(force = false): void {
    const now = performance.now();
    if (!force && now - this.lastHistory < 600) return;
    this.lastHistory = now;
    this.undoStack.push(cloneMechanism(this.mech));
    if (this.undoStack.length > 200) this.undoStack.shift();
    this.redoStack.length = 0;
  }

  private undo(redo = false): void {
    const from = redo ? this.redoStack : this.undoStack;
    const to = redo ? this.undoStack : this.redoStack;
    const m = from.pop();
    if (!m) return;
    to.push(cloneMechanism(this.mech));
    this.mech = m;
    this.editor.load(m);
    this.lastHistory = 0;
    void this.rebuild();
  }

  private flash(message: string): void {
    const status = $('status');
    status.dataset.kind = 'flash';
    status.textContent = message;
    status.hidden = false;
    window.setTimeout(() => {
      if (status.textContent === message) status.hidden = true;
    }, 2600);
  }

  private closePopover(): void {
    this.popover?.remove();
    this.popover = null;
  }

  private openPortMenu(body: number, s: number, [x, y]: [number, number]): void {
    this.closePopover();
    const pop = document.createElement('div');
    pop.className = 'popover overlay';
    pop.style.left = `${x + 14}px`;
    pop.style.top = `${y - 10}px`;
    const title = document.createElement('div');
    title.className = 'popover-title';
    title.textContent = `Body ${body + 1} · ${portLabel(s)}`;
    pop.append(title);
    const action = (label: string, run: () => void, disabled = false) => {
      const b = document.createElement('button');
      b.type = 'button';
      b.className = 'btn small';
      b.textContent = label;
      b.disabled = disabled;
      b.addEventListener('click', () => {
        this.closePopover();
        run();
      });
      pop.append(b);
    };
    for (const kind of ['revolute', 'prismatic', 'fixed'] as JointKind[]) {
      action(`+ ${kind} child here`, () => this.editor.addChild(body, kind, s));
    }
    action(
      'pin a loop from here…',
      () => this.startConnect(body, s),
      this.mech.closures.length >= MAX_CLOSURES,
    );
    if (s === 1) {
      action(this.mech.tool === body ? 'tool is here' : 'make this the tool', () => {
        const m = cloneMechanism(this.mech);
        m.tool = body;
        this.editor.commit(m);
      });
    }
    $('viewport').append(pop);
    this.popover = pop;
  }

  private startConnect(body: number, s: number): void {
    this.closePopover();
    this.gizmos.connectFrom = { body, s };
    this.flash('Pin a loop: click a port of another body, or empty space to pin to the ground. Esc cancels.');
  }

  private finishConnect(body: number, s: number, [x, y]: [number, number]): void {
    const from = this.gizmos.connectFrom;
    this.gizmos.connectFrom = null;
    if (!from) return;
    const m = cloneMechanism(this.mech);
    if (body < 0) {
      m.closures.push({
        a: -1,
        sa: 0,
        b: from.body,
        sb: from.s,
        gx: snap(x, SNAP_LEN),
        gy: snap(y, SNAP_LEN),
      });
    } else {
      m.closures.push({ a: from.body, sa: from.s, b: body, sb: s, gx: 0, gy: 0 });
    }
    m.name = 'Custom';
    this.editor.commit(m);
  }

  private dragJoint([x, y]: [number, number], free: boolean): void {
    const i = this.editor.selection;
    const b = this.mech.bodies[i];
    if (!b || !this.robot) return;
    const m = cloneMechanism(this.mech);
    const t = m.bodies[i]!;
    if (b.parent < 0) {
      t.mountX = free ? x : snap(x, SNAP_LEN);
      t.mountY = free ? y : snap(y, SNAP_LEN);
    } else {
      const [px, py, pp] = this.snap.frame(b.parent + 1);
      const l = this.robot.length(b.parent, this.params);
      if (l < 1e-6) return;
      const along = ((x - px) * Math.cos(pp) + (y - py) * Math.sin(pp)) / l;
      t.mountS = Math.max(0, Math.min(1, free ? along : snap(along, 0.05)));
    }
    m.name = 'Custom';
    this.edited(m, false);
  }

  private dragTip([x, y]: [number, number], free: boolean): void {
    const i = this.editor.selection;
    const b = this.mech.bodies[i];
    if (!b || !this.robot) return;
    const [ox, oy] = this.snap.frame(i + 1);
    const [, , parentPhi] = this.robot.parentPose(i, this.snap);
    const desired = Math.atan2(y - oy, x - ox);
    const own = b.joint === 'revolute' ? b.q0 : 0;
    let mount = wrapAngle(desired - parentPhi - own);
    if (!free) mount = snap(mount, SNAP_ANGLE);
    const m = cloneMechanism(this.mech);
    const t = m.bodies[i]!;
    t.mountAngle = mount;
    if (b.shape !== 'block') {
      const len = Math.hypot(x - ox, y - oy) - TIP_OFFSET_PX * this.stage.metresPerPixel;
      t.l = Math.max(0.1, Math.min(3, free ? len : snap(len, SNAP_LEN)));
      m.bodies[i] = normalise(t);
    }
    m.name = 'Custom';
    this.edited(m, false);
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

  // ---------------------------------------------------------------- building

  /** A new document from the editor or a drag. `record` = add an undo step (drags record on start). */
  private edited(m: Mechanism, record = true): void {
    if (record && !this.dragging) this.record();
    this.mech = m;
    window.clearTimeout(this.buildTimer);
    this.buildTimer = window.setTimeout(() => void this.rebuild(), this.dragging ? 10 : 60);
  }

  private async rebuild(): Promise<void> {
    const seq = ++this.buildSeq;
    const res = await this.client.build(encode(this.mech));
    if (seq !== this.buildSeq) return; // a newer edit superseded this build
    const status = $('status');
    if (!res.ok) {
      status.dataset.kind = 'build';
      status.textContent = res.error;
      status.hidden = false;
      return;
    }
    if (status.dataset.kind === 'build') status.hidden = true;
    this.derivation = res.derivation;
    this.lastBuildMs = res.ms;
    this.params = paramVector(this.mech);
    history.replaceState(null, '', encodeHash(this.mech));
    this.pushSettings();
    this.mode = defaultMode(res.derivation);
    this.client.send({ type: 'mode', mode: this.mode });
    this.ik = false;
    this.client.flag('Ik', 0);

    this.stage.clear();
    this.robot = new RobotView(this.stage, this.mech);
    this.stage.castShadows();
    this.robot.showFrames = this.tuning.frames;
    this.robot.showTrail = this.tuning.trail;
    if (!this.dragging) {
      const b = this.robot.bounds();
      this.stage.setView(b.cx, b.cy, b.width, b.height);
    }
    if (this.stageName === 'build') this.robot.select(this.editor.selection);
    this.gizmos.select(this.stageName === 'build' ? this.editor.selection : -1);
    // the loop closures are solved numerically from q(0): report when they cannot close
    window.setTimeout(() => {
      if (this.snap.v('NC') > 0 && this.snap.v('CONSTRAINT') > 1e-3) {
        const st = $('status');
        st.dataset.kind = 'build';
        st.textContent = `A loop cannot close in this pose (gap ${this.snap.v('CONSTRAINT').toFixed(3)} m): adjust lengths, ports or initial angles.`;
        st.hidden = false;
      }
    }, 80);
    if (!this.dragging) this.refreshStage();
  }

  private pushSettings(): void {
    const t = this.tuning;
    const flags: [Flag, number][] = [
      ['Umax', t.umax],
      ['PdWn', t.pdWn],
      ['CtWn', t.ctWn],
      ['RefWn', t.refWn],
      ['LqrQx', t.lqrQx],
      ['LqrQv', t.lqrQv],
      ['LqrR', t.lqrR],
      ['SwingGain', t.swing],
      ['Coriolis', t.coriolis ? 1 : 0],
    ];
    for (const [f, v] of flags) this.client.flag(f, v);
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
      this.client.send({ type: 'reset' });
      this.robot?.resetTrail();
    }
    this.robot?.select(s === 'build' ? this.editor.selection : -1);
    this.gizmos.select(s === 'build' ? this.editor.selection : -1);
    this.gizmos.visible = s === 'build';
    this.gizmos.connectFrom = null;
    this.closePopover();
    this.refreshStage();
  }

  /** Re-render the side panel and overlays for the current stage. */
  private refreshStage(): void {
    const d = this.derivation;
    const head = $('side-head');
    const side = $('side');
    head.replaceChildren();
    this.panel.clear();
    if (!d) return;
    const next = (label: string, to: StageName) => {
      const b = document.createElement('button');
      b.type = 'button';
      b.className = 'btn primary';
      b.textContent = label;
      b.addEventListener('click', () => this.setStage(to));
      return b;
    };
    const title = (t: string, sub: string) => {
      const el = document.createElement('div');
      el.className = 'side-title';
      el.innerHTML = `<b>${t}</b><span>${sub}</span>`;
      return el;
    };

    if (this.stageName === 'build') {
      const loops = d.closures.length;
      const tools = document.createElement('div');
      tools.className = 'head-tools';
      for (const [label, redo, hint] of [
        ['↶', false, 'undo (Ctrl+Z)'],
        ['↷', true, 'redo (Ctrl+Shift+Z)'],
      ] as const) {
        const b = document.createElement('button');
        b.type = 'button';
        b.className = 'btn small';
        b.textContent = label;
        b.title = hint;
        b.disabled = (redo ? this.redoStack : this.undoStack).length === 0;
        b.addEventListener('click', () => this.undo(redo));
        tools.append(b);
      }
      tools.append(next('Derive equations →', 'derive'));
      head.append(
        title(
          'Build',
          `${this.mech.bodies.length} bodies · ${d.n} DOF${loops ? ` − ${2 * loops} constraints` : ''} · ${d.nu} actuated · derived in ${this.lastBuildMs.toFixed(1)} ms`,
        ),
        tools,
      );
      side.replaceChildren(this.editorHost());
    } else if (this.stageName === 'derive') {
      head.append(
        title('Derivation', `Lagrange's method for this topology · ${this.lastBuildMs.toFixed(1)} ms`),
        next('Simulate →', 'simulate'),
      );
      this.panel.mount(derivationSections(this.mech, d));
    } else {
      head.append(title('Simulation', `${d.n} DOF · 1 kHz RK4`), this.sideTabs());
      this.panel.mount(
        liveSections({
          d,
          snap: this.snap,
          mode: () => this.mode,
          param: (i) => this.params[i] ?? Number.NaN,
        }),
      );
      this.plots.mount(this.plotSpecs(d));
    }
    this.buildModes();
    this.buildToggles();
    this.buildKeys();
  }

  private editorHost(): HTMLElement {
    this.editor.host.className = 'editor';
    return this.editor.host;
  }

  private sideTabs(): HTMLElement {
    let tab: 'eq' | 'tune' = 'eq';
    return segmented(
      [
        ['eq', 'Equations'],
        ['tune', 'Tuning'],
      ],
      tab,
      (v) => {
        tab = v;
        const d = this.derivation;
        if (!d) return;
        if (v === 'eq') {
          this.panel.mount(
            liveSections({
              d,
              snap: this.snap,
              mode: () => this.mode,
              param: (i) => this.params[i] ?? Number.NaN,
            }),
          );
        } else {
          this.panel.clear();
          $('side').replaceChildren(this.tuningPanel(d));
        }
      },
      false,
    );
  }

  private tuningPanel(d: Derivation): HTMLElement {
    const host = document.createElement('div');
    host.className = 'editor';
    const t = this.tuning;
    const card = (title: string, ...items: HTMLElement[]) => {
      const c = document.createElement('section');
      c.className = 'param-card';
      c.innerHTML = `<h3>${title}</h3>`;
      c.append(...items);
      host.append(c);
    };
    const flagSlider = (
      label: string,
      key: keyof Tuning,
      flag: Flag | null,
      o: Partial<Parameters<typeof slider>[0]>,
    ) =>
      slider({
        label,
        tex: true,
        min: 0,
        max: 1,
        value: t[key] as number,
        ...o,
        onInput: (v) => {
          (t[key] as number) = v;
          if (flag) this.client.flag(flag, v);
        },
      });

    // joint targets q*
    const targets = d.coords.map((c, i) => {
      const angular = c.kind === 'revolute';
      const b = this.mech.bodies[c.body]!;
      const range = angular ? 180 : Math.max(b.limit, 1);
      return slider({
        label: String.raw`${c.tex}^{*}`,
        tex: true,
        min: -range,
        max: range,
        step: angular ? 1 : 0.01,
        value: b.qTarget * (angular ? DEG : 1),
        unit: angular ? '°' : 'm',
        onInput: (v) => {
          b.qTarget = v / (angular ? DEG : 1);
          this.ik = false;
          this.client.flag('Ik', 0);
          this.client.send({ type: 'target', i, v: b.qTarget });
        },
      });
    });
    card('Targets q*', ...targets);

    card(
      'Controller',
      flagSlider(String.raw`u_{\max}`, 'umax', 'Umax', { min: 1, max: 300, log: true, unit: 'N·m | N' }),
      flagSlider(String.raw`u_{\text{key}}`, 'push', null, { min: 1, max: 100, log: true, unit: 'N·m | N' }),
      flagSlider(String.raw`\omega_{pd}`, 'pdWn', 'PdWn', { min: 0.5, max: 30, log: true, unit: 'rad/s' }),
      flagSlider(String.raw`\omega_{c}`, 'ctWn', 'CtWn', { min: 1, max: 40, log: true, unit: 'rad/s' }),
      flagSlider(String.raw`\omega_{\text{ref}}`, 'refWn', 'RefWn', {
        min: 0.5,
        max: 20,
        log: true,
        unit: 'rad/s',
      }),
      flagSlider('Q_x', 'lqrQx', 'LqrQx', { min: 0.1, max: 1000, log: true }),
      flagSlider(String.raw`Q_{\dot x}`, 'lqrQv', 'LqrQv', { min: 0.01, max: 100, log: true }),
      flagSlider('R', 'lqrR', 'LqrR', { min: 0.001, max: 100, log: true }),
      flagSlider('k_E', 'swing', 'SwingGain', { min: 0, max: 40 }),
    );

    // symbolic coefficients: changing them needs no re-derivation
    const coeffs = d.params.map((p) =>
      slider({
        label: p.tex,
        tex: true,
        min: p.kind === 'r' ? -1 : 0,
        max: p.kind === 'm' ? 10 : p.kind === 'g' ? 20 : p.kind === 'beta' ? 2 : 2,
        step: 0.001,
        value: this.params[p.index] ?? p.value,
        onInput: (v) => {
          this.params[p.index] = v;
          this.client.send({ type: 'param', i: p.index, v });
        },
      }),
    );
    const note = document.createElement('p');
    note.className = 'prose';
    note.textContent =
      'The equations were derived with symbolic coefficients, so these only change numbers — no re-derivation. (Uniform-rod links keep r and I as set here until you rebuild.)';
    card('Coefficients', note, ...coeffs);
    return host;
  }

  private plotSpecs(d: Derivation): PlotSpec[] {
    const s = this.snap;
    const colors = [
      palette.accent,
      palette.input,
      palette.coriolis,
      palette.gravity,
      palette.inertial,
      0xe8eefc,
    ];
    const plain = (texName: string) =>
      texName.replace(/\\theta_\{(\d+)\}/, 'θ$1').replace(/d_\{(\d+)\}/, 'd$1');
    const row = Math.max(
      0,
      d.coords.findIndex((c) => !c.actuated),
    );
    return [
      {
        title: 'Configuration',
        unit: 'rad, m',
        series: d.coords.map((c, i) => ({
          label: plain(c.tex),
          color: css(colors[i % colors.length]!),
          get: () => s.v('Q', i),
        })),
      },
      {
        title: 'Energy',
        unit: 'J',
        series: [
          { label: 'T', color: css(palette.inertial), get: () => s.v('ENERGY', 0) },
          { label: 'V', color: css(palette.gravity), get: () => s.v('ENERGY', 1) },
          { label: 'E', color: '#e8eefc', get: () => s.v('ENERGY', 2) },
        ],
      },
      {
        title: `Row ${row + 1} of the equations of motion`,
        unit: 'N·m | N',
        series: [
          { label: 'Mq̈', color: css(palette.inertial), get: () => s.v('MQDD', row) },
          { label: 'Cq̇', color: css(palette.coriolis), get: () => s.v('CQD', row) },
          { label: 'N', color: css(palette.gravity), get: () => s.v('N', row) },
          { label: 'τ', color: css(palette.input), get: () => s.v('TAU', row), dash: [4, 4] },
        ],
      },
    ];
  }

  // ---------------------------------------------------------------- chrome

  private setMode(m: number): void {
    this.mode = m;
    this.client.send({ type: 'mode', mode: m });
    for (const b of $('modes').querySelectorAll<HTMLButtonElement>('button')) {
      const on = Number(b.dataset.mode) === m;
      b.classList.toggle('on', on);
      b.setAttribute('aria-pressed', String(on));
    }
    $('hint').textContent = MODES.find((x) => x.id === m)?.hint ?? '';
  }

  private buildModes(): void {
    const host = $('modes');
    host.replaceChildren();
    const d = this.derivation;
    if (!d || this.stageName !== 'simulate') {
      $('hint').textContent =
        this.stageName === 'build'
          ? 'Click a body to select it. Steel hinges are actuated, amber ones passive.'
          : 'Every formula on the right was derived symbolically for exactly this structure.';
      return;
    }
    availableModes(d).forEach((m, k) => {
      const b = document.createElement('button');
      b.type = 'button';
      b.dataset.mode = String(m.id);
      b.innerHTML = `<span class="num">${k + 1}</span>${m.label}`;
      b.addEventListener('click', () => this.setMode(m.id));
      host.append(b);
    });
    this.setMode(this.mode);
  }

  private buildToggles(): void {
    const host = $('toggles');
    host.replaceChildren();
    const t = this.tuning;
    const items: [keyof Tuning, string, () => void][] = [
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
      ['coriolis', 'Coriolis', () => this.client.flag('Coriolis', t.coriolis ? 1 : 0)],
    ];
    for (const [key, label, apply] of items) {
      if (key === 'coriolis' && this.stageName !== 'simulate') continue;
      const b = document.createElement('button');
      b.type = 'button';
      b.className = 'chip';
      b.textContent = label;
      if (key === 'coriolis')
        b.title = 'Switch off C(q, q̇)q̇ in the simulation (non-physical: watch the energy drift)';
      const sync = () => b.classList.toggle('on', t[key] as boolean);
      b.addEventListener('click', () => {
        (t[key] as boolean) = !(t[key] as boolean);
        sync();
        apply();
      });
      sync();
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
    const parts: string[] = [];
    let a = 0;
    d.coords.forEach((c) => {
      if (c.actuated && a < ACTUATOR_LABELS.length) {
        parts.push(
          `<span><kbd>${ACTUATOR_LABELS[a]}</kbd><span data-tex="${encodeURIComponent(c.tex)}"></span></span>`,
        );
        a++;
      }
    });
    if (d.tool && d.nu === d.n) parts.push('<span><kbd>drag</kbd>tool target</span>');
    else if (this.mech.bodies.some((b) => b.actuated && b.joint === 'prismatic'))
      parts.push('<span><kbd>click</kbd>rail target</span>');
    parts.push('<span><kbd>Space</kbd>kick</span><span><kbd>R</kbd>reset</span>');
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
  }

  private chrome(): void {
    const stages: [StageName, string][] = [
      ['build', 'Build'],
      ['derive', 'Derive'],
      ['simulate', 'Simulate'],
    ];
    const nav = $('stages');
    stages.forEach(([s, label], k) => {
      const b = document.createElement('button');
      b.type = 'button';
      b.dataset.stage = s;
      b.innerHTML = `<span class="num">${k + 1}</span>${label}`;
      b.addEventListener('click', () => this.setStage(s));
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
      if (kind === 'down') this.closePopover();
      if (kind === 'down') viewport.setPointerCapture(e.pointerId);
      const [x, y] = this.stage.toWorld(e.clientX, e.clientY);
      this.pointer(x, y, kind, e.buttons !== 0 || kind === 'down');
    };
    viewport.addEventListener('pointerdown', pointer('down'));
    viewport.addEventListener('pointermove', pointer('move'));
    viewport.addEventListener('pointerup', pointer('up'));

    window.addEventListener('keydown', (e) => this.key(e, true));
    window.addEventListener('keyup', (e) => this.key(e, false));
    window.addEventListener('blur', () => {
      for (let a = 0; a < ACTUATOR_KEYS.length; a++) this.client.send({ type: 'input', i: a, v: 0 });
      this.held.clear();
    });
  }

  private pointer(x: number, y: number, kind: 'down' | 'move' | 'up', pressed: boolean): void {
    const d = this.derivation;
    if (!d || !this.robot) return;
    if (this.stageName === 'build') {
      if (kind !== 'down') return;
      const i = this.robot.pick(x, y, this.snap, this.params);
      if (i >= 0) this.editor.select(i);
      return;
    }
    if (this.stageName !== 'simulate' || !pressed || kind === 'up') return;
    if (d.tool && d.nu === d.n) {
      this.client.flag('TargetX', x);
      this.client.flag('TargetY', y);
      if (!this.ik) {
        this.ik = true;
        this.client.flag('Ik', 1);
      }
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
    if (this.stageName === 'build') {
      if (!down) return;
      const k = e.key.toLowerCase();
      if ((e.ctrlKey || e.metaKey) && k === 'z') {
        this.undo(e.shiftKey);
        e.preventDefault();
      } else if ((e.ctrlKey || e.metaKey) && k === 'y') {
        this.undo(true);
        e.preventDefault();
      } else if (k === 'escape') {
        this.gizmos.connectFrom = null;
        this.closePopover();
        $('status').hidden = true;
      } else if (k === 'delete' || k === 'backspace') {
        this.editor.remove(this.editor.selection);
        e.preventDefault();
      }
      return;
    }
    if (e.metaKey || e.ctrlKey) return;
    if (this.stageName !== 'simulate') return;
    const k = e.key.toLowerCase();
    if (down && e.repeat) {
      if (this.held.has(k)) e.preventDefault();
      return;
    }
    if (down) this.held.add(k);
    else this.held.delete(k);
    const a = ACTUATOR_KEYS.findIndex(([neg, pos]) => k === neg || k === pos);
    const d = this.derivation;
    if (a >= 0 && d && a < d.nu) {
      const [neg] = ACTUATOR_KEYS[a]!;
      // arrows/letters: left = negative for prismatic, counter-clockwise (positive) for revolute
      const actuated = d.coords.map((c, i) => [c, i] as const).filter(([c]) => c.actuated);
      const kind = actuated[a]?.[0].kind;
      const sign = (k === neg ? -1 : 1) * (kind === 'revolute' ? -1 : 1);
      this.client.send({ type: 'input', i: a, v: down ? sign * this.tuning.push : 0 });
      e.preventDefault();
      return;
    }
    if (!down) return;
    if (k === ' ' && d) {
      const passive = d.coords.map((c, i) => [c, i] as const).filter(([c]) => !c.actuated);
      for (const [, i] of passive.length ? passive : d.coords.map((c, i) => [c, i] as const)) {
        this.client.send({ type: 'kick', i, v: (Math.random() < 0.5 ? -1 : 1) * 1.5 });
      }
      e.preventDefault();
    } else if (k === 'r') this.reset();
    else if (k === 'p') this.setRunning(!this.running);
    else if (/^[1-6]$/.test(k) && d) {
      const m = availableModes(d)[Number(k) - 1];
      if (m) this.setMode(m.id);
    }
  }

  // ---------------------------------------------------------------- frame loop

  private frame(now: number): void {
    this.client.pull();
    const simulating = this.stageName === 'simulate';
    this.robot?.update(this.snap, this.params, this.ik && simulating, simulating);
    this.stage.render();
    if (this.stageName === 'build' && this.robot)
      this.gizmos.update(this.mech, this.robot, this.snap, this.params);
    this.frames++;
    if (this.stageName === 'simulate' && now - this.lastPlot >= 1000 / PLOT_HZ) {
      this.plots.sample(this.snap.t);
      this.plots.draw();
      this.lastPlot = now;
    }
    if (now - this.lastEq >= 1000 / EQ_HZ) {
      this.panel.update();
      this.lastEq = now;
    }
    if (now - this.lastHud >= 500) {
      const fps = (this.frames * 1000) / (now - this.lastHud);
      this.frames = 0;
      this.lastHud = now;
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
