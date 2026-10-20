import 'katex/dist/katex.min.css';
import './styles.css';

import { EquationPanel } from './eq/panel';
import { commonSections } from './eq/sections';
import { Plots } from './plots/plots';
import { Stage } from './scene/stage';
import { SimClient } from './sim/client';
import type { Snapshot } from './sim/layout';
import { arm2System } from './systems/arm2';
import { cartpoleSystem } from './systems/cartpole';
import type { SceneHandle, SimApi, SystemDef, UiParams } from './systems/types';
import { buildParams } from './ui/controls';

const SYSTEMS: SystemDef[] = [cartpoleSystem, arm2System];
const EQ_HZ = 30;
const PLOT_HZ = 30;

const $ = <T extends HTMLElement = HTMLElement>(id: string) => document.getElementById(id) as T;

class App {
  private sys: SystemDef = cartpoleSystem;
  private snap!: Snapshot;
  private ui: UiParams = {};
  private mode = 0;
  private scene: SceneHandle | null = null;
  private readonly panel = new EquationPanel($('equations'));
  private readonly plots = new Plots($('plots'));
  private lastEq = 0;
  private lastPlot = 0;
  private lastHud = 0;
  private frames = 0;
  private fps = 0;
  private running = true;
  private readonly held = new Set<string>();

  private readonly api: SimApi;

  constructor(
    private readonly client: SimClient,
    private readonly stage: Stage,
  ) {
    const self = this;
    this.api = {
      client,
      get snap() {
        return self.snap;
      },
      get ui() {
        return self.ui;
      },
      mode: () => this.mode,
      flag: (f, v) => client.flag(f, v),
      input: (i, v) => client.send({ type: 'input', i, v }),
      kick: (i, v) => client.send({ type: 'kick', i, v }),
      state: (i, q, qd) => client.send({ type: 'state', i, q, qd }),
    };
  }

  start(): void {
    this.buildChrome();
    const fromHash = SYSTEMS.find((s) => `#${s.key}` === location.hash);
    this.load(fromHash ?? cartpoleSystem);
    const loop = (now: number) => {
      this.frame(now);
      requestAnimationFrame(loop);
    };
    requestAnimationFrame(loop);
  }

  private load(sys: SystemDef): void {
    this.sys = sys;
    history.replaceState(null, '', `#${sys.key}`);
    this.ui = { showFrames: 1, showTrail: 1, coriolis: 1 };
    for (const p of sys.params) this.ui[p.key] = p.value;
    this.mode = sys.defaultMode;
    this.snap = this.client.init(
      sys.id,
      sys.model.nq,
      sys.modelParams(this.ui),
      this.mode,
      sys.flags(this.ui),
    );

    this.stage.clear();
    this.stage.setView(sys.view.cx, sys.view.cy, sys.view.width, sys.view.height);
    this.scene = sys.scene(this.stage, this.api);
    this.stage.castShadows();

    const ctx = { model: sys.model, snap: this.snap, q: sys.q, param: (id: string) => this.modelParam(id) };
    const common = commonSections(ctx);
    const notation = common.filter((s) => s.id === 'notation');
    this.panel.mount([
      ...common.filter((s) => s.id !== 'notation'),
      ...sys.sections(ctx, this.api),
      ...notation,
    ]);
    this.plots.mount(sys.plots(this.api));
    buildParams($('params'), sys.params, this.ui, () => this.pushParams());

    for (const b of $('systems').querySelectorAll<HTMLButtonElement>('button')) {
      b.classList.toggle('on', b.dataset.key === sys.key);
    }
    this.buildModes();
    this.buildToggles();
    $('keys').innerHTML = sys.keys.map((k) => `<span><kbd>${k.keys}</kbd>${k.action}</span>`).join('');
  }

  private modelParam(id: string): number {
    const i = this.sys.model.params.findIndex((p) => p.id === id);
    return this.sys.modelParams(this.ui)[i] ?? Number.NaN;
  }

  private pushParams(): void {
    this.sys.modelParams(this.ui).forEach((v, i) => {
      this.client.send({ type: 'param', i, v });
    });
    for (const [f, v] of this.sys.flags(this.ui)) this.client.flag(f, v);
  }

  private setMode(m: number): void {
    this.mode = m;
    this.client.send({ type: 'mode', mode: m });
    for (const b of $('modes').querySelectorAll<HTMLButtonElement>('button')) {
      b.classList.toggle('on', Number(b.dataset.mode) === m);
      b.setAttribute('aria-pressed', String(Number(b.dataset.mode) === m));
    }
    $('hint').textContent = this.sys.modes.find((x) => x.id === m)?.hint ?? '';
  }

  private reset(): void {
    this.client.send({ type: 'reset' });
    this.scene?.reset();
  }

  private setRunning(on: boolean): void {
    this.running = on;
    this.client.send({ type: 'run', running: on });
    $('run').textContent = on ? 'Pause' : 'Run';
    $('run').classList.toggle('on', !on);
  }

  private buildModes(): void {
    const host = $('modes');
    host.replaceChildren();
    this.sys.modes.forEach((m, k) => {
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
    const items: [string, string, (on: boolean) => void][] = [
      ['showFrames', 'Frames', () => {}],
      ['showTrail', 'Trail', () => this.scene?.reset()],
      ['coriolis', 'Coriolis', (on) => this.client.flag('Coriolis', on ? 1 : 0)],
    ];
    for (const [key, label, apply] of items) {
      const b = document.createElement('button');
      b.type = 'button';
      b.className = 'chip';
      b.textContent = label;
      if (key === 'coriolis')
        b.title = 'Switch off C(q, q̇)q̇ in the simulation (non-physical: watch the energy drift)';
      const sync = () => b.classList.toggle('on', this.ui[key] !== 0);
      b.addEventListener('click', () => {
        this.ui[key] = this.ui[key] ? 0 : 1;
        sync();
        apply(this.ui[key] !== 0);
      });
      sync();
      host.append(b);
    }
  }

  private buildChrome(): void {
    const tabs = $('systems');
    for (const s of SYSTEMS) {
      const b = document.createElement('button');
      b.type = 'button';
      b.dataset.key = s.key;
      b.textContent = s.title;
      b.addEventListener('click', () => {
        if (s !== this.sys) this.load(s);
      });
      tabs.append(b);
    }
    $('run').addEventListener('click', () => this.setRunning(!this.running));
    $('reset').addEventListener('click', () => this.reset());
    $<HTMLSelectElement>('speed').addEventListener('change', (e) => {
      this.client.send({ type: 'speed', speed: Number((e.target as HTMLSelectElement).value) });
    });

    for (const b of document.querySelectorAll<HTMLButtonElement>('.sidetabs button')) {
      b.addEventListener('click', () => {
        for (const x of document.querySelectorAll('.sidetabs button')) x.classList.toggle('on', x === b);
        $('equations').hidden = b.dataset.tab !== 'eq';
        $('params').hidden = b.dataset.tab !== 'params';
      });
    }

    const canvas = $('viewport');
    const pointer = (kind: 'down' | 'move' | 'up') => (e: PointerEvent) => {
      if ((e.target as HTMLElement).closest('.overlay')) return;
      if (kind === 'down') canvas.setPointerCapture(e.pointerId);
      const [x, y] = this.stage.toWorld(e.clientX, e.clientY);
      this.scene?.pointer?.(x, y, kind);
    };
    canvas.addEventListener('pointerdown', pointer('down'));
    canvas.addEventListener('pointermove', pointer('move'));
    canvas.addEventListener('pointerup', pointer('up'));

    const onKey = (down: boolean) => (e: KeyboardEvent) => {
      if ((e.target as HTMLElement).closest('input, select, textarea') || e.metaKey || e.ctrlKey) return;
      const k = e.key.toLowerCase();
      if (down && e.repeat && this.held.has(k)) {
        e.preventDefault();
        return;
      }
      if (down) this.held.add(k);
      else this.held.delete(k);
      if (this.sys.onKey(e, down, this.api)) {
        e.preventDefault();
        return;
      }
      if (!down) return;
      if (k === 'r') this.reset();
      else if (k === 'p') this.setRunning(!this.running);
      else if (/^[1-9]$/.test(k)) {
        const m = this.sys.modes[Number(k) - 1];
        if (m) this.setMode(m.id);
      }
    };
    window.addEventListener('keydown', onKey(true));
    window.addEventListener('keyup', onKey(false));
    window.addEventListener('blur', () => {
      for (const k of this.held) this.sys.onKey(new KeyboardEvent('keyup', { key: k }), false, this.api);
      this.held.clear();
    });
  }

  private frame(now: number): void {
    this.client.pull();
    this.scene?.update();
    this.stage.render();
    this.frames++;

    if (now - this.lastPlot >= 1000 / PLOT_HZ) {
      this.plots.sample(this.snap.t);
      this.plots.draw();
      this.lastPlot = now;
    }
    if (now - this.lastEq >= 1000 / EQ_HZ) {
      this.panel.update();
      this.lastEq = now;
    }
    if (now - this.lastHud >= 500) {
      this.fps = (this.frames * 1000) / (now - this.lastHud);
      this.frames = 0;
      this.lastHud = now;
      const s = this.client.stats;
      $('stats').innerHTML =
        `<span><b>${this.fps.toFixed(0)}</b> fps</span>` +
        `<span><b>${(s.stepsPerSecond / 1000).toFixed(2)}</b> kHz physics</span>` +
        `<span><b>${s.stepUs.toFixed(2)}</b> µs/step</span>` +
        `<span><b>t</b> ${this.snap.t.toFixed(1)} s</span>` +
        `<span class="muted">${this.stage.backend}${s.shared ? ' · shared memory' : ''}</span>`;
    }
  }
}

async function main(): Promise<void> {
  const stage = new Stage($('viewport'));
  const [client] = await Promise.all([SimClient.start(), stage.init()]);
  new App(client, stage).start();
}

main().catch((err: unknown) => {
  console.error(err);
  $('viewport').insertAdjacentHTML(
    'beforeend',
    `<div class="fatal"><h2>Could not start the simulation</h2><pre>${String(err)}</pre></div>`,
  );
});
