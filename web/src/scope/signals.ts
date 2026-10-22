/**
 * Every quantity the scope can plot, addressed by a short id. Ids are stable across mechanisms and slot
 * assignments, so presets, experiments and saved layouts can name what they want to see:
 *
 *   q:i qd:i qdd:i e:i qref:i tau:i text:i u:a       joints (i = coordinate, a = actuator)
 *   eom:i:{m|c|n|a|tau}                              row i of M q̈ + C q̇ + N + Aᵀλ = τ
 *   energy:{T|V|E|P|D|dE}  detM  eig:i               energy, power, inertia
 *   tool:{x|y} xref:{x|y} fext:{x|y|n} w taskerr     task space
 *   viol  lambda:j                                   loop closures
 *   slot:<key>:{err|effort|E|q<i>|u<a>}             the slot running controller <key>
 *   tele:<key>:<name>:<i>                            telemetry of controller <key>
 */

import { shortName } from '../control/pages';
import type { ControlStore } from '../control/store';
import { SLOT_COLORS } from '../control/store';
import { plain, subscript } from '../eq/tex';
import type { Derivation } from '../model/derivation';
import type { Snapshot } from '../sim/layout';

export interface Signal {
  id: string;
  label: string;
  unit: string;
  get: () => number;
  /** Fixed colour (slot signals use their slot's colour). */
  color?: number;
  dash?: number[];
}

export interface SignalGroup {
  name: string;
  ids: string[];
}

/** Friendly names and units of controller telemetry fields. */
const TELE: Record<string, [string, string]> = {
  kp: ['K_p', ''],
  kd: ['K_v', ''],
  ug: ['u_g', 'N·m'],
  qdd: ['q̈ cmd', 'rad/s²'],
  s: ['s (sliding)', 'rad/s'],
  lyap: ['V (Lyapunov)', 'J'],
  lyap_dot: ['V̇', 'W'],
  x: ['x', 'm'],
  xd: ['x_d', 'm'],
  force: ['F virtual', 'N'],
  tau_task: ['JᵀF', 'N·m'],
  tau_null: ['N⊥τ₀', 'N·m'],
  leak: ['leakage ‖JM⁻¹N⊥τ₀‖', 'm/s²'],
  w: ['manipulability w', ''],
  e: ['e = x − x_d', 'm'],
  fext: ['F_ext', 'N'],
  xdd_cmd: ['ẍ cmd', 'm/s²'],
  xv: ['x_v (virtual)', 'm'],
  xv_dot: ['ẋ_v', 'm/s'],
  cost: ['cost', ''],
  iterations: ['QP iterations', ''],
  active: ['active bounds', ''],
  size: ['horizon N', ''],
  energy: ['E − E*', 'J'],
  orient_err: ['orientation error', 'rad'],
  elbow_err: ['elbow error', 'm'],
  leak_sec: ['leakage into secondary tasks', 'm/s² | rad/s²'],
  energy_ref: ['E_ref', 'J'],
};

export class Signals {
  constructor(
    private readonly d: Derivation,
    private readonly snap: Snapshot,
    private readonly store: ControlStore,
  ) {}

  private coord(i: number): string {
    return plain(this.d.coords[i]?.tex ?? `q${i + 1}`);
  }

  private actuator(a: number): string {
    const i = this.d.coords.map((c, k) => (c.actuated ? k : -1)).filter((k) => k >= 0)[a];
    return `u ${i === undefined ? a + 1 : this.coord(i)}`;
  }

  private unit(i: number, rate = 0): string {
    const angular = this.d.coords[i]?.kind !== 'prismatic';
    return [angular ? 'rad' : 'm', angular ? 'rad/s' : 'm/s', angular ? 'rad/s²' : 'm/s²'][rate]!;
  }

  private kind(key: string): number {
    return this.store.reg.specs.find((s) => s.key === key)?.id ?? -1;
  }

  /** The signal with this id, or undefined if the id does not apply to this mechanism. */
  resolve(id: string): Signal | undefined {
    const s = this.snap;
    const [head, a, b] = id.split(':');
    const i = Number(a);
    const n = this.d.n;
    const joint = (f: Parameters<Snapshot['v']>[0], label: string, unit: string): Signal | undefined =>
      i >= 0 && i < n ? { id, label, unit, get: () => s.v(f, i) } : undefined;
    switch (head) {
      case 'q':
        return joint('Q', this.coord(i), this.unit(i));
      case 'qd':
        return joint('QD', `${this.coord(i)}̇`, this.unit(i, 1));
      case 'qdd':
        return joint('QDD', `${this.coord(i)}̈`, this.unit(i, 2));
      case 'qref':
        return joint('Q_REF', `${this.coord(i)} ref`, this.unit(i));
      case 'tau':
        return joint('TAU', `τ ${this.coord(i)}`, 'N·m | N');
      case 'text':
        return joint('TAU_EXT', `τ_ext ${this.coord(i)}`, 'N·m | N');
      case 'e':
        return i < n
          ? { id, label: `e ${this.coord(i)}`, unit: this.unit(i), get: () => s.v('Q_REF', i) - s.v('Q', i) }
          : undefined;
      case 'u':
        return i < this.d.nu
          ? { id, label: this.actuator(i), unit: 'N·m | N', get: () => s.v('U', i) }
          : undefined;
      case 'eom': {
        if (i >= n) return undefined;
        const row = `row ${i + 1}`;
        const terms: Record<string, [string, () => number]> = {
          m: [`Mq̈ ${row}`, () => s.v('MQDD', i)],
          c: [`Cq̇ ${row}`, () => s.v('CQD', i)],
          n: [`N ${row}`, () => s.v('N', i)],
          a: [`Aᵀλ ${row}`, () => s.v('AT_LAMBDA', i)],
          tau: [`τ ${row}`, () => s.v('TAU', i)],
        };
        const t = terms[a === undefined ? '' : (b ?? '')];
        return t ? { id, label: t[0], unit: 'N·m | N', get: t[1] } : undefined;
      }
      case 'energy': {
        const e: Record<string, [string, () => number]> = {
          T: ['T kinetic', () => s.v('ENERGY', 0)],
          V: ['V potential', () => s.v('ENERGY', 1)],
          E: ['E = T + V', () => s.v('ENERGY', 2)],
          P: ['q̇ᵀτ power in', () => s.v('POWER')],
          D: ['q̇ᵀβq̇ friction', () => s.v('DISSIPATION')],
          dE: ['Ė = q̇ᵀτ − q̇ᵀβq̇', () => s.v('POWER') - s.v('DISSIPATION')],
          E0: ['E₀ (start of budget)', () => s.v('E0')],
          // E plus what friction took minus what inputs, pushes, stops and loops gave: constant = E₀
          Ebal: ['E − net work (= E₀)', () => s.v('E0') + s.v('BUDGET')],
        };
        const t = e[a ?? ''];
        return t
          ? { id, label: t[0], unit: a === 'P' || a === 'D' || a === 'dE' ? 'W' : 'J', get: t[1] }
          : undefined;
      }
      case 'work': {
        const w: Record<string, [string, number]> = {
          in: ['work by inputs ∫q̇ᵀBu', 0],
          ext: ['work by push & kicks', 1],
          stop: ['work by end stops', 2],
          fric: ['friction loss ∫q̇ᵀβq̇', 3],
          con: ['work by loop forces', 4],
        };
        const t = w[a ?? ''];
        if (!t || (a === 'con' && !this.d.closures.length)) return undefined;
        return { id, label: t[0], unit: 'J', get: () => s.v('WORK', t[1]) };
      }
      case 'recoveries':
        return { id, label: 'integration recoveries since reset', unit: '', get: () => s.v('RECOVERY') };
      case 'retries':
        return { id, label: 'loop integration retries since reset', unit: '', get: () => s.v('RECOVERY', 1) };
      case 'budget':
        return { id, label: 'budget residual E − E₀ − ΣW', unit: 'J', get: () => s.v('BUDGET') };
      case 'detM':
        return { id, label: 'det M', unit: '', get: () => s.v('DET_M') };
      case 'eig':
        return joint('EIG_M', `λ${subscript(i + 1)}(M)`, '');
      case 'tool':
      case 'xref': {
        if (!this.d.tool) return undefined;
        const r = a === 'y' ? 1 : 0;
        return head === 'tool'
          ? { id, label: `tool ${a}`, unit: 'm', get: () => s.v('TOOL_P', r) }
          : { id, label: `x_d ${a}`, unit: 'm', get: () => s.v('X_REF', r) };
      }
      case 'fext':
        if (!this.d.tool) return undefined;
        return a === 'n'
          ? { id, label: '‖F_ext‖', unit: 'N', get: () => Math.hypot(s.v('FEXT'), s.v('FEXT', 1)) }
          : { id, label: `F_ext ${a}`, unit: 'N', get: () => s.v('FEXT', a === 'y' ? 1 : 0) };
      case 'orient':
      case 'orienterr': {
        if (!this.d.tool) return undefined;
        const tool = this.d.bodies.length + 1;
        const phi = () => s.frame(tool)[2];
        return head === 'orient'
          ? { id, label: 'tool angle φ_t', unit: 'rad', get: phi }
          : {
              id,
              label: 'φ* − φ_t',
              unit: 'rad',
              get: () => {
                if (!s.v('GOALS')) return Number.NaN; // orientation goal off
                const e = s.v('GOALS', 1) - phi();
                return Math.atan2(Math.sin(e), Math.cos(e));
              },
            };
      }
      case 'elbow':
      case 'elbowerr': {
        if (!this.d.tool) return undefined;
        const at = () => s.frame(Math.round(s.v('GOALS', 5)) + 1);
        if (head === 'elbow') {
          const r = a === 'y' ? 1 : 0;
          return { id, label: `elbow ${a}`, unit: 'm', get: () => at()[r] };
        }
        return {
          id,
          label: '‖elbow* − elbow‖',
          unit: 'm',
          get: () =>
            s.v('GOALS', 2) ? Math.hypot(s.v('GOALS', 3) - at()[0], s.v('GOALS', 4) - at()[1]) : Number.NaN,
        };
      }
      case 'w':
        return this.d.tool
          ? { id, label: 'manipulability w', unit: '', get: () => s.v('MANIPULABILITY') }
          : undefined;
      case 'taskerr':
        return this.d.tool
          ? {
              id,
              label: '‖x − x_d‖',
              unit: 'm',
              get: () => Math.hypot(s.v('TOOL_P') - s.v('X_REF'), s.v('TOOL_P', 1) - s.v('X_REF', 1)),
            }
          : undefined;
      case 'viol':
        return this.d.closures.length
          ? { id, label: 'max |φ(q)|', unit: 'm', get: () => s.v('CONSTRAINT') }
          : undefined;
      case 'lambda':
        return i < this.d.phi.length
          ? { id, label: `λ${subscript(i + 1)}`, unit: 'N', get: () => s.v('LAMBDA', i) }
          : undefined;
      case 'slot':
        return this.slotSignal(id, a ?? '', b ?? '');
      case 'tele':
        return this.teleSignal(id, a ?? '', b ?? '', Number(id.split(':')[3] ?? 0));
      default:
        return undefined;
    }
  }

  private slotStyle(kind: number): Pick<Signal, 'color' | 'dash'> {
    const slot = this.store.slotOf(kind);
    return slot < 0 ? {} : { color: SLOT_COLORS[slot]!, ...(slot > 0 ? { dash: [5, 3] } : {}) };
  }

  private slotSignal(id: string, key: string, what: string): Signal | undefined {
    const kind = this.kind(key);
    const spec = this.store.spec(kind);
    if (!spec) return undefined;
    const slot = () => this.snap.slotOf(kind);
    const read =
      (f: Parameters<NonNullable<ReturnType<typeof slot>>['v']>[0], i = 0) =>
      () =>
        slot()?.v(f, i) ?? Number.NaN;
    const name = shortName(spec);
    const style = this.slotStyle(kind);
    if (what === 'err') {
      return {
        id,
        label: `${name} error`,
        unit: 'rad | m',
        get: () => {
          const v = slot();
          return v ? v.v(this.snap.v('USE_TOOL') ? 'ERR_X' : 'ERR_Q') : Number.NaN;
        },
        ...style,
      };
    }
    if (what === 'effort')
      return { id, label: `${name} ‖u‖`, unit: 'N·m | N', get: read('EFFORT'), ...style };
    if (what === 'recoveries')
      return { id, label: `${name} recoveries`, unit: '', get: read('RECOVERY'), ...style };
    if (what === 'E') return { id, label: `${name} E`, unit: 'J', get: read('ENERGY', 2), ...style };
    const j = Number(what.slice(1));
    if (what[0] === 'q' && j < this.d.n) {
      return { id, label: `${name} ${this.coord(j)}`, unit: this.unit(j), get: read('Q', j), ...style };
    }
    if (what[0] === 'u' && j < this.d.nu) {
      return { id, label: `${name} ${this.actuator(j)}`, unit: 'N·m | N', get: read('U', j), ...style };
    }
    return undefined;
  }

  private teleSignal(id: string, key: string, name: string, i: number): Signal | undefined {
    const kind = this.kind(key);
    const spec = this.store.spec(kind);
    const field = spec?.telemetry[name];
    if (!spec || !field) return undefined;
    const [off, len] = field;
    const [base, unit] = TELE[name] ?? [name, ''];
    let label = base;
    if (len === 2) label = `${base} ${i === 1 ? 'y' : 'x'}`;
    else if (len > 2) {
      // a joint vector (one entry per coordinate); matrices and long plans are not plottable signals
      if (len !== this.snap.layout.maxn || i >= this.d.n) return undefined;
      label = `${base} ${this.coord(i)}`;
    }
    return {
      id,
      label: `${shortName(spec)}: ${label}`,
      unit,
      get: () => this.snap.slotOf(kind)?.tele(off + i) ?? Number.NaN,
      ...this.slotStyle(kind),
    };
  }

  /** The signals offered in the picker, grouped, for the current mechanism and running controllers. */
  groups(): SignalGroup[] {
    const { n, nu } = this.d;
    const range = (k: number, f: (i: number) => string) => Array.from({ length: k }, (_, i) => f(i));
    const groups: SignalGroup[] = [
      {
        name: 'Joints',
        ids: [...range(n, (i) => `q:${i}`), ...range(n, (i) => `qd:${i}`), ...range(n, (i) => `e:${i}`)],
      },
      {
        name: 'Forces',
        ids: [...range(nu, (a) => `u:${a}`), ...range(n, (i) => `tau:${i}`), ...range(n, (i) => `text:${i}`)],
      },
      {
        name: 'Equations of motion',
        ids: Array.from({ length: n }, (_, i) =>
          ['m', 'c', 'n', ...(this.d.closures.length ? ['a'] : []), 'tau'].map((t) => `eom:${i}:${t}`),
        ).flat(),
      },
      {
        name: 'Energy',
        ids: [
          'energy:T',
          'energy:V',
          'energy:E',
          'energy:P',
          'energy:D',
          'energy:dE',
          'energy:E0',
          'energy:Ebal',
        ],
      },
      {
        name: 'Energy budget',
        ids: ['work:in', 'work:ext', 'work:stop', 'work:fric', 'work:con', 'budget'],
      },
      { name: 'Integration', ids: ['recoveries', 'retries'] },
      { name: 'Inertia', ids: ['detM', ...range(n, (i) => `eig:${i}`)] },
    ];
    if (this.d.tool) {
      groups.push({
        name: 'Task space',
        ids: ['tool:x', 'tool:y', 'xref:x', 'xref:y', 'taskerr', 'fext:x', 'fext:y', 'fext:n', 'w'],
      });
      groups.push({ name: 'Task hierarchy', ids: ['orient', 'orienterr', 'elbow:x', 'elbow:y', 'elbowerr'] });
    }
    if (this.d.closures.length)
      groups.push({ name: 'Loops', ids: ['viol', ...range(this.d.phi.length, (j) => `lambda:${j}`)] });
    const running = this.store.slots.filter((k) => k >= 0);
    const specs = running.map((k) => this.store.spec(k)).filter((s) => s !== undefined);
    groups.push({
      name: 'Compare slots',
      ids: ['err', 'effort', 'E'].flatMap((w) => specs.map((s) => `slot:${s.key}:${w}`)),
    });
    for (const s of specs) {
      const ids = Object.entries(s.telemetry).flatMap(([name, [, len]]) =>
        len === 1
          ? [`tele:${s.key}:${name}:0`]
          : len === 2
            ? [0, 1].map((i) => `tele:${s.key}:${name}:${i}`)
            : len <= this.snap.layout.maxn
              ? range(n, (i) => `tele:${s.key}:${name}:${i}`)
              : [],
      );
      if (ids.length) groups.push({ name: `${s.name} internals`, ids });
    }
    return groups
      .map((g) => ({ ...g, ids: g.ids.filter((id) => this.resolve(id)) }))
      .filter((g) => g.ids.length);
  }
}
