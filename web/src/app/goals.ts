/**
 * The Goals and Model tabs: what the controllers aim for (joint or tool targets, and below the tool a
 * hierarchy of further goals — orientation, elbow, posture — each in the null space of the ones above), how
 * the world pushes back, and the symbolic coefficients of the model.
 */
import { cachedTaskSpace } from '../control/nullspace';
import { toolChainDof } from '../control/page';
import type { EquationPanel, SectionSpec } from '../eq/panel';
import { fix, sci, texMat } from '../eq/tex';
import type { Derivation } from '../model/derivation';
import type { Mechanism } from '../model/description';
import { physicalControl } from '../model/fields';
import type { SimClient } from '../sim/client';
import type { Snapshot } from '../sim/layout';
import {
  button,
  card,
  DEG,
  el,
  note,
  row,
  type Slider,
  type SliderOpts,
  segmented,
  slider,
} from '../ui/widgets';
import {
  defaultSettings,
  GOAL_W,
  type Numeric,
  type Settings,
  setSetting,
  TUNING,
  toolTargets,
} from './settings';
import { taskList } from './tasks';

type Range = Pick<SliderOpts, 'min' | 'max'> & Partial<Pick<SliderOpts, 'log' | 'unit' | 'hint' | 'format'>>;

function settingSlider(client: SimClient, s: Settings, key: Numeric, label: string, range: Range) {
  return slider({
    label,
    tex: true,
    value: s[key],
    ...range,
    onInput: (v) => setSetting(client, s, key, v),
  });
}

/** Tuning values span decades: small ones in scientific notation rather than rounded to 0. */
const scale = (v: number) => (Math.abs(v) < 0.01 && v !== 0 ? sci(v) : fix(v));

/** A folded group of tuning sliders with a button back to the defaults. */
function tuningFold(g: GoalsContext, group: keyof typeof TUNING, title: string): HTMLElement {
  const { client, settings } = g;
  const specs = TUNING[group];
  const sliders = specs.map((t) =>
    settingSlider(client, settings, t.key, t.label, { log: true, format: scale, ...t }),
  );
  const defaults = defaultSettings(g.snap.layout);
  const reset = button(
    'Defaults',
    () => {
      for (const t of specs) setSetting(client, settings, t.key, defaults[t.key]);
      g.refresh();
    },
    'btn small',
    'back to the values the experiments are verified with',
  );
  const changed = specs.some((t) => settings[t.key] !== defaults[t.key]);
  const fold = el(
    'details',
    'fold',
    el('summary', '', changed ? `${title} (changed)` : title),
    note('The same numbers for every mechanism. Hover a name for what it does.'),
    ...sliders,
    el('div', 'card-actions', reset),
  );
  if (changed) (fold as HTMLDetailsElement).open = true;
  return fold;
}

export interface GoalsContext {
  d: Derivation;
  mech: Mechanism;
  snap: Snapshot;
  client: SimClient;
  settings: Settings;
  panel: EquationPanel;
  /** The joint targets changed (the tool target is dropped). */
  onJointTarget: () => void;
  /** Toggle following the tool target. */
  setUseTool: (on: boolean) => void;
  /** Re-render the tab (after a goal was switched on or off). */
  refresh: () => void;
}

/** Bodies whose joint can serve as the elbow: on the chain to the tool, and not fixed to the ground. */
export function elbowCandidates(mech: Mechanism): number[] {
  const out: number[] = [];
  let b = mech.tool;
  while (b >= 0) {
    const body = mech.bodies[b]!;
    if (body.parent >= 0 || body.joint === 'prismatic') out.unshift(b);
    b = body.parent;
  }
  return out;
}

export function goalsTab(g: GoalsContext): HTMLElement {
  const { d, mech, client, settings } = g;
  const host = el('div', 'editor');

  const targets = d.coords.map((c, i) => {
    const angular = c.kind === 'revolute';
    const b = mech.bodies[c.body]!;
    const range = angular ? 180 : Math.max(b.limit, 1);
    return slider({
      label: String.raw`${c.tex}^{*}`,
      tex: true,
      min: -range,
      max: range,
      step: angular ? 1 : 0.01,
      value: client.targets[i]! * (angular ? DEG : 1),
      unit: angular ? '°' : 'm',
      onInput: (v) => {
        g.onJointTarget();
        client.send({ type: 'target', i, v: v / (angular ? DEG : 1) });
      },
    });
  });
  const task = card('Primary task');
  if (toolTargets(d)) {
    task.append(
      segmented(
        [
          ['joint', 'Joint targets q*'],
          ['tool', 'Tool target x_d'],
        ],
        settings.useTool ? 'tool' : 'joint',
        (v) => {
          g.setUseTool(v === 'tool');
          g.refresh(); // fold or unfold the joint targets (not from a slider drag: that would end the drag)
        },
      ),
      note(
        `Drag in the viewport to move the tool target; Shift-drag to push the tool with a spring (at most ${settings.pushMax} N, like a hand).`,
      ),
    );
  } else if (d.tool !== null && d.nu > 0) {
    const rail = mech.bodies.some((b) => b.actuated && b.joint === 'prismatic');
    task.append(
      note(
        `Underactuated: the mechanism can only rest where its passive joints balance, and a tool point generally is not such a place — so it is given joint targets q*.${rail ? ' Click or drag in the viewport to move the rail target.' : ''}`,
      ),
    );
  }
  task.append(
    ...(d.closures.length
      ? [
          note(
            'With loops, q* is moved onto the constraint manifold: the actuated targets are kept, the rest follows.',
          ),
        ]
      : []),
    // with a tool target the joint targets only serve as the posture: fold them away
    ...(settings.useTool && d.tool
      ? [el('details', 'fold', el('summary', '', 'Joint targets q* (the posture goal)'), ...targets)]
      : targets),
    settingSlider(client, settings, 'refWn', String.raw`\omega_{\text{ref}}`, {
      min: 0.5,
      max: 20,
      log: true,
      unit: 'rad/s',
    }),
  );

  const secondary = hierarchyCard(g);
  if (g.d.tool && toolTargets(g.d)) secondary.append(taskList(g.mech, g.snap, g.client, g.refresh));

  const world = card(
    'Interaction & limits',
    settingSlider(client, settings, 'pushK', String.raw`k_{\text{push}}`, {
      min: 10,
      max: 2000,
      log: true,
      unit: 'N/m',
      hint: 'stiffness of the Shift-drag spring',
    }),
    settingSlider(client, settings, 'umax', String.raw`u_{\max}`, {
      min: 1,
      max: 300,
      log: true,
      unit: 'N·m | N',
    }),
    settingSlider(client, settings, 'key', String.raw`u_{\text{key}}`, {
      min: 1,
      max: 100,
      log: true,
      unit: 'N·m | N',
      hint: 'input of the keyboard keys',
    }),
    settingSlider(client, settings, 'kick', String.raw`\Delta\dot q_{\text{kick}}`, {
      min: 0.05,
      max: 10,
      log: true,
      unit: 'rad/s | m/s',
      hint: 'size of a Space kick (a random sign, on the passive joints, or on all when every joint is driven)',
    }),
    tuningFold(g, 'world', 'Contacts, pushes and loops'),
  );
  const tuning = card(
    'Controller tuning',
    tuningFold(g, 'task', 'Damping, singularities and inverse kinematics'),
  );
  host.append(task, secondary, world, tuning);
  const sections = el('div', 'goal-sections');
  host.append(sections);
  g.panel.mount(goalSections(d, g.snap, g.mech), sections);
  return host;
}

/** The goals below the tool, in priority order. */
function hierarchyCard(g: GoalsContext): HTMLElement {
  const { d, mech, client, settings, snap } = g;
  // joints that move the tool (a gripper's fingers do not)
  const dof = d.closures.length ? snap.v('DOF') : toolChainDof(d);
  const box = card('Task hierarchy');
  if (d.tool && !toolTargets(d)) {
    box.append(
      note('Goals below the tool need a tool target, and this mechanism cannot follow one (see above).'),
    );
    return box;
  }
  if (!d.tool) {
    box.append(note('Build a tool point to stack goals below it.'));
    return box;
  }
  box.append(
    note(
      dof > 2
        ? `Pursued by VMC, impedance and admittance control. Each level acts only in what the levels above leave free (${dof} DOF − 2 for the tool = ${dof - 2} to share): a lower goal can never disturb a higher one, and an unreachable one is pursued as far as possible.`
        : 'This mechanism has no redundancy for the tool task: only the tool position is controlled.',
    ),
  );
  if (dof <= 2) return box;
  const set = (key: Numeric, v: number) => setSetting(client, settings, key, v);
  const toggle = (key: 'orient' | 'elbow', on: boolean) => {
    set(key, on ? GOAL_W : 0);
    g.refresh();
  };
  const nb = mech.bodies.length;
  const here = (what: 'orient' | 'elbow') =>
    button(
      'here',
      () => {
        if (what === 'orient') set('orientTarget', snap.frame(nb + 1)[2]);
        else {
          const [x, y] = snap.frame(Math.round(snap.v('GOALS', 5)) + 1);
          set('elbowX', x);
          set('elbowY', y);
        }
        g.refresh();
      },
      'btn small',
      'use the current pose as the target',
    );
  const onOff = (key: 'orient' | 'elbow') =>
    segmented(
      [
        ['off', 'off'],
        ['on', 'on'],
      ],
      settings[key] > 0 ? 'on' : 'off',
      (v) => toggle(key, v === 'on'),
    );

  box.append(
    el('div', 'level', el('b', '', '1 · tool position'), el('span', 'muted small', 'the primary task')),
  );
  box.append(el('div', 'level', el('b', '', '2 · tool orientation φ_t'), onOff('orient')));
  if (settings.orient > 0) {
    box.append(
      settingSlider(client, settings, 'orient', String.raw`\omega_{\varphi}`, {
        min: 0.5,
        max: 20,
        log: true,
        unit: 'rad/s',
      }),
      slider({
        label: String.raw`\varphi^{*}`,
        tex: true,
        min: -180,
        max: 180,
        step: 1,
        unit: '°',
        value: settings.orientTarget * DEG,
        onInput: (v) => set('orientTarget', v / DEG),
      }),
      el('div', 'card-actions', here('orient')),
    );
  }
  box.append(el('div', 'level', el('b', '', '3 · elbow'), onOff('elbow')));
  if (settings.elbow > 0) {
    const options = elbowCandidates(mech).map((b) => [String(b), `joint ${b + 1}`] as [string, string]);
    const current = String(Math.round(snap.v('GOALS', 5)));
    box.append(
      row(
        'which joint',
        segmented(options, current, (v) => {
          client.flag('ElbowBody', Number(v));
          window.setTimeout(g.refresh, 50);
        }),
      ),
      settingSlider(client, settings, 'elbow', String.raw`\omega_{e}`, {
        min: 0.5,
        max: 20,
        log: true,
        unit: 'rad/s',
      }),
      slider({
        label: 'x_e^{*}',
        tex: true,
        min: -3,
        max: 3,
        step: 0.01,
        unit: 'm',
        value: settings.elbowX,
        onInput: (v) => set('elbowX', v),
      }),
      slider({
        label: 'y_e^{*}',
        tex: true,
        min: -3,
        max: 3,
        step: 0.01,
        unit: 'm',
        value: settings.elbowY,
        onInput: (v) => set('elbowY', v),
      }),
      el('div', 'card-actions', here('elbow')),
      note('Alt-drag in the viewport to place the elbow target.'),
    );
  }
  box.append(
    el(
      'div',
      'level',
      el('b', '', '4 · posture & manipulability'),
      el('span', 'muted small', 'whatever is left'),
    ),
    settingSlider(client, settings, 'posture', String.raw`\omega_{p}`, {
      min: 0,
      max: 20,
      unit: 'rad/s',
      hint: 'posture stiffness as a natural frequency',
    }),
    settingSlider(client, settings, 'manipulability', String.raw`k_{w}`, {
      min: 0,
      max: 20,
      hint: 'gain on the manipulability gradient',
    }),
  );
  return box;
}

function goalSections(d: Derivation, snap: Snapshot, mech: Mechanism): SectionSpec[] {
  const n = d.n;
  const ts = cachedTaskSpace(snap);
  const tool = d.tool;
  const redundant = tool !== null && !d.phi.length && toolChainDof(d) > 2;
  const out: SectionSpec[] = [
    {
      id: 'g-task',
      title: 'Targets and references',
      build(b) {
        b.text(
          'Controllers never chase a target directly: a critically damped filter turns every jump of the target into a smooth reference (position, velocity and acceleration), in joint space and in task space:',
        );
        b.math(
          String.raw`\ddot q_d = \omega_{\text{ref}}^2\,(q^{*} - q_d) - 2\omega_{\text{ref}}\,\dot q_d,\qquad \ddot x_d = \omega_{\text{ref}}^2\,(x^{*} - x_d) - 2\omega_{\text{ref}}\,\dot x_d`,
        );
        b.math(
          String.raw`q^{*} = ${b.live.vec(n, (i) => snap.v('Q_TARGET', i))},\quad q_d = ${b.live.vec(n, (i) => snap.v('Q_REF', i))}${tool ? String.raw`,\quad x^{*} = ${b.live.vec(2, (i) => snap.v('TARGET', i))},\quad x_d = ${b.live.vec(2, (i) => snap.v('X_REF', i))}` : ''}`,
        );
        if (tool) {
          b.text(
            String.raw`Joint-space controllers follow a tool target through inverse kinematics: damped least squares on the stacked task and loop constraints, a few steps per millisecond,`,
          );
          b.math(
            String.raw`\Delta q = S^{T}\big(SS^{T} + \mu\big)^{-1}\begin{bmatrix} -\varphi(q) \\ x^{*} - p_{st}(q) \end{bmatrix},\qquad S = \begin{bmatrix} A(q) \\ J(q) \end{bmatrix}`,
            {
              caption:
                'the minimum-norm step: for redundant arms the joint-space solution ignores the secondary goals',
            },
          );
          b.math(
            String.raw`\text{reachable: }${b.live.num(
              () => snap.v('REACHABLE'),
              (v) => (v ? 'yes' : 'no'),
            )}`,
          );
        }
      },
    },
  ];
  if (!tool) return out;
  out.push({
    id: 'g-null',
    title: 'Null-space projection',
    cite: 'Khatib 1987',
    build(b) {
      b.text(
        String.raw`The tool task $x = p_{st}(q)$ fixes 2 of the ${n - d.phi.length} degrees of freedom. A torque $\tau_0$ for a secondary goal must not accelerate the tool. The tool acceleration a torque produces is $J M^{-1}\tau$, so we need a projector $N_\perp$ with`,
      );
      b.math(String.raw`J\,M^{-1}N_{\perp}\,\tau_0 = 0\quad\text{for all }\tau_0`);
      b.text(
        String.raw`The task-space inertia $\Lambda = (JM^{-1}J^{T})^{-1}$ and the dynamically consistent generalised inverse $\bar J = M^{-1}J^{T}\Lambda$ give it:`,
      );
      b.math(
        String.raw`N_{\perp} = I - J^{T}\bar J^{T},\qquad J M^{-1}N_{\perp} = JM^{-1} - \underbrace{JM^{-1}J^{T}\Lambda}_{=I}\,JM^{-1} = 0`,
      );
      b.text(
        String.raw`The plain pseudo-inverse projector $I - J^{+}J$ is only kinematically consistent: it keeps the tool still under joint <i>velocities</i>, but its torques still accelerate the tool. The metric $M$ makes the difference.`,
      );
      if (!redundant) {
        b.text(
          '<span class="muted">This mechanism is not redundant for the tool task: N⊥ removes everything.</span>',
        );
      }
      b.math(String.raw`J = ${texMat(tool.Jp)} = ${b.live.mat(2, n, (i, j) => snap.m('JP', i, j))}`);
      b.math(
        String.raw`\Lambda = ${b.live.mat(2, 2, (i, j) => ts().lambda[i * 2 + j]!)}\,\mathrm{kg},\quad \bar J = ${b.live.mat(n, 2, (i, j) => ts().jbar[i * 2 + j]!)}`,
      );
      b.math(String.raw`N_{\perp} = ${b.live.mat(n, n, (i, j) => ts().proj[i * n + j]!)}`, {
        caption: `rank ${Math.max(0, n - d.phi.length - 2)} for an open chain: the directions left for secondary goals`,
      });
    },
  });
  out.push({
    id: 'g-hierarchy',
    title: 'A hierarchy of tasks',
    cite: 'Siciliano & Slotine 1991; Khatib 1987',
    build(b) {
      b.text(
        String.raw`A redundant arm can pursue several goals at once, ranked: the tool position first, then the tool orientation $\varphi_t$, then an elbow point, then a posture. With a 7-joint arm in space this is the classic split — the 6-D tool pose plus one "arm angle" that swings the elbow. In the plane the tool needs 2 coordinates, its orientation 1, an elbow point 2.`,
      );
      b.text(
        String.raw`Every task $k$ has a Jacobian $J_k$ and a desired acceleration from a critically damped spring at its own frequency:`,
      );
      b.math(
        String.raw`a_k = \omega_k^2\,(x_k^{*} - x_k) - 2\omega_k\,\dot x_k,\qquad \varphi_t = \textstyle\sum_{i\,\text{revolute}} q_i + c \;\Rightarrow\; J_\varphi = [1\ \cdots\ 1]`,
      );
      b.text(
        String.raw`Strict priority comes from recursion. Level $k$ only sees what levels $1 \dots k-1$ leave free, through the restricted Jacobian $\hat J_k = J_k P_{k-1}$ ($P_0 = I$), with the dynamically consistent inverse in the metric of $M$:`,
      );
      b.math(
        String.raw`\hat\Lambda_k = \big(\hat J_k M^{-1}\hat J_k^{T}\big)^{-1},\qquad \hat J_k^{\#} = M^{-1}\hat J_k^{T}\hat\Lambda_k,\qquad P_k = P_{k-1} - \hat J_k^{\#}\hat J_k`,
      );
      if (d.closures.length) {
        b.text(
          String.raw`With loop constraints, the hierarchy acts in the tangent space spanned by $Z$: use $H = Z(Z^T M Z)^{-1}Z^T$ in place of $M^{-1}$ and start from $P_0 = HM$. Each task correction then preserves the loop constraints; the initial acceleration accounts for their curvature.`,
        );
      }
      b.math(
        String.raw`\ddot q \mathrel{+}= \hat J_k^{\#}\,\big(a_k - \dot J_k\dot q - J_k\ddot q\big)\quad\text{(impedance)},\qquad \tau \mathrel{+}= \hat J_k^{T}\hat\Lambda_k\,a_k\quad\text{(VMC)}`,
      );
      b.text(
        String.raw`Why a lower level cannot disturb a higher one: its correction lies in the range of $M^{-1}\hat J_k^{T} = M^{-1}P_{k-1}^{T}J_k^{T} = P_{k-1}M^{-1}J_k^{T}$, and $J_i P_{k-1} = 0$ for every $i < k$. This holds for <i>any</i> $\hat\Lambda_k$, so the command can be damped without breaking the priorities: near a singularity, or when the levels above have used up a task's directions, each eigen-direction $\mu$ of $\hat J_k M^{-1}\hat J_k^{T}$ gets $\mu/(\mu^2+\delta^2)$ instead of $1/\mu$, with $\delta = 0$ while the task is well conditioned and growing smoothly as $\mu \to 0$ (adaptive damped least squares). The projector keeps the exact inverse but fades out, smoothly, the directions that are all but lost — a hard cut-off would switch them in and out of the null space as the arm hovers at the threshold. An unreachable goal then rests at the boundary of what is possible instead of shaking the arm.`,
      );
      b.text(
        String.raw`What is left after the last task, $P_K$, carries the posture and manipulability goal $\tau_0 = M(q^{*})\big(\omega_p^2(q^{*} - q) - 2\omega_p\dot q + k_w\nabla w\big)$ — and when every direction is used ($P_K = 0$), nothing.`,
      );
      // live: the levels, and what the running task controller reports
      const levels = [
        ['1', 'tool position', 2, true],
        ['2', 'tool orientation', 1, snap.v('GOALS') > 0],
        ['3', 'elbow', 2, snap.v('GOALS', 2) > 0],
      ] as const;
      const dof = d.phi.length ? n - d.phi.length : toolChainDof(d);
      let left = dof;
      const rows = levels.map(([k, name, dim, on]) => {
        const used = on ? Math.min(dim, Math.max(0, left)) : 0;
        left -= used;
        return String.raw`${k}\ &\text{${name}}\ &${on ? dim : '—'}\ &${on ? used : '—'}`;
      });
      b.math(
        String.raw`\begin{array}{llcc} & \text{task} & \text{needs} & \text{gets} \\ \hline ${rows.join(String.raw`\\`)} \\ 4 & \text{posture} & & ${Math.max(0, left)} \end{array}`,
        {
          caption: `degrees of freedom when the tab was opened (${dof} in all); reopen the tab after switching goals`,
        },
      );
      b.math(
        String.raw`\varphi^{*} - \varphi_t = ${b.live.num(() => {
          if (!(snap.v('GOALS') > 0)) return Number.NaN;
          const e = snap.v('GOALS', 1) - snap.frame(mech.bodies.length + 1)[2];
          return Math.atan2(Math.sin(e), Math.cos(e));
        })}\,\mathrm{rad},\qquad \|x_e^{*} - x_e\| = ${b.live.num(() => {
          if (!(snap.v('GOALS', 2) > 0)) return Number.NaN;
          const [x, y] = snap.frame(Math.round(snap.v('GOALS', 5)) + 1);
          return Math.hypot(snap.v('GOALS', 3) - x, snap.v('GOALS', 4) - y);
        })}\,\mathrm{m}`,
        { caption: '— means that goal is off' },
      );
      b.text(
        'The VMC, impedance and admittance pages report the leakage of the lower levels into the ones above (‖J_k M⁻¹ τ_lower‖, ‖J_k Δq̈‖): round-off.',
      );
      b.math(String.raw`w(q) = \sqrt{\det J(q)J(q)^{T}} = ${b.live.num(() => snap.v('MANIPULABILITY'))}`, {
        caption: 'Yoshikawa manipulability of the tool task: zero at singular poses',
      });
    },
  });
  return out;
}

export function modelTab(
  d: Derivation,
  params: number[],
  client: SimClient,
  settings: Settings,
  /** Highlight body i in the figure (−1: none). */
  highlight: (body: number) => void = () => undefined,
): HTMLElement {
  const host = el('div', 'editor');
  const sliders = new Map<number, Slider>();
  const set = (i: number, v: number) => {
    params[i] = v;
    client.send({ type: 'param', i, v });
  };
  const coeffs = d.params.map((p) => {
    const s = slider({
      label: p.tex,
      tex: true,
      ...physicalControl(p.kind, 'runtime'),
      value: params[p.index] ?? p.value,
      onInput: (v) => set(p.index, v),
    });
    sliders.set(p.index, s);
    return s;
  });
  // friction: all β to zero (a conservative mechanism), or back to the built values
  const betas = d.params.filter((p) => p.kind === 'beta');
  const friction = (on: boolean) => {
    for (const p of betas) {
      const v = on ? p.value : 0;
      set(p.index, v);
      sliders.get(p.index)?.set(v);
    }
  };
  const frictionOn = betas.some((p) => (params[p.index] ?? p.value) !== 0);
  // one group per body (hover: the body lights up in the figure), gravity on its own
  const groups: HTMLElement[] = [];
  const bodies = [...new Set(d.params.filter((p) => p.kind !== 'g').map((p) => p.body))];
  for (const b of bodies) {
    const grid = el('div', 'coeff-grid');
    d.params.forEach((p, k) => {
      if (p.kind !== 'g' && p.body === b) grid.append(coeffs[k]!);
    });
    const group = el('div', 'coeff-group', el('h4', '', `Body ${b + 1}`), grid);
    group.addEventListener('pointerenter', () => highlight(b));
    group.addEventListener('pointerleave', () => highlight(-1));
    groups.push(group);
  }
  d.params.forEach((p, k) => {
    if (p.kind === 'g') groups.push(el('div', 'coeff-group', el('h4', '', 'Gravity'), coeffs[k]!));
  });
  host.append(
    card(
      'Coefficients',
      note(
        'The equations were derived with symbolic coefficients, so these only change numbers — no re-derivation. Every slot (primary and shadows) shares them. (Uniform-rod links keep r and I as set here until you rebuild.)',
        'prose',
      ),
      ...groups,
    ),
    card(
      'What-if',
      note(
        'Joint friction β is part of N(q, q̇): with it a passive mechanism loses energy (see the energy budget in the Equations tab and the Energy plots). Without it, energy is conserved to integration accuracy.',
        'prose',
      ),
      row(
        'joint friction β',
        segmented(
          [
            ['on', 'as built'],
            ['off', 'none'],
          ],
          frictionOn ? 'on' : 'off',
          (v) => friction(v === 'on'),
        ),
      ),
      note(
        `Coriolis & centrifugal forces are ${settings.coriolis ? 'on' : 'off'} (toggle in the viewport). Switching them off is non-physical: energy is no longer conserved, and model-based controllers that still cancel C q̇ inject energy.`,
        'prose',
      ),
    ),
  );
  return host;
}
