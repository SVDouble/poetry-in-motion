/**
 * The Goals and Model tabs: what the controllers aim for
 * (joint or tool targets, secondary goals in the null space of the tool task), how the world pushes back,
 * and the symbolic coefficients of the model.
 */
import { cachedTaskSpace } from '../control/nullspace';
import type { EquationPanel, SectionSpec } from '../eq/panel';
import { texMat } from '../eq/tex';
import type { Derivation } from '../model/derivation';
import type { Mechanism } from '../model/description';
import type { SimClient } from '../sim/client';
import type { Snapshot } from '../sim/layout';
import { card, DEG, el, note, row, type Slider, type SliderOpts, segmented, slider } from '../ui/widgets';
import { type Numeric, SETTING_FLAGS, type Settings, toolTargets } from './settings';

type Range = Pick<SliderOpts, 'min' | 'max'> & Partial<Pick<SliderOpts, 'log' | 'unit' | 'hint'>>;

function settingSlider(client: SimClient, s: Settings, key: Numeric, label: string, range: Range) {
  const flag = SETTING_FLAGS.find(([k]) => k === key)?.[1];
  return slider({
    label,
    tex: true,
    value: s[key],
    ...range,
    onInput: (v) => {
      s[key] = v;
      if (flag) client.flag(flag, v);
    },
  });
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
      value: b.qTarget * (angular ? DEG : 1),
      unit: angular ? '°' : 'm',
      onInput: (v) => {
        b.qTarget = v / (angular ? DEG : 1);
        g.onJointTarget();
        client.send({ type: 'target', i, v: b.qTarget });
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
        (v) => g.setUseTool(v === 'tool'),
      ),
      note(
        'Drag in the viewport to move the tool target; Shift-drag to push the tool with a spring (at most 30 N, like a hand).',
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
    ...targets,
    settingSlider(client, settings, 'refWn', String.raw`\omega_{\text{ref}}`, {
      min: 0.5,
      max: 20,
      log: true,
      unit: 'rad/s',
    }),
  );

  const secondary = card(
    'Secondary goals (null space)',
    note(
      d.tool && d.n - d.phi.length > 2 && !d.closures.length
        ? 'Pursued by VMC and impedance control without moving the tool. Posture pulls towards the joint targets q*; manipulability climbs ∇w away from singular poses.'
        : 'This mechanism has no redundancy for the tool task (or has loops): the goals below have no effect.',
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
    }),
  );
  host.append(task, secondary, world);
  const sections = el('div', 'goal-sections');
  host.append(sections);
  g.panel.mount(goalSections(d, g.snap), sections);
  return host;
}

function goalSections(d: Derivation, snap: Snapshot): SectionSpec[] {
  const n = d.n;
  const ts = cachedTaskSpace(snap);
  const tool = d.tool;
  const redundant = tool !== null && n - d.phi.length > 2;
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
    id: 'g-secondary',
    title: 'Secondary objectives',
    build(b) {
      b.text('Both goals are expressed as a joint torque τ₀ and then projected:');
      b.math(
        String.raw`\tau_0 = \operatorname{diag}M(q^{*})\Big(\omega_p^2\,(q^{*} - q) - 2\omega_p\,\dot q\Big) + k_w \operatorname{diag}M(q^{*})\,\nabla_q w(q)`,
      );
      b.math(String.raw`w(q) = \sqrt{\det J(q)J(q)^{T}} = ${b.live.num(() => snap.v('MANIPULABILITY'))}`, {
        caption: 'Yoshikawa manipulability: zero at singular poses; its gradient is taken numerically',
      });
      b.text(
        'VMC adds N⊥τ₀ to its torque. Impedance control uses the same idea at the acceleration level: among all joint accelerations that realise the commanded tool acceleration it picks the one closest, in the metric of M, to M⁻¹τ₀ — which is equivalent. Both pages report the <i>leakage</i> ‖J M⁻¹ N⊥ τ₀‖, the tool acceleration caused by the secondary goals: it stays at round-off level.',
      );
    },
  });
  return out;
}

export function modelTab(
  d: Derivation,
  params: number[],
  client: SimClient,
  settings: Settings,
): HTMLElement {
  const host = el('div', 'editor');
  const max = { m: 10, g: 20, beta: 2, l: 2, r: 2, I: 2 } as const;
  const sliders = new Map<number, Slider>();
  const set = (i: number, v: number) => {
    params[i] = v;
    client.send({ type: 'param', i, v });
  };
  const coeffs = d.params.map((p) => {
    const s = slider({
      label: p.tex,
      tex: true,
      min: p.kind === 'r' ? -1 : 0,
      max: max[p.kind],
      step: 0.001,
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
  host.append(
    card(
      'Coefficients',
      note(
        'The equations were derived with symbolic coefficients, so these only change numbers — no re-derivation. Every slot (primary and shadows) shares them. (Uniform-rod links keep r and I as set here until you rebuild.)',
        'prose',
      ),
      ...coeffs,
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
