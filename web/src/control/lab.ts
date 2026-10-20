/**
 * The lab of every controller page: which signals matter for it (its "Watch" plots) and guided experiments.
 * An experiment makes the page's controller primary, applies its setup (shadows, parameters, limits,
 * kicks, pushes), switches the plots to what matters and says what to notice.
 */
import { type Experiment, type Lab, type PageContext, range } from './page';

const slot = (key: string, what: string) => `slot:${key}:${what}`;
const tele = (key: string, name: string, i = 0) => `tele:${key}:${name}:${i}`;
const passive = (c: PageContext) => c.d.coords.flatMap((x, i) => (x.actuated ? [] : [i]));

function common(c: PageContext) {
  const q = range(c.n, (i) => `q:${i}`);
  const e = range(c.n, (i) => `e:${i}`);
  const u = range(c.nu, (a) => `u:${a}`);
  const tool = c.d.tool !== null;
  const redundant = tool && c.dof > 2 && !c.loops;
  const full = c.nu >= c.dof && c.nu > 0;
  const under = c.nu > 0 && c.nu < c.dof;
  return { q, e, u, tool, redundant, full, under };
}

type LabFn = (c: PageContext) => Lab;

const passiveLab: LabFn = (c) => {
  const { q } = common(c);
  return {
    watch: [
      { ids: q },
      { ids: ['energy:T', 'energy:V', 'energy:E'] },
      { ids: ['energy:P', 'energy:D', 'energy:dE'] },
    ],
    experiments: [
      {
        title: 'Where does the energy go?',
        goal: 'Without inputs, only friction (and pushes) can change the energy.',
        actions: [{ do: 'reset' }, { do: 'kick', v: 2 }],
        plots: [{ ids: ['energy:T', 'energy:V', 'energy:E'] }, { ids: ['energy:D', 'energy:dE'] }],
        notice:
          'T and V trade places all the time, yet E only drifts down — exactly by the friction power $\\dot q^T\\operatorname{diag}(\\beta)\\dot q$. The identity $\\dot E = \\dot q^T\\tau - \\dot q^T\\beta\\dot q$ rests on the skew-symmetry of $\\dot M - 2C$.',
      },
      {
        title: 'Switch Coriolis off',
        goal: 'What happens if the simulation forgets the C(q, q̇)q̇ term?',
        actions: [{ do: 'reset' }, { do: 'coriolis', on: false }, { do: 'kick', v: 2 }],
        plots: [{ ids: ['energy:E'] }, { ids: q }],
        notice:
          'With $C\\dot q$ missing the energy is no longer conserved: it wanders up and down although nothing does work. Coriolis and centrifugal forces do no work, but they are needed to keep the bookkeeping right. (Switch it back on with the Coriolis toggle.)',
      },
    ],
  };
};

const pdLab =
  (gravity: boolean): LabFn =>
  (c) => {
    const { q, e, full } = common(c);
    const me = gravity ? 'pd_gravity' : 'pd';
    const other = gravity ? 'pd' : 'pd_gravity';
    const experiments: Experiment[] = [
      {
        title: gravity ? 'Exact compensation vs. sag' : 'The gravity sag',
        goal: 'Run plain PD and PD + gravity side by side towards the same target.',
        actions: [{ do: 'shadow', key: other }, { do: 'reset' }],
        plots: [{ ids: [slot(me, 'err'), slot(other, 'err')] }, { ids: e }],
        notice:
          'Plain PD settles where its springs balance gravity, $K_p e = \\partial V/\\partial q$, so its error never reaches zero. The gravity feed-forward removes the only force pulling away from $q^*$: the error converges to zero for any positive gains.',
      },
      {
        title: 'Damping ratio',
        goal: 'Lower ζ and kick the mechanism.',
        actions: [{ do: 'param', key: 'zeta', value: 0.15 }, { do: 'reset' }, { do: 'kick', v: 2 }],
        plots: [{ ids: q }, { ids: e }],
        notice:
          'Each joint rings like a mass–spring with $\\zeta = 0.15$, but the joints also push each other through the off-diagonal terms of $M(q)$: the oscillations are not independent. Set ζ back to 1 for a critically damped response.',
      },
    ];
    if (!gravity && !full) {
      experiments.push({
        title: 'PD cannot balance',
        goal: 'Try plain PD on an underactuated mechanism.',
        actions: [{ do: 'shadow', key: 'lqr' }, { do: 'reset' }, { do: 'kick', v: 0.3 }],
        plots: [{ ids: [slot(me, 'err'), slot('lqr', 'err')] }],
        notice:
          'Independent joint servos ignore the passive joints entirely, so an inverted pendulum falls. LQR uses the coupling $M_{ua}$ to steer them.',
      });
    }
    return {
      watch: [{ ids: q }, { ids: e }, { ids: range(c.n, (i) => tele(me, 'kp', i)) }],
      experiments,
    };
  };

const computedTorqueLab: LabFn = (c) => {
  const { q, e, tool } = common(c);
  const experiments: Experiment[] = [
    {
      title: 'Same error dynamics everywhere',
      goal: 'Kick computed torque and PD + gravity: who recovers the same way in every pose?',
      actions: [{ do: 'shadow', key: 'pd_gravity' }, { do: 'reset' }, { do: 'kick', v: 3 }],
      plots: [{ ids: [slot('computed_torque', 'err'), slot('pd_gravity', 'err')] }, { ids: e }],
      notice:
        'Computed torque cancels $M$, $C$ and $N$: every joint error obeys $\\ddot e + K_v\\dot e + K_p e = 0$, critically damped and decoupled. PD + gravity also converges, but its transient depends on the pose and the joints disturb each other.',
    },
    {
      title: 'Saturation breaks cancellation',
      goal: 'Limit the actuators and kick hard.',
      actions: [{ do: 'setting', key: 'umax', value: 8 }, { do: 'reset' }, { do: 'kick', v: 5 }],
      plots: [{ ids: [slot('computed_torque', 'err')] }, { ids: [slot('computed_torque', 'effort')] }],
      notice:
        'When $|u| \\le u_{\\max}$ clips the computed torque, the dynamics are no longer cancelled and the linear error equation no longer holds: slow, distorted recovery. Raise $u_{\\max}$ again in the Goals tab.',
    },
  ];
  if (tool) {
    experiments.push({
      title: 'A push folds the light links',
      goal: 'Push the tool with computed torque and an impedance shadow.',
      actions: [
        { do: 'shadow', key: 'impedance' },
        { do: 'hold' },
        { do: 'push', dx: 0.6, dy: -0.4, ms: 3000 },
      ],
      plots: [{ ids: [slot('computed_torque', 'err'), slot('impedance', 'err')] }, { ids: ['fext:n'] }],
      notice:
        'Computed torque treats the push as a disturbance: $M(\\ddot e + K_v\\dot e + K_p e) = -J^T F_{\\text{ext}}$, a joint stiffness of $M\\omega_c^2$ that is tiny for light distal links. Impedance control yields like one clean 2-D spring of stiffness $K_d$.',
    });
  }
  return {
    watch: [{ ids: q }, { ids: e }, { ids: range(c.n, (i) => tele('computed_torque', 'qdd', i)) }],
    experiments,
  };
};

const passivityLab: LabFn = (c) => {
  const { e } = common(c);
  return {
    watch: [{ ids: [tele('passivity', 'lyap')] }, { ids: [tele('passivity', 'lyap_dot')] }, { ids: e }],
    experiments: [
      {
        title: 'Watch the Lyapunov function',
        goal: 'Kick the mechanism and follow V and V̇.',
        actions: [{ do: 'reset' }, { do: 'kick', v: 3 }],
        plots: [{ ids: [tele('passivity', 'lyap')] }, { ids: [tele('passivity', 'lyap_dot')] }],
        notice:
          'The kick injects energy, so $V = \\tfrac12 s^T M s + e^T\\Lambda K_D e$ jumps up — and from then on only decreases: $\\dot V = -\\dot e^T K_D\\dot e - e^T\\Lambda K_D\\Lambda e \\le 0$. The proof uses $\\dot M - 2C$ skew-symmetric instead of cancelling the dynamics.',
      },
      {
        title: 'Passivity vs. computed torque',
        goal: 'Same kick, two model-based trackers.',
        actions: [{ do: 'shadow', key: 'computed_torque' }, { do: 'reset' }, { do: 'kick', v: 3 }],
        plots: [
          { ids: [slot('passivity', 'err'), slot('computed_torque', 'err')] },
          { ids: [slot('passivity', 'effort'), slot('computed_torque', 'effort')] },
        ],
        notice:
          'Both converge globally with an exact model. Passivity-based control keeps the natural inertia ($M\\dot s + Cs + K_D s = 0$), so it typically spends less effort right after the kick; computed torque enforces the same linear transient in every pose.',
      },
    ],
  };
};

const vmcLab: LabFn = (c) => {
  const { q, tool, redundant } = common(c);
  if (!tool) return { watch: [{ ids: q }], experiments: [] };
  const experiments: Experiment[] = [
    {
      title: 'Energy of the closed loop',
      goal: 'Push the tool, let go, and follow V_L.',
      actions: [{ do: 'hold' }, { do: 'push', dx: 0.5, dy: 0.3, ms: 2000 }],
      plots: [{ ids: [tele('vmc', 'lyap')] }, { ids: [tele('vmc', 'lyap_dot'), 'fext:n'] }],
      notice:
        'While you push, the push does work and $V_L = T + \\tfrac12 e^T K e$ may grow. After release $\\dot V_L = -\\dot x^T D\\dot x - \\dot q^T\\beta\\dot q \\le 0$: the virtual spring–damper drains it. Nothing is cancelled, so the law stays passive.',
    },
    {
      title: 'Stiff spring, same damping ratio',
      goal: 'Raise K and push again.',
      actions: [
        { do: 'hold' },
        { do: 'param', key: 'k', value: 1500 },
        { do: 'push', dx: 0.5, dy: 0.3, ms: 2000 },
      ],
      plots: [{ ids: ['taskerr'] }, { ids: [tele('vmc', 'force', 0), tele('vmc', 'force', 1)] }],
      notice:
        'The static deflection shrinks like $F/K$; the damping $D = 2\\zeta\\sqrt{K\\bar\\lambda(\\Lambda)}$ grows with it, so the response stays well damped.',
    },
  ];
  if (redundant) {
    experiments.push({
      title: 'Posture in the null space',
      goal: 'Turn the posture goal up: the joints move, the tool stays.',
      actions: [{ do: 'setting', key: 'posture', value: 12 }, { do: 'hold' }],
      plots: [{ ids: q }, { ids: [tele('vmc', 'leak'), 'taskerr'] }],
      notice:
        'The posture torque $\\tau_0$ is projected with $N_\\perp = I - J^T\\bar J^T$. The arm folds towards $q^*$ while the tool keeps its target, and the leakage $\\|JM^{-1}N_\\perp\\tau_0\\|$ stays at round-off: dynamically consistent projection.',
    });
  }
  return {
    watch: [{ ids: ['taskerr'] }, { ids: [tele('vmc', 'lyap'), tele('vmc', 'lyap_dot')] }, { ids: q }],
    experiments,
  };
};

const impedanceLab: LabFn = (c) => {
  const { tool } = common(c);
  if (!tool) return { watch: [], experiments: [] };
  return {
    watch: [
      { ids: [tele('impedance', 'e', 0), tele('impedance', 'e', 1)] },
      { ids: ['fext:x', 'fext:y'] },
      { ids: [tele('impedance', 'leak')] },
    ],
    experiments: [
      {
        title: 'The spring law',
        goal: 'Push and compare the deflection with F_ext / K_d.',
        actions: [{ do: 'hold' }, { do: 'push', dx: 0.4, dy: 0, ms: 4000 }],
        plots: [
          { ids: [tele('impedance', 'e', 0), tele('impedance', 'e', 1)] },
          { ids: ['fext:x', 'fext:y'] },
        ],
        notice:
          'At rest $K_d e = F_{\\text{ext}}$ in every pose: with $K_d$ = 300 N/m, 30 N give 10 cm. The joint-space stiffness needed for that changes with the pose; the controller computes it.',
      },
      {
        title: 'Light virtual mass',
        goal: 'Lower Λ_d to 0.1 kg and push.',
        actions: [
          { do: 'hold' },
          { do: 'param', key: 'mass', value: 0.1 },
          { do: 'push', dx: 0.4, dy: 0, ms: 2000 },
        ],
        plots: [{ ids: ['taskerr'] }, { ids: [slot('impedance', 'effort')] }],
        notice:
          'A light target mass makes the tool feel nimble, but the controller has to fight the real task inertia Λ(q): larger, faster torques. With a real force sensor this amplifies noise.',
      },
      {
        title: 'Impedance vs. admittance',
        goal: 'Same push, the dual controller as a shadow.',
        actions: [
          { do: 'hold' },
          { do: 'shadow', key: 'admittance' },
          { do: 'push', dx: 0.4, dy: 0, ms: 4000 },
        ],
        plots: [{ ids: [slot('impedance', 'err'), slot('admittance', 'err')] }, { ids: ['fext:n'] }],
        notice:
          'Admittance deflects a bit more: its inner position loop is a second spring in series with the virtual one, $x - x_d = (K_d^{-1} + (\\Lambda\\omega_{\\text{in}}^2)^{-1})F$.',
      },
    ],
  };
};

const admittanceLab: LabFn = (c) => {
  const { tool } = common(c);
  if (!tool) return { watch: [], experiments: [] };
  return {
    watch: [
      { ids: [tele('admittance', 'xv', 0), tele('admittance', 'x', 0)] },
      { ids: ['fext:x', 'fext:y'] },
    ],
    experiments: [
      {
        title: 'Two springs in series',
        goal: 'Push and watch the real tool lag behind the virtual one.',
        actions: [{ do: 'hold' }, { do: 'push', dx: 0.4, dy: 0, ms: 4000 }],
        plots: [
          { ids: [tele('admittance', 'xv', 0), tele('admittance', 'x', 0), 'xref:x'] },
          { ids: ['fext:x'] },
        ],
        notice:
          'The virtual tool $x_v$ obeys $K_d(x_v - x_d) = F$; the real tool trails it by $F/(\\Lambda\\omega_{\\text{in}}^2)$ because the inner loop does not cancel the push.',
      },
      {
        title: 'A slow inner loop',
        goal: 'Lower ω_in and push.',
        actions: [
          { do: 'hold' },
          { do: 'param', key: 'wn_in', value: 4 },
          { do: 'push', dx: 0.4, dy: 0, ms: 3000 },
        ],
        plots: [{ ids: [tele('admittance', 'xv', 0), tele('admittance', 'x', 0)] }],
        notice:
          'The cascade needs the inner loop to be much faster than the virtual dynamics; with ω_in close to $\\sqrt{K_d/\\Lambda_d}$ the tool lags and overshoots.',
      },
    ],
  };
};

const lqrLab: LabFn = (c) => {
  const { q, u, under } = common(c);
  const experiments: Experiment[] = [
    {
      title: 'Region of attraction',
      goal: 'Kick harder and harder (press Space): when does the linear controller give up?',
      actions: [{ do: 'reset' }, { do: 'kick', v: under ? 0.6 : 2 }],
      plots: [{ ids: q }, { ids: [tele('lqr', 'cost')] }],
      notice:
        'LQR is optimal for the linearisation only. Small kicks die out exponentially; past some size the nonlinear terms dominate and the state leaves the region of attraction — for an underactuated pendulum it falls.',
    },
    {
      title: 'Cheap control',
      goal: 'Make inputs cheap (R → 0.01) and kick.',
      actions: [{ do: 'param', key: 'r', value: 0.01 }, { do: 'reset' }, { do: 'kick', v: under ? 0.3 : 1 }],
      plots: [{ ids: q }, { ids: u }],
      notice:
        'Cheap control means high gains: faster recovery, much larger inputs. The gain $K$ in the derivation changes with $R$ (the Riccati equation is re-solved).',
    },
  ];
  if (under) {
    // verified on the cart-pole: at u_max = 4 N a 0.5 rad/s kick topples the clipped LQR, MPC recovers
    experiments.push({
      title: 'LQR clips, MPC plans',
      goal: 'Tight actuator limits, MPC as a shadow, same kick.',
      actions: [
        { do: 'shadow', key: 'mpc' },
        { do: 'setting', key: 'umax', value: 4 },
        { do: 'reset' },
        { do: 'kick', v: 0.5 },
      ],
      plots: [
        { ids: [slot('lqr', 'err'), slot('mpc', 'err')] },
        { ids: [slot('lqr', 'effort'), slot('mpc', 'effort')] },
      ],
      notice:
        'LQR computes $u = u^* - Kx$ and the actuator clips it; MPC knows the limit and plans a sequence that respects it. Here the clipped LQR loses the pendulum while MPC, planning within the bound, catches it. (Kick harder and both fail: no controller beats physics with 4 N.)',
    });
  }
  return { watch: [{ ids: q }, { ids: u }, { ids: [tele('lqr', 'cost')] }], experiments };
};

const mpcLab: LabFn = (c) => {
  const { q, u, under } = common(c);
  if (!under) {
    return {
      watch: [{ ids: q }, { ids: u }, { ids: [tele('mpc', 'active'), tele('mpc', 'iterations')] }],
      experiments: [
        {
          title: 'Active constraints',
          goal: 'Tight limits and a kick: watch the bounds become active.',
          actions: [{ do: 'setting', key: 'umax', value: 6 }, { do: 'reset' }, { do: 'kick', v: 1.5 }],
          plots: [{ ids: u }, { ids: [tele('mpc', 'active'), tele('mpc', 'iterations')] }],
          notice:
            'While the plan pushes against $u_{\\max}$, several of the $N$ planned inputs sit on the bound (active constraints) and FISTA needs more iterations. Once the state is small, no bound is active and MPC returns the LQR input.',
        },
      ],
    };
  }
  return {
    watch: [{ ids: q }, { ids: u }, { ids: [tele('mpc', 'active'), tele('mpc', 'iterations')] }],
    experiments: [
      {
        title: 'Active constraints',
        goal: 'Tight limits and a kick: watch the bounds become active.',
        actions: [{ do: 'setting', key: 'umax', value: 4 }, { do: 'reset' }, { do: 'kick', v: 0.5 }],
        plots: [{ ids: u }, { ids: [tele('mpc', 'active'), tele('mpc', 'iterations')] }],
        notice:
          'While the plan pushes against $u_{\\max}$, several of the $N$ planned inputs sit on the bound (active constraints) and FISTA needs more iterations. Once the state is small, no bound is active and MPC returns the LQR input.',
      },
      {
        title: 'A coarse model',
        goal: 'Predict with Δt = 0.05 s over 3 s, against LQR at the 1 kHz rate.',
        actions: [
          { do: 'param', key: 'horizon', value: 60 },
          { do: 'param', key: 'step', value: 0.05 },
          { do: 'shadow', key: 'lqr' },
          { do: 'setting', key: 'umax', value: 6 },
          { do: 'reset' },
          { do: 'kick', v: 0.8 },
        ],
        plots: [
          { ids: [slot('mpc', 'err'), slot('lqr', 'err')] },
          { ids: [slot('mpc', 'effort'), slot('lqr', 'effort')] },
        ],
        notice:
          'A longer horizon is not automatically better: each re-plan holds the input for 50 ms and predicts with a model discretised at that step, while the pendulum moves on. Here the clipped 1 kHz LQR recovers and the coarse MPC does not. Prediction accuracy and update rate matter as much as foresight.',
      },
    ],
  };
};

const swingLab: LabFn = (c) => {
  const { q } = common(c);
  const p = passive(c);
  return {
    watch: [{ ids: [tele('swing_up', 'energy')] }, { ids: q }],
    experiments: [
      {
        title: 'Pump, then catch',
        goal: 'Start hanging down and watch the energy reach the target level.',
        actions: [{ do: 'hang' }],
        plots: [{ ids: [tele('swing_up', 'energy')] }, { ids: p.length ? p.map((i) => `q:${i}`) : q }],
        notice:
          'The pumping law makes $\\dot E$ have the sign of $-E$: each swing adds energy until $E \\to 0$, the energy of the upright pose. The motion then passes near $q^*$ and the LQR catches it (the phase switches to “LQR catch”).',
      },
      {
        title: 'Gentle pumping',
        goal: 'Lower the energy gain k_E: more swings, softer inputs.',
        actions: [{ do: 'param', key: 'ke', value: 4 }, { do: 'hang' }],
        plots: [{ ids: [tele('swing_up', 'energy')] }, { ids: [slot('swing_up', 'effort')] }],
        notice:
          'Swing-up is a trade-off between time and effort (and room on the rail: watch the end stops).',
      },
    ],
  };
};

export const LABS: Record<string, LabFn> = {
  passive: passiveLab,
  pd: pdLab(false),
  pd_gravity: pdLab(true),
  computed_torque: computedTorqueLab,
  passivity: passivityLab,
  vmc: vmcLab,
  impedance: impedanceLab,
  admittance: admittanceLab,
  lqr: lqrLab,
  mpc: mpcLab,
  swing_up: swingLab,
};
