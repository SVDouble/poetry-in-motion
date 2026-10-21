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
  // a pendulum on an actuated rail: where the LQR/MPC comparisons were verified
  const cartLike = c.dof === 2 && c.nu === 1 && c.d.coords.some((x) => x.actuated && x.kind === 'prismatic');
  // the inputs that hold q* (u*): zero when gravity does not load the target pose
  const uff = Math.max(0, ...Array.from({ length: c.nu }, (_, a) => Math.abs(c.snap.v('U_FF', a))));
  // q* is an equilibrium the inputs can hold (else no linear controller can balance there)
  const balanced = c.snap.v('EQ_RESIDUAL') < 1e-6;
  return { q, e, u, tool, redundant, full, under, uff, cartLike, balanced };
}

type LabFn = (c: PageContext) => Lab;

/** Task-hierarchy experiments for a stacked task controller (open chains with a tool). */
function hierarchy(c: PageContext, key: string): Experiment[] {
  if (!c.d.tool || c.loops) return [];
  const leaks = [tele(key, 'leak'), tele(key, 'leak_sec')];
  const out: Experiment[] = [
    {
      title: 'Reaching beyond the workspace',
      goal: 'Aim at a point twice as far from the base as the tool: out of reach.',
      actions: [{ do: 'hold' }, { do: 'reach', factor: 2 }],
      plots: [{ ids: range(c.n, (i) => `q:${i}`) }, { ids: ['taskerr', 'w'] }],
      notice:
        'The arm stretches towards the target and stays there, at rest. A spring towards a point it can never reach would pull ever harder against the stretched pose, where $J$ loses the radial direction ($w \\to 0$, $\\Lambda$ unbounded) — so the reference stops at the nearest reachable point, found by inverse kinematics (Levenberg–Marquardt, which ends there instead of flipping the elbow), a hair inside the boundary. What is left near the singularity is handled by adaptive damped least squares (exact while the task is well conditioned, bounded as it loses a direction) and by damping the motion the task lets go of. The tool error stays: no controller can reach further than the arm.',
    },
  ];
  if (c.dof >= 5) {
    out.push({
      title: 'A stack of tasks',
      goal: 'Hold the tool, turn it by 0.3 rad and move the elbow: three goals, one arm.',
      actions: [{ do: 'hold' }, { do: 'stack', w: 6, dphi: 0.3, delbow: [0.05, 0.08] }],
      plots: [{ ids: ['taskerr', 'orienterr', 'elbowerr'] }, { ids: leaks }],
      notice:
        'Priority 1 (the tool) never moves; orientation and elbow converge in what it leaves free, and posture takes the rest. Each level is resolved through $P_{k-1}$, the dynamically consistent null space of the levels above: their leakage stays at round-off. With 7 joints: 2 for the tool, 1 for its orientation, 2 for the elbow — 2 left for posture.',
    });
  }
  if (c.dof === 3) {
    out.push({
      title: 'Priority under conflict',
      goal: 'Tool position and orientation use all three joints: the elbow goal has no room.',
      actions: [{ do: 'hold' }, { do: 'stack', w: 6, dphi: -0.1, delbow: [0.3, -0.3] }],
      plots: [{ ids: ['taskerr', 'orienterr'] }, { ids: ['elbowerr'] }],
      notice:
        'The elbow level only sees $\\hat J_3 = J_3 P_2$, and $P_2 = 0$ once the tool and its orientation have used every joint: its command vanishes. The higher goals are met exactly; the elbow error simply stays. Order the goals the other way and the tool orientation would give way instead.',
    });
  }
  return out;
}

const passiveLab: LabFn = (c) => {
  const { q } = common(c);
  const friction = c.d.params.some((p) => p.kind === 'beta' && p.value > 0);
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
        plots: [
          { ids: ['energy:T', 'energy:V', 'energy:E'] },
          { ids: ['energy:E', 'work:fric', 'energy:Ebal'] },
          { ids: ['budget'] },
        ],
        notice: friction
          ? 'T and V trade places all the time, yet E only drifts down — by exactly what friction takes, $\\int \\dot q^T\\operatorname{diag}(\\beta)\\dot q\\,dt$: E plus that loss stays at $E_0$ and the budget residual sits at round-off. The identity $\\dot E = \\dot q^T\\tau - \\dot q^T\\beta\\dot q$ rests on the skew-symmetry of $\\dot M - 2C$. (Model tab → friction “none” makes E constant.)'
          : 'T and V trade places all the time, and E stays constant: this mechanism has no friction ($\\beta = 0$), and gravity and the Coriolis forces do no net work. The budget residual shows the integration error — round-off level. The identity $\\dot E = \\dot q^T\\tau - \\dot q^T\\beta\\dot q$ rests on the skew-symmetry of $\\dot M - 2C$.',
      },
      ...(c.d.C.some((row) => row.some((x) => x !== '0'))
        ? ([
            {
              title: 'Switch Coriolis off',
              goal: 'What happens if the simulation forgets the C(q, q̇)q̇ term?',
              actions: [{ do: 'reset' }, { do: 'coriolis', on: false }, { do: 'kick', v: 2 }],
              plots: [{ ids: ['energy:E', 'energy:Ebal'] }, { ids: ['budget'] }],
              notice:
                'With $C\\dot q$ missing the energy is no longer accounted for: the budget residual wanders although nothing does work. Coriolis and centrifugal forces do no work, but they are needed to keep the bookkeeping right. (Switch it back on with the Coriolis toggle.)',
            },
          ] as Experiment[])
        : []),
    ],
  };
};

const pdLab =
  (gravity: boolean): LabFn =>
  (c) => {
    const { q, e, full, uff, balanced } = common(c);
    const me = gravity ? 'pd_gravity' : 'pd';
    const other = gravity ? 'pd' : 'pd_gravity';
    const experiments: Experiment[] = [];
    // the sag needs gravity loading the target pose, and a full actuator set to settle at all
    if (full && uff > 0.5) {
      experiments.push({
        title: gravity ? 'Exact compensation vs. sag' : 'The gravity sag',
        goal: 'Run plain PD and PD + gravity side by side towards the same target.',
        actions: [{ do: 'shadow', key: other }, { do: 'reset' }],
        plots: [{ ids: [slot(me, 'err'), slot(other, 'err')] }, { ids: e }],
        notice:
          'Plain PD settles where its springs balance gravity, $K_p e = \\partial V/\\partial q$, so its error never reaches zero. The gravity feed-forward removes the only force pulling away from $q^*$: the error converges to zero for any positive gains.',
      });
    }
    if (!c.loops) {
      experiments.push({
        title: 'Damping ratio',
        goal: 'Lower ζ and kick the mechanism.',
        actions: [{ do: 'param', key: 'zeta', value: 0.15 }, { do: 'reset' }, { do: 'kick', v: 2 }],
        plots: [{ ids: q }, { ids: e }],
        notice:
          'Near $q^*$ every mode rings with $\\zeta = 0.15$ — the gains are shaped by $M(q^*)$, so no direction is damped differently. Far from $q^*$, $M(q)$ differs from $M(q^*)$ and the response is no longer the same in every direction. Set ζ back to 1 for a critically damped response.',
      });
    }
    if (!gravity && !full && balanced) {
      experiments.push({
        title: 'PD cannot balance',
        goal: 'Try plain PD on an underactuated mechanism.',
        actions: [{ do: 'shadow', key: 'lqr' }, { do: 'atTarget' }, { do: 'kick', v: 0.05 }],
        plots: [{ ids: [slot(me, 'err'), slot('lqr', 'err')] }],
        notice:
          'Independent joint servos ignore the passive joints entirely, so an inverted pendulum falls. LQR uses the coupling $M_{ua}$ to steer them.',
      });
    }
    return {
      watch: [{ ids: q }, { ids: e }, { ids: range(c.nu, (a) => `u:${a}`) }],
      experiments,
    };
  };

const computedTorqueLab: LabFn = (c) => {
  const { q, e, tool } = common(c);
  const experiments: Experiment[] = [];
  experiments.push({
    title: 'Same error dynamics everywhere',
    goal: 'Kick computed torque and PD + gravity: who recovers the same way in every pose?',
    actions: [{ do: 'shadow', key: 'pd_gravity' }, { do: 'reset' }, { do: 'kick', v: 3 }],
    plots: [{ ids: [slot('computed_torque', 'err'), slot('pd_gravity', 'err')] }, { ids: e }],
    notice:
      'Computed torque cancels $M$, $C$ and $N$: every joint error obeys $\\ddot e + K_v\\dot e + K_p e = 0$, critically damped and decoupled. PD + gravity also converges, but its transient depends on the pose and the joints disturb each other.',
  });
  if (!c.loops) {
    experiments.push({
      title: 'Saturation breaks cancellation',
      goal: 'Limit the actuators and kick hard.',
      actions: [{ do: 'setting', key: 'umax', value: 8 }, { do: 'reset' }, { do: 'kick', v: 5 }],
      plots: [{ ids: [slot('computed_torque', 'err')] }, { ids: [slot('computed_torque', 'effort')] }],
      notice:
        'When $|u| \\le u_{\\max}$ clips the computed torque, the dynamics are no longer cancelled and the linear error equation no longer holds: slow, distorted recovery. Raise $u_{\\max}$ again in the Goals tab.',
    });
  }
  if (tool && !c.loops) {
    experiments.push({
      title: 'Joint stiffness vs. task stiffness',
      goal: 'Push the tool with computed torque and an impedance shadow.',
      actions: [{ do: 'shadow', key: 'impedance' }, { do: 'hold' }, { do: 'push', inward: 0.4, ms: 3000 }],
      plots: [{ ids: [slot('computed_torque', 'err'), slot('impedance', 'err')] }, { ids: ['fext:n'] }],
      notice:
        'Computed torque treats the push as a disturbance: $M(\\ddot e + K_v\\dot e + K_p e) = -J^T F_{\\text{ext}}$, so it resists with the joint stiffness $M(q)\\,\\omega_c^2$ — set by the inertia, different in every pose and direction (light distal links give way easily). Impedance control renders one designed 2-D spring: it deflects by exactly $F/K_d$ in every pose. Which one gives more depends on the mechanism; only one of them is chosen.',
    });
  }
  return {
    watch: [{ ids: q }, { ids: e }, { ids: range(c.n, (i) => tele('computed_torque', 'qdd', i)) }],
    experiments,
  };
};

const passivityLab: LabFn = (c) => {
  const { e } = common(c);
  const experiments: Experiment[] = [];
  // with loops the argument needs the reduced (independent) coordinates: V of the full q is not monotone
  if (!c.loops) {
    experiments.push({
      title: 'Watch the Lyapunov function',
      goal: 'Kick the mechanism and follow V and V̇.',
      // the proof assumes the torques are delivered: no actuator limit for this one
      actions: [{ do: 'setting', key: 'umax', value: 1e4 }, { do: 'reset' }, { do: 'kick', v: 3 }],
      plots: [{ ids: [tele('passivity', 'lyap')] }, { ids: [tele('passivity', 'lyap_dot')] }],
      notice:
        'The kick injects energy, so $V = \\tfrac12 s^T M s + e^T\\Lambda K_D e$ jumps up — and from then on only decreases: $\\dot V = -\\dot e^T K_D\\dot e - e^T\\Lambda K_D\\Lambda e \\le 0$. The proof uses $\\dot M - 2C$ skew-symmetric instead of cancelling the dynamics.',
    });
  }
  experiments.push({
    title: 'Passivity vs. computed torque',
    goal: 'Same kick, two model-based trackers.',
    actions: [{ do: 'shadow', key: 'computed_torque' }, { do: 'reset' }, { do: 'kick', v: 3 }],
    plots: [
      { ids: [slot('passivity', 'err'), slot('computed_torque', 'err')] },
      { ids: [slot('passivity', 'effort'), slot('computed_torque', 'effort')] },
    ],
    notice:
      'Both converge globally with an exact model. Passivity-based control keeps the natural inertia ($M\\dot s + Cs + K_D s = 0$), so it typically spends less effort right after the kick; computed torque enforces the same linear transient in every pose.',
  });
  return {
    watch: [{ ids: [tele('passivity', 'lyap')] }, { ids: [tele('passivity', 'lyap_dot')] }, { ids: e }],
    experiments,
  };
};

const vmcLab: LabFn = (c) => {
  const { q, tool, redundant } = common(c);
  if (!tool) return { watch: [{ ids: q }], experiments: [] };
  const experiments: Experiment[] = [
    {
      title: 'Energy of the closed loop',
      goal: 'Push the tool, let go, and follow V_L.',
      // the Lyapunov argument is for the task alone: no secondary goal feeding the null space
      actions: [
        { do: 'setting', key: 'posture', value: 0 },
        { do: 'hold' },
        { do: 'push', inward: 0.4, ms: 2000 },
      ],
      plots: [{ ids: [tele('vmc', 'lyap')] }, { ids: [tele('vmc', 'lyap_dot'), 'fext:n'] }],
      notice:
        'While you push, the push does work and $V_L = T + \\tfrac12 e^T K e$ may grow. After release $\\dot V_L = -\\dot x^T D\\dot x - \\dot q^T\\beta\\dot q \\le 0$: the virtual spring–damper drains it. Nothing is cancelled, so the law stays passive.',
    },
  ];
  // the static spring law F = K Δx needs a tool free to move in the plane (no loop constraining it)
  if (!c.loops) {
    experiments.push({
      title: 'Stiff spring, same damping ratio',
      goal: 'Raise K and push again.',
      actions: [
        { do: 'hold' },
        { do: 'param', key: 'k', value: 1500 },
        { do: 'push', inward: 0.4, ms: 2000 },
      ],
      plots: [{ ids: ['taskerr'] }, { ids: [tele('vmc', 'force', 0), tele('vmc', 'force', 1)] }],
      notice:
        'The static deflection shrinks like $F/K$; the damping $D = 2\\zeta\\sqrt{K\\bar\\lambda(\\Lambda)}$ grows with it, so the response stays well damped.',
    });
  }
  experiments.push(...hierarchy(c, 'vmc'));
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
      ...hierarchy(c, 'impedance'),
      // K_d e = F holds for a tool free to move in the plane: not where a loop constrains it
      ...(c.loops
        ? []
        : ([
            {
              title: 'The spring law',
              goal: 'Push and compare the deflection with F_ext / K_d.',
              actions: [{ do: 'hold' }, { do: 'push', inward: 0.4, ms: 4000 }],
              plots: [
                { ids: [tele('impedance', 'e', 0), tele('impedance', 'e', 1)] },
                { ids: ['fext:x', 'fext:y'] },
              ],
              notice:
                'At rest $K_d e = F_{\\text{ext}}$ in every pose: with $K_d$ = 300 N/m, 30 N give 10 cm. The joint-space stiffness needed for that changes with the pose; the controller computes it.',
            },
          ] satisfies Experiment[])),
      {
        title: 'Light virtual mass',
        goal: 'Lower Λ_d to 0.1 kg and push.',
        actions: [
          { do: 'hold' },
          { do: 'param', key: 'mass', value: 0.1 },
          { do: 'push', inward: 0.4, ms: 2000 },
        ],
        plots: [{ ids: ['taskerr'] }, { ids: [slot('impedance', 'effort')] }],
        notice:
          'A light target mass makes the tool feel nimble, but the controller has to fight the real task inertia Λ(q): larger, faster torques. With a real force sensor this amplifies noise.',
      },
      {
        title: 'Impedance vs. admittance',
        goal: 'Same push, the dual controller as a shadow.',
        actions: [{ do: 'hold' }, { do: 'shadow', key: 'admittance' }, { do: 'push', inward: 0.4, ms: 4000 }],
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
        actions: [{ do: 'hold' }, { do: 'push', inward: 0.4, ms: 4000 }],
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
          { do: 'push', inward: 0.4, ms: 3000 },
        ],
        plots: [{ ids: [tele('admittance', 'xv', 0), tele('admittance', 'x', 0)] }],
        notice:
          'The cascade needs the inner loop to be much faster than the virtual dynamics; with ω_in close to $\\sqrt{K_d/\\Lambda_d}$ the tool lags and overshoots.',
      },
    ],
  };
};

const lqrLab: LabFn = (c) => {
  const { q, u, under, cartLike, balanced } = common(c);
  const watch = [{ ids: q }, { ids: u }, { ids: [tele('lqr', 'cost')] }];
  if (!balanced) return { watch, experiments: [] };
  const experiments: Experiment[] = [
    {
      title: 'Region of attraction',
      goal: 'Kick harder and harder (press Space): when does the linear controller give up?',
      actions: [{ do: 'atTarget' }, { do: 'kick', v: under ? 0.6 : 2 }],
      plots: [{ ids: q }, { ids: [tele('lqr', 'cost')] }],
      notice:
        'LQR is optimal for the linearisation only. Small kicks die out exponentially; past some size the nonlinear terms dominate and the state leaves the region of attraction — for an underactuated pendulum it falls.',
    },
    {
      title: 'Cheap control',
      goal: 'Make inputs cheap (R → 0.01) and kick.',
      actions: [
        { do: 'param', key: 'r', value: 0.01 },
        { do: 'atTarget' },
        { do: 'kick', v: under ? 0.3 : 1 },
      ],
      plots: [{ ids: q }, { ids: u }],
      notice:
        'Cheap control means high gains: faster recovery, much larger inputs. The gain $K$ in the derivation changes with $R$ (the Riccati equation is re-solved).',
    },
  ];
  if (cartLike) {
    // verified on the cart-pole from q*: at u_max = 4 N both recover the same kicks (and fail the same)
    experiments.push({
      title: 'Clipping vs. planning',
      goal: 'Tight actuator limits, MPC as a shadow, same kick.',
      actions: [
        { do: 'shadow', key: 'mpc' },
        { do: 'setting', key: 'umax', value: 4 },
        { do: 'atTarget' },
        { do: 'kick', v: 0.8 },
      ],
      plots: [
        { ids: [slot('lqr', 'err'), slot('mpc', 'err')] },
        { ids: [slot('lqr', 'effort'), slot('mpc', 'effort')] },
      ],
      notice:
        'LQR computes $u = u^* - Kx$ and the actuator clips it; MPC knows the limit and plans a sequence within it. Both hit the bound — and both recover, with nearly the same response: with only input bounds and the LQR cost-to-go as terminal cost, MPC behaves much like saturated LQR on this system (and a kick that topples one topples the other). MPC earns its cost with constraints LQR cannot express, such as limits on the state (the rail) — which this MPC does not model.',
    });
  }
  return { watch, experiments };
};

const mpcLab: LabFn = (c) => {
  const { q, u, under, uff, cartLike, balanced } = common(c);
  if (!balanced) return { watch: [{ ids: q }, { ids: u }], experiments: [] };
  if (!under) {
    return {
      watch: [{ ids: q }, { ids: u }, { ids: [tele('mpc', 'active'), tele('mpc', 'iterations')] }],
      experiments: [
        {
          title: 'Active constraints',
          goal: 'Tight limits and a kick: watch the bounds become active.',
          actions: [
            { do: 'setting', key: 'umax', value: Math.round(Math.abs(uff) + 1) },
            { do: 'atTarget' },
            // both ways: the bound sits just above the holding input, and only one direction loads it
            { do: 'kick', v: 4 },
            { do: 'wait', ms: 1500 },
            { do: 'kick', v: -4 },
          ],
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
        actions: [{ do: 'setting', key: 'umax', value: 4 }, { do: 'atTarget' }, { do: 'kick', v: 0.5 }],
        plots: [{ ids: u }, { ids: [tele('mpc', 'active'), tele('mpc', 'iterations')] }],
        notice:
          'While the plan pushes against $u_{\\max}$, several of the $N$ planned inputs sit on the bound (active constraints) and FISTA needs more iterations. Once the state is small, no bound is active and MPC returns the LQR input.',
      },
      ...(cartLike
        ? ([
            {
              title: 'A coarse model',
              goal: 'Predict with Δt = 0.05 s over 3 s, against LQR at the 1 kHz rate.',
              actions: [
                { do: 'param', key: 'horizon', value: 60 },
                { do: 'param', key: 'step', value: 0.05 },
                { do: 'shadow', key: 'lqr' },
                { do: 'setting', key: 'umax', value: 6 },
                { do: 'atTarget' },
                { do: 'kick', v: 0.8 },
              ],
              plots: [
                { ids: [slot('mpc', 'err'), slot('lqr', 'err')] },
                { ids: [slot('mpc', 'effort'), slot('lqr', 'effort')] },
              ],
              notice:
                'A longer horizon is not automatically better: each re-plan holds the input for 50 ms and predicts with a model discretised at that step, while the pendulum moves on. Here the clipped 1 kHz LQR recovers and the coarse MPC does not. Prediction accuracy and update rate matter as much as foresight.',
            },
          ] satisfies Experiment[])
        : []),
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
