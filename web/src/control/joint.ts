/** Pages of the joint-space controllers. */
import { texMat } from '../eq/tex';
import { solve } from './nullspace';
import { actuatedIdx, col, idleNote, type Page, type PageContext, qVec, realise } from './page';

const err = (c: PageContext) => (i: number) => c.snap.v('Q_REF', i) - c.snap.v('Q', i);

/** q*, q_d and e = q_d − q, live. */
function references(c: PageContext, b: Parameters<Page['concrete']>[0]): void {
  const { n, snap } = c;
  b.math(
    String.raw`q = ${qVec(c.d)},\quad q^{*} = ${b.live.vec(n, (i) => snap.v('Q_TARGET', i))},\quad
    q_d = ${b.live.vec(n, (i) => snap.v('Q_REF', i))},\quad e = q_d - q = ${b.live.vec(n, err(c))}`,
    { caption: 'target, smoothed reference (critically damped filter at ω_ref) and tracking error' },
  );
}

export const passive = (c: PageContext): Page => ({
  lead: 'No control at all: the mechanism moves under gravity, friction, the loops and your pushes.',
  idea(b) {
    b.text(
      'The baseline every controller is compared with. With <i>u</i> = 0 the equations of motion are a statement about energy: nothing but external work and friction can change it.',
    );
  },
  generic(b) {
    b.math(String.raw`M(q)\ddot q + C(q,\dot q)\dot q + N(q,\dot q) = \tau_{\text{ext}},\qquad u = 0`);
    b.text(
      String.raw`Multiplying by $\dot q^{T}$ and using the skew-symmetry of $\dot M - 2C$ gives the power balance`,
    );
    b.math(
      String.raw`\dot E = \frac{d}{dt}\Big(\tfrac12 \dot q^{T} M \dot q + V(q)\Big) = \dot q^{T}\tau_{\text{ext}} - \dot q^{T}\operatorname{diag}(\beta)\,\dot q`,
    );
  },
  concrete(b) {
    const s = c.snap;
    b.math(
      String.raw`E = ${b.live.num(() => s.v('ENERGY', 2))}\,\mathrm{J},\quad \dot q^{T}\tau = ${b.live.num(() => s.v('POWER'))}\,\mathrm{W},\quad \dot q^{T}\operatorname{diag}(\beta)\dot q = ${b.live.num(() => s.v('DISSIPATION'))}\,\mathrm{W}`,
    );
  },
  stability(b) {
    b.text(
      'Energy can only decrease without external work, so the motion is bounded. With friction it settles in a local minimum of <i>V</i> (a hanging equilibrium); inverted equilibria are unstable.',
    );
  },
  limits(b) {
    b.text('Nothing to tune — and nothing to steer.');
  },
});

function pdPage(c: PageContext, gravity: boolean): Page {
  const { n, d, snap } = c;
  // K_p, packed row-major n × n
  const kp = () => Array.from({ length: n * n }, (_, k) => c.tele('kp', k));
  const sag = () =>
    solve(
      n,
      kp(),
      Array.from({ length: n }, (_, i) => snap.v('DV', i)),
      1,
    ) ?? new Array(n).fill(Number.NaN);
  return {
    lead: gravity
      ? 'Joint PD plus exact gravity compensation: no steady-state error, and stability follows from energy alone.'
      : 'A spring and damper on the actuated joints, shaped by the inertia at the target. Simple and robust, but gravity makes it sag.',
    cite: gravity ? 'Takegaki & Arimoto 1981; MLS §4.5' : 'MLS §4.5',
    idea(b) {
      b.text(
        gravity
          ? 'A PD controller is a virtual spring and damper on every joint. Adding the gravity force the mechanism needs at its current pose cancels the only thing that pulls it away from the target, so the springs can settle exactly at q*. No model of the inertia is needed — only of the potential energy.'
          : 'Each actuated joint is pulled towards its reference by a virtual spring and slowed by a damper. The gains are constant matrices shaped like the inertia at the target, M(q*): near q* the mechanism then responds in every direction with the natural frequency ω<sub>n</sub> and damping ratio ζ you set — no direction sluggish, none violently stiff.',
      );
    },
    generic(b) {
      b.math(
        gravity
          ? String.raw`u = K_p\,e + K_v\,\dot e + u_g,\qquad B u_g = \frac{\partial V}{\partial q}(q)`
          : String.raw`u = K_p\,e + K_v\,\dot e,\qquad e = q_d - q`,
      );
      b.math(String.raw`K_p = \omega_n^2\, M(q^{*}),\qquad K_v = 2\zeta\omega_n\, M(q^{*})`);
      b.text(
        String.raw`Linearised at $q^{*}$ (gravity balanced), $M(q^{*})\,\ddot e + K_v\dot e + K_p e = 0$ becomes $M(q^{*})\big(\ddot e + 2\zeta\omega_n\dot e + \omega_n^2 e\big) = 0$: every mode has exactly $\omega_n$ and $\zeta$. The classical decentralised choice $K_p = \omega_n^2 \operatorname{diag} M(q^{*})$ treats each joint alone; in a strongly coupled chain the modes then spread by orders of magnitude — some sluggish, some so stiff that a 1 kHz digital controller destabilises them.`,
      );
      if (!gravity) {
        b.text(
          String.raw`At rest ($\dot q = 0$, $q_d = q^{*}$) the closed loop balances the springs against gravity, $K_p e = \partial V / \partial q$ (fully actuated open chain), so the arm settles with the error`,
        );
        b.math(String.raw`e_{ss} = K_p^{-1}\,\frac{\partial V}{\partial q}(q_{ss}) \neq 0`);
        b.text('Stiffer gains shrink the sag but never remove it — and amplify noise and saturation.');
      } else {
        b.text(
          'With the gravity term the rest condition becomes K<sub>p</sub> e = 0, i.e. e = 0 for any positive gains.',
        );
      }
    },
    concrete(b) {
      references(c, b);
      b.math(
        String.raw`K_p = ${b.live.mat(n, n, (i, j) => c.tele('kp', i * n + j))},\quad K_v = ${b.live.mat(n, n, (i, j) => c.tele('kd', i * n + j))}`,
        {
          caption: c.loops
            ? 'all coordinates take part: the loops tie them together'
            : 'on an open chain only the actuated joints’ errors enter (a collocated servo)',
        },
      );
      b.math(
        String.raw`\frac{\partial V}{\partial q} = ${col(d.dV)} = ${b.live.vec(n, (i) => snap.v('DV', i))}`,
      );
      if (gravity) {
        b.text('The gravity inputs at the current pose:');
        realise(b, c, String.raw`\frac{\partial V}{\partial q}(q)`);
        b.math(String.raw`u_g = ${b.live.vec(c.nu, (i) => c.tele('ug', i))}`);
      } else {
        b.math(
          String.raw`K_p^{-1}\frac{\partial V}{\partial q}(q) = ${b.live.vec(n, (i) => sag()[i] ?? Number.NaN)}\quad\text{vs.}\quad e = ${b.live.vec(n, err(c))}`,
          { caption: 'predicted sag versus the actual error (equal once at rest, for actuated joints)' },
        );
      }
      idleNote(b, c);
    },
    stability(b) {
      b.text(
        String.raw`For a set-point ($\dot q_d = 0$) take the energy of the closed loop: kinetic energy plus the virtual springs${gravity ? '' : ' plus gravity'}.`,
      );
      b.math(
        gravity
          ? String.raw`V_L = \tfrac12\dot q^{T}M\dot q + \tfrac12 e^{T}K_p e,\qquad \dot V_L = -\dot q^{T}K_v\dot q - \dot q^{T}\operatorname{diag}(\beta)\dot q \le 0`
          : String.raw`V_L = \tfrac12\dot q^{T}M\dot q + \tfrac12 e^{T}K_p e + V(q),\qquad \dot V_L = -\dot q^{T}K_v\dot q - \dot q^{T}\operatorname{diag}(\beta)\dot q \le 0`,
      );
      b.text(
        String.raw`The step uses $\tfrac12\dot q^{T}(\dot M - 2C)\dot q = 0$. $\dot V_L$ vanishes only when $\dot q = 0$; there the dynamics force ${gravity ? '$K_p e = 0$' : 'the rest condition'}, so by LaSalle's principle the state converges to ${gravity ? 'q* (globally, for a fully actuated open chain)' : 'the sagged equilibrium (a local minimum of V_L)'}.`,
      );
    },
    limits(b) {
      b.text(
        gravity
          ? 'Needs an exact potential-energy model (a wrong mass leaves a residual error). Transients are still shaped by M(q): the response differs from pose to pose and the joints are coupled. Tracking a moving reference is only approximate.'
          : 'Steady-state error under gravity; transients coupled through M(q); and with an underactuated mechanism the passive joints are simply left alone (PD cannot balance a pendulum).',
      );
    },
  };
}

export const pd = (c: PageContext) => pdPage(c, false);
export const pdGravity = (c: PageContext) => pdPage(c, true);

export const computedTorque = (c: PageContext): Page => {
  const { n, d, loops } = c;
  const act = actuatedIdx(d);
  return {
    lead: 'Inverse dynamics: cancel the whole model so that every joint error obeys the same linear ODE everywhere.',
    cite: loops ? 'MLS §4.5, ch. 6' : 'MLS §4.5',
    idea(b) {
      b.text(
        'If the model is exact, we can compute the torque that produces any acceleration we like. Ask for the reference acceleration plus a PD correction, and the nonlinear robot behaves like n decoupled double integrators with a PD loop.',
      );
    },
    generic(b) {
      b.math(
        String.raw`\tau = M(q)\,v + C(q,\dot q)\dot q + N(q,\dot q),\qquad v = \ddot q_d + K_v\dot e + K_p e`,
      );
      b.text(
        String.raw`Substituting into $M\ddot q + C\dot q + N = \tau$ and cancelling $M \succ 0$ gives $\ddot q = v$, hence`,
      );
      b.math(String.raw`\ddot e + K_v\dot e + K_p e = 0,\qquad K_p = \omega_c^2 I,\ K_v = 2\zeta\omega_c I`);
      if (loops) {
        b.text(
          String.raw`With loops only the actuated coordinates are free. They receive $v_a$; the constraints decide the rest, $A\ddot q + \dot A\dot q = 0$. Among the accelerations compatible with both, the one closest to $v$ in the metric of $M$ is taken:`,
        );
        b.math(
          String.raw`\ddot q = \arg\min_{\ddot q}\ \|\ddot q - v\|_{M}^{2}\quad\text{s.t.}\quad \begin{bmatrix} A \\ S_a \end{bmatrix}\ddot q = \begin{bmatrix} -\dot A\dot q \\ v_a \end{bmatrix}`,
        );
      }
    },
    concrete(b) {
      references(c, b);
      b.math(
        String.raw`\ddot q = ${b.live.vec(n, (i) => c.tele('qdd', i))},\quad \omega_c = ${b.live.num(() => c.p('wn'))}\,\mathrm{rad/s},\ \zeta = ${b.live.num(() => c.p('zeta'))}`,
        { caption: 'the commanded acceleration' },
      );
      b.math(String.raw`M(q) = ${texMat(d.M)}`);
      realise(b, c, String.raw`M(q)\ddot q + C(q,\dot q)\dot q + N(q,\dot q)`);
      b.text(
        act.length === n
          ? 'Every coordinate is actuated: the law is exact.'
          : `Actuated coordinates: ${act.map((i) => `$${d.coords[i]!.tex}$`).join(', ')}; the loops carry the others.`,
      );
      idleNote(b, c);
    },
    stability(b) {
      b.text(
        'With an exact model the error dynamics are linear and exponentially stable for any ω<sub>c</sub> > 0, ζ > 0 — globally, in every configuration. This is the cleanest guarantee of all controllers here, and the most fragile one: it relies on cancellation.',
      );
    },
    limits(b) {
      b.text(
        'Needs a full actuator per degree of freedom and an accurate model; model errors enter as a disturbance M̂⁻¹ΔM v. High ω<sub>c</sub> saturates the actuators (see u<sub>max</sub>). Cancelling the natural dynamics also cancels useful passivity.',
      );
      b.text(String.raw`A push is an unmodelled disturbance. The error dynamics become`);
      b.math(
        String.raw`M(q)\big(\ddot e + K_v\dot e + K_p e\big) = -J^{T}F_{\text{ext}}\qquad\Rightarrow\qquad e_{ss} = -K_p^{-1}M(q)^{-1}J^{T}F_{\text{ext}}`,
      );
      b.text(
        'so the stiffness against a push is M(q) ω<sub>c</sub><sup>2</sup>: stiff for heavy proximal links, soft for light distal ones. Shift-drag the tool of the three-link arm and watch the last link fold while the impedance shadow yields as a clean 2-D spring.',
      );
    },
  };
};

export const passivity = (c: PageContext): Page => {
  const { n } = c;
  return {
    lead: 'Slotine–Li tracking: use the energy structure of the model instead of cancelling it.',
    cite: 'Slotine & Li 1987; MLS §4.5',
    idea(b) {
      b.text(
        'Rather than linearise the robot, choose a reference velocity that already contains the error correction, and feed forward the dynamics along it. The closed loop stays a mechanical system (with M and C), and the skew-symmetry of Ṁ − 2C makes a simple energy function decrease.',
      );
    },
    generic(b) {
      b.math(
        String.raw`\dot q_r = \dot q_d + \Lambda e,\qquad s = \dot q - \dot q_r = -(\dot e + \Lambda e)`,
        { caption: 'reference velocity and sliding variable' },
      );
      b.math(String.raw`\tau = M(q)\,\ddot q_r + C(q,\dot q)\,\dot q_r + N(q,\dot q) - K_D\,s`);
      b.text('Subtracting from the equations of motion leaves a mechanical system in s:');
      b.math(String.raw`M(q)\,\dot s + C(q,\dot q)\,s + K_D\,s = 0`);
      b.text(
        String.raw`$\Lambda = \lambda I$ sets how fast $e$ follows once $s = 0$ (first order: $\dot e = -\lambda e$); $K_D = k_D\, M(q^{*})$ how hard $s$ is driven to zero — shaped like the inertia, so every direction converges alike.`,
      );
    },
    concrete(b) {
      references(c, b);
      b.math(
        String.raw`s = ${b.live.vec(n, (i) => c.tele('s', i))},\quad V = ${b.live.num(() => c.tele('lyap'))},\quad \dot V = ${b.live.num(() => c.tele('lyap_dot'))}`,
        { caption: 'the Lyapunov function below, evaluated on the simulation' },
      );
      realise(b, c, String.raw`M\ddot q_r + C\dot q_r + N - K_D s`);
      idleNote(b, c);
    },
    stability(b) {
      b.math(String.raw`V = \tfrac12 s^{T} M s + e^{T}\Lambda K_D e`);
      b.math(
        String.raw`\dot V = s^{T}(M\dot s + \tfrac12\dot M s) + 2e^{T}\Lambda K_D\dot e = -s^{T}K_D s + 2e^{T}\Lambda K_D\dot e + \underbrace{\tfrac12 s^{T}(\dot M - 2C)s}_{=0}`,
      );
      b.math(
        String.raw`\dot V = -\dot e^{T}K_D\dot e - e^{T}\Lambda K_D\Lambda e < 0\quad\text{for } (e,\dot e) \neq 0`,
      );
      b.text(
        c.loops
          ? 'Global asymptotic tracking for a fully actuated open chain. With loops the argument has to be made in independent coordinates (the constraint forces enter the s-dynamics), and V written in the full q is not guaranteed to decrease monotonically — the controller still converges, but watch V wiggle.'
          : 'Global asymptotic tracking for a fully actuated system — and, unlike computed torque, the proof survives bounded parameter errors well (it is the basis of adaptive control).',
      );
    },
    limits(b) {
      b.text(
        'Still a model-based, fully actuated law. The derivative of V on the page is numerical, so it is noisy at large steps. Saturation breaks the argument.',
      );
    },
  };
};
