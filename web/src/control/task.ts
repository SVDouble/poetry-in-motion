/** Pages of the task-space controllers acting on the tool: virtual model control, impedance, admittance. */
import type { SectionBuilder } from '../eq/panel';
import { texMat } from '../eq/tex';
import { col, idleNote, type Page, type PageContext, realise } from './page';

/** Tool position, its Jacobian and the live task state. */
function tool(b: SectionBuilder, c: PageContext): void {
  const t = c.d.tool;
  if (!t) return;
  b.math(
    String.raw`x = p_{st}(q) = ${col(t.p)},\qquad J(q) = \frac{\partial p_{st}}{\partial q} = ${texMat(t.Jp)}`,
  );
  b.math(
    String.raw`x = ${b.live.vec(2, (i) => c.snap.v('TOOL_P', i))},\quad x_d = ${b.live.vec(2, (i) => c.snap.v('X_REF', i))},\quad F_{\text{ext}} = ${b.live.vec(2, (i) => c.snap.v('FEXT', i))}\,\mathrm{N}`,
    { caption: 'tool, smoothed task reference and the push (Shift-drag in the viewport)' },
  );
}

function nullSpace(b: SectionBuilder, c: PageContext, how: string): void {
  if (c.dof <= 2) {
    b.text(
      `With ${c.dof} degree${c.dof === 1 ? '' : 's'} of freedom the tool task uses all of them: there is no null space for secondary goals.`,
    );
    return;
  }
  b.text(
    `The tool needs 2 of the ${c.dof} degrees of freedom. The remaining ${c.dof - 2} pursue the secondary goals of the Goals tab (posture, manipulability) ${how}, without disturbing the tool:`,
  );
}

export const vmc = (c: PageContext): Page => {
  const { n } = c;
  return {
    lead: 'A virtual spring and damper pull the tool to its target; gravity is compensated. Pure energy shaping — no inverse dynamics.',
    cite: 'Takegaki & Arimoto 1981; Pratt et al. 2001',
    idea(b) {
      b.text(
        'Imagine a spring from the target to the tool and a damper on the tool. The forces they would exert are mapped to joint torques through the Jacobian transpose — the principle of virtual work. Replacing the real potential (gravity) by the virtual one <i>shapes</i> the energy of the closed loop so that its minimum is at the target; the damper <i>injects</i> dissipation. Nothing is cancelled: the robot keeps its natural inertia, which makes the law passive and safe in contact.',
      );
    },
    generic(b) {
      b.math(
        String.raw`\tau = J(q)^{T}\Big(K\,(x_d - x) - D\,(\dot x - \dot x_d)\Big) + \frac{\partial V}{\partial q}(q) + N_{\perp}\,\tau_0 - d\,\big(M - J^{T}\hat\Lambda J\big)\,\dot q`,
      );
      b.text(
        String.raw`Virtual work: a force $F$ at the tool does the work $F^{T}\delta x = F^{T}J\,\delta q$, so the equivalent joint torque is $J^{T}F$. The damping is set from the task-space inertia $\Lambda = (J M^{-1} J^{T})^{-1}$ so that the tool behaves like a damped oscillator with ratio ζ — through the harmonic mean of its eigenvalues, which stays finite at a singular pose (their plain mean would not, and an unbounded damping gain is unstable in a 1 kHz controller):`,
      );
      b.math(
        String.raw`D = 2\zeta\sqrt{K\,\bar\lambda},\qquad \bar\lambda = \frac{2}{\operatorname{tr}\big(J M^{-1} J^{T}\big)}`,
      );
      b.text(
        String.raw`The last term damps what the task cannot see: $M - J^{T}\hat\Lambda J$ is symmetric and positive semi-definite ($\hat\Lambda \le \Lambda$ is the task inertia with its near-singular directions damped), so it only removes energy. Away from singularities it acts in the null space alone; at a stretched arm it catches the elbow mode, which barely moves the tool and would otherwise ring.`,
      );
      nullSpace(b, c, 'through the dynamically consistent projector');
      b.math(
        String.raw`N_{\perp} = I - J^{T}\bar J^{T},\qquad \bar J = M^{-1}J^{T}\Lambda\qquad\Rightarrow\qquad J M^{-1} N_{\perp}\tau_0 = 0`,
      );
    },
    concrete(b) {
      tool(b, c);
      b.math(
        String.raw`F = ${b.live.vec(2, (i) => c.tele('force', i))}\,\mathrm{N},\quad J^{T}F = ${b.live.vec(n, (i) => c.tele('tau_task', i))},\quad N_{\perp}\tau_0 = ${b.live.vec(n, (i) => c.tele('tau_null', i))}`,
      );
      if (c.dof > 2 && !c.loops) {
        b.math(String.raw`N_{\perp} = ${b.live.mat(n, n, (i, j) => c.tele('projector', i * n + j))}`);
        b.math(
          String.raw`\|J M^{-1} N_{\perp}\tau_0\| = ${b.live.num(() => c.tele('leak'))}\,\mathrm{m/s^2}`,
          { caption: 'leakage: tool acceleration caused by the secondary goals (zero up to round-off)' },
        );
      }
      realise(b, c, String.raw`J^{T}F + \frac{\partial V}{\partial q} + N_{\perp}\tau_0`);
      b.math(
        String.raw`w = \sqrt{\det J J^{T}} = ${b.live.num(() => c.tele('w'))},\qquad V_L = ${b.live.num(() => c.tele('lyap'))}\,\mathrm{J},\quad \dot V_L = ${b.live.num(() => c.tele('lyap_dot'))}\,\mathrm{W}`,
      );
      idleNote(b, c);
    },
    stability(b) {
      b.text('The closed-loop energy is the kinetic energy plus the energy stored in the virtual spring:');
      b.math(String.raw`V_L = \tfrac12\dot q^{T}M\dot q + \tfrac12 (x_d - x)^{T}K(x_d - x)`);
      b.math(
        String.raw`\dot V_L = -\dot x^{T}D\,\dot x - d\,\dot q^{T}\big(M - J^{T}\hat\Lambda J\big)\dot q - \dot q^{T}\operatorname{diag}\beta\,\dot q + \dot q^{T}N_\perp\tau_0`,
      );
      b.text(
        'For a fixed target and no secondary goal the energy can only decrease: the tool converges to x<sub>d</sub> (LaSalle). A target out of reach is no exception: x<sub>d</sub> then stops at the nearest point the arm can reach (found by inverse kinematics, a hair inside the boundary), and the arm comes to rest there, stretched towards the target. The law is passive from the push F<sub>ext</sub> to the tool velocity, which is why it behaves gracefully when you push it.',
      );
    },
    limits(b) {
      b.text(
        'Tracking is only approximate (the spring lags a moving target). Near singularities (w → 0) the Jacobian transpose cannot produce force in some direction: a target out of reach is approached as far as the arm goes, not reached. The null-space projection needs M and an open chain here; with loops the secondary goals are disabled.',
      );
    },
  };
};

export const impedance = (c: PageContext): Page => {
  const { n } = c;
  return {
    lead: 'Hogan’s impedance control: the tool behaves like a chosen mass–spring–damper when pushed.',
    cite: 'Hogan 1985; Khatib 1987',
    idea(b) {
      b.text(
        'Instead of rejecting disturbances, specify how the tool should <i>respond</i> to them. A force at the tool should produce the motion of a prescribed mass Λ<sub>d</sub> on a spring K<sub>d</sub> with damping D<sub>d</sub>. Motion follows force — the controller renders a mechanical impedance.',
      );
    },
    generic(b) {
      b.math(String.raw`\Lambda_d\,\ddot e + D_d\,\dot e + K_d\,e = F_{\text{ext}},\qquad e = x - x_d`, {
        caption: 'target dynamics of the tool',
      });
      b.text('Solve for the tool acceleration this demands:');
      b.math(
        String.raw`\ddot x_{\text{cmd}} = \ddot x_d + \Lambda_d^{-1}\big(F_{\text{ext}} - D_d\,\dot e - K_d\,e\big),\qquad D_d = 2\zeta\sqrt{K_d\Lambda_d}`,
      );
      b.text(
        String.raw`Resolve it into joint accelerations: $J\ddot q + \dot J\dot q = \ddot x_{\text{cmd}}$ fixes 2 components of $\ddot q$; the rest is chosen closest (in the metric of $M$) to the null-space preference $\ddot q_0 = k_\perp M^{-1}\tau_0$:`,
      );
      b.math(
        String.raw`\ddot q = \arg\min\ \|\ddot q - \ddot q_0\|^2_{M}\quad\text{s.t.}\quad J\ddot q = \ddot x_{\text{cmd}} - \dot J\dot q${c.loops ? String.raw`,\ A\ddot q = -\dot A\dot q` : ''}`,
      );
      b.text(
        String.raw`The robot itself feels the push as $J^{T}F_{\text{ext}}$ in $M\ddot q + C\dot q + N = \tau + J^{T}F_{\text{ext}}$. Cancelling it (with a measured force) and inverting the dynamics gives`,
      );
      b.math(String.raw`\tau = M(q)\,\ddot q + C(q,\dot q)\dot q + N(q,\dot q) - J^{T}F_{\text{ext}}`);
      b.text(
        String.raw`At rest $K_d e = F_{\text{ext}}$: a push of 10 N deflects the tool by $10/K_d$ metres, in every pose.`,
      );
    },
    concrete(b) {
      tool(b, c);
      b.math(
        String.raw`e = ${b.live.vec(2, (i) => c.tele('e', i))},\quad \ddot x_{\text{cmd}} = ${b.live.vec(2, (i) => c.tele('xdd_cmd', i))},\quad F_{\text{ext}}/K_d = ${b.live.vec(2, (i) => c.tele('fext', i) / c.p('k'))}`,
        {
          caption: 'deflection, commanded tool acceleration and the static deflection the push should cause',
        },
      );
      nullSpace(b, c, 'as an acceleration preference');
      if (c.dof > 2) {
        b.math(
          String.raw`M(\ddot q - \ddot q_{\text{task}}) = ${b.live.vec(n, (i) => c.tele('tau_null', i))},\qquad \|J(\ddot q - \ddot q_{\text{task}})\| = ${b.live.num(() => c.tele('leak'))}`,
          { caption: 'the part of the torque spent on secondary goals, and its effect on the tool (zero)' },
        );
      }
      realise(b, c, String.raw`M\ddot q + C\dot q + N - J^{T}F_{\text{ext}}`);
      idleNote(b, c);
    },
    stability(b) {
      b.text(
        'With an exact model and exact force measurement the tool follows the linear target dynamics exactly, which are stable for any positive Λ<sub>d</sub>, D<sub>d</sub>, K<sub>d</sub>. In contact with a passive environment the coupled system stays stable because the rendered impedance is passive.',
      );
    },
    limits(b) {
      b.text(
        'Needs the push to be measured (here the simulation knows it exactly — a perfect force sensor). Choosing Λ<sub>d</sub> far below the natural task inertia Λ(q) makes the controller fight the robot and amplifies sensor noise; choosing Λ<sub>d</sub> = Λ(q) removes the need for force sensing (and turns it into VMC). Singular poses make Λ(q) unbounded: near them the commanded acceleration is resolved with adaptive damped least squares (exact while the task is well conditioned, bounded and fading as it loses a direction), and the directions it lets go of are damped. A target out of reach would ask for an acceleration no pose can realise, pulling ever harder against the singularity (a redundant arm then snakes back and forth); x<sub>d</sub> therefore stops at the nearest reachable point, and the arm comes to rest stretched towards the target. Like every law that cancels the dynamics it relies on the actuators delivering the computed torque; when they cannot, the commanded part is scaled down as a whole (same direction, gentler) rather than clipped joint by joint, which would break the cancellation.',
      );
    },
  };
};

export const admittance = (c: PageContext): Page => ({
  lead: 'The dual of impedance: a measured force drives a virtual mass–spring–damper, and a stiff position loop follows it.',
  cite: 'Hogan 1985; Ott et al. 2010',
  idea(b) {
    b.text(
      'Impedance control computes force from motion; admittance control computes motion from force. The push is fed into a simulated mass on a spring (the virtual tool x<sub>v</sub>), and the real tool tracks x<sub>v</sub> with a stiff inner loop. This is how compliance is added to industrial position-controlled robots that cannot command torques.',
    );
  },
  generic(b) {
    b.math(
      String.raw`\Lambda_d\,\ddot x_v + D_d\,(\dot x_v - \dot x_d) + K_d\,(x_v - x_d) = F_{\text{ext}}`,
      { caption: 'virtual dynamics, integrated every step' },
    );
    b.math(
      String.raw`\ddot x_{\text{cmd}} = \ddot x_v + \omega_{\text{in}}^2\,(x_v - x) + 2\omega_{\text{in}}\,(\dot x_v - \dot x),\qquad \tau = M\ddot q + C\dot q + N`,
      { caption: 'inner position loop (resolved as for impedance, without cancelling F_ext)' },
    );
    b.text(
      String.raw`The push is <i>not</i> cancelled: the inner loop resists it like a spring of stiffness $\approx\Lambda(q)\,\omega_{\text{in}}^2$. In steady state two springs act in series:`,
    );
    b.math(
      String.raw`x - x_d = \underbrace{K_d^{-1}F_{\text{ext}}}_{\text{virtual}} + \underbrace{\big(\Lambda\,\omega_{\text{in}}^{2}\big)^{-1}F_{\text{ext}}}_{\text{inner loop}}`,
    );
  },
  concrete(b) {
    tool(b, c);
    b.math(
      String.raw`x_v = ${b.live.vec(2, (i) => c.tele('xv', i))},\quad \dot x_v = ${b.live.vec(2, (i) => c.tele('xv_dot', i))},\quad x - x_v = ${b.live.vec(2, (i) => c.tele('x', i) - c.tele('xv', i))}`,
      { caption: 'virtual tool, its velocity and how far the real tool lags behind it' },
    );
    b.math(
      String.raw`K_d^{-1}F_{\text{ext}} = ${b.live.vec(2, (i) => c.tele('fext', i) / c.p('k'))}\quad\text{vs.}\quad x_v - x_d = ${b.live.vec(2, (i) => c.tele('xv', i) - c.snap.v('X_REF', i))}`,
    );
    realise(b, c, String.raw`M\ddot q + C\dot q + N`);
    idleNote(b, c);
  },
  stability(b) {
    b.text(
      'The virtual dynamics are a stable linear system; the inner loop is computed torque in task space. Stability of the cascade needs the inner loop to be much faster than the virtual dynamics (ω<sub>in</sub> ≫ √(K<sub>d</sub>/Λ<sub>d</sub>)).',
    );
  },
  limits(b) {
    b.text(
      'Admittance excels in free space and with soft environments; against stiff contacts the stiff inner loop and the force feedback can oscillate. Impedance has the opposite profile: good in stiff contact, less precise in free space. Light virtual masses (small Λ<sub>d</sub>) feel responsive but need fast, clean force signals.',
    );
  },
});
