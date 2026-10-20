/**
 * The derivation of the equations of motion for the built mechanism, step by step (MLS ch. 4):
 * coordinates → kinematics → COM velocities → T and V → M → Christoffel symbols → C → N → equations.
 * Every formula comes from the core's symbolic engine for this specific topology, with generic coefficients.
 */

import type { Derivation } from '../model/derivation';
import type { Mechanism } from '../model/description';
import type { SectionSpec } from './panel';
import { texMat } from './tex';

const col = (xs: string[]) => texMat(xs.map((x) => [x]));
const cls = (c: string, s: string) => String.raw`\htmlClass{${c}}{${s}}`;

const portName = (frac: number) =>
  frac === 0 ? 'origin' : frac === 1 ? 'tip' : `$${frac.toFixed(2).replace(/0+$/, '')}\\,l$ along`;

function describeJoint(mech: Mechanism, d: Derivation, i: number): string {
  const b = mech.bodies[i]!;
  const c = d.coords.find((x) => x.body === i);
  const where =
    b.parent < 0
      ? `on the ground at $(${b.mountX.toFixed(2)}, ${b.mountY.toFixed(2)})$`
      : `at the ${portName(b.mountS)} of body ${b.parent + 1}`;
  if (b.joint === 'fixed') return `Body ${i + 1}: <b>welded</b> ${where} — no coordinate.`;
  const act = b.actuated ? 'actuated' : '<em>passive</em>';
  return `Joint ${i + 1}: ${b.joint}, ${where}, ${act} — coordinate $${c?.tex ?? ''}$.`;
}

export function derivationSections(mech: Mechanism, d: Derivation): SectionSpec[] {
  const n = d.n;
  const q = col(d.coords.map((c) => c.tex));
  const qd = col(d.coords.map((c) => c.dot));
  const steps: SectionSpec[] = [];
  const step = (title: string, cite: string | undefined, build: SectionSpec['build']) =>
    steps.push({
      id: `d-${steps.length}`,
      title: `${steps.length + 1}. ${title}`,
      ...(cite ? { cite } : {}),
      build,
    });

  step('Topology & generalised coordinates', 'MLS §4.2', (b) => {
    const nb = mech.bodies.length;
    const fixed = nb - n;
    const loops = d.closures.length;
    b.text(
      `The mechanism has ${nb} rigid bodies connected as a ${loops ? 'graph' : 'tree'}. Along its spanning tree every moving joint adds one degree of freedom${fixed ? ` (the ${fixed} welded one${fixed > 1 ? 's add' : ' adds'} none)` : ''}, so the ${n} joint variables are <b>generalised coordinates</b>: Lagrange's equations then need no internal forces, which makes them the easiest route to the dynamics.${loops ? ` The ${loops} loop-closing pin${loops > 1 ? 's' : ''} will add ${2 * loops} constraints (see “Loop-closure constraints”), leaving ${n - 2 * loops} degree${n - 2 * loops === 1 ? '' : 's'} of freedom.` : ''}`,
    );
    for (let i = 0; i < nb; i++) b.text(describeJoint(mech, d, i), 'prose item');
    b.math(String.raw`q = ${q} \in Q,\qquad \dot q = ${qd}`);
    b.text(
      'Each body $i$ carries generic coefficients: mass $m_i$, length $l_i$, centre of mass at distance $r_i$ along its $x$-axis, inertia $I_i$ about the centre of mass, and joint damping $\\beta_i$. Gravity $g$ acts along $-y$.',
    );
  });

  step('Forward kinematics', 'MLS §2.3, (3.1)', (b) => {
    b.text(
      'Attach a frame to every body (origin at its joint, $x$-axis along the body). Each joint contributes a rigid motion $g_{p,i}(q_i) \\in SE(2)$ relative to the parent frame $p$; composing them gives the configuration of every body in the spatial frame $\\{s\\}$:',
    );
    b.math(
      String.raw`g_{s i} = g_{s p}\, g_{p i},\qquad g = \begin{bmatrix} R(\varphi) & p \\ 0 & 1 \end{bmatrix}`,
    );
    for (const body of d.bodies) {
      const p = body.parent;
      b.math(String.raw`g_{${p}${body.label}} = ${texMat(body.gLocal)}`);
      if (p !== 's')
        b.math(String.raw`g_{s${body.label}} = g_{s${p}}\,g_{${p}${body.label}} = ${texMat(body.gWorld)}`);
      b.math(String.raw`\varphi_{${body.label}} = ${body.phi}`);
    }
  });

  step('Centre-of-mass positions & velocities', undefined, (b) => {
    b.text(
      'The centre of mass of body $i$ is $p_i = g_{si}\\,(r_i, 0, 1)^T$. Differentiating along a motion, $\\dot p_i = \\sum_j \\frac{\\partial p_i}{\\partial q_j}\\dot q_j$; the angular velocity is $\\omega_i = \\dot\\varphi_i$.',
    );
    for (const body of d.bodies) {
      b.math(
        String.raw`p_{${body.label}} = ${col(body.com)},\quad v_{${body.label}} = \dot p_{${body.label}} = ${col(body.vcom)},\quad \omega_{${body.label}} = ${body.omega}`,
      );
    }
  });

  step('Kinetic energy', 'MLS (4.10)', (b) => {
    b.text('Each body contributes translational and rotational kinetic energy (about its centre of mass):');
    b.math(String.raw`T_i = \tfrac12 m_i\,v_i^{T} v_i + \tfrac12 I_i\,\omega_i^2`);
    for (const body of d.bodies) b.math(String.raw`T_{${body.label}} = ${body.T}`);
    b.math(String.raw`T = \sum_i T_i = ${d.T}`);
  });

  step('Potential energy', undefined, (b) => {
    b.text('Gravity acts on each centre of mass: $V_i = m_i\\,g\\,y_i$.');
    for (const body of d.bodies) b.math(String.raw`V_{${body.label}} = ${body.V}`);
    b.math(String.raw`V = \sum_i V_i = ${d.V}`);
  });

  step('Inertia matrix', 'MLS (4.20)', (b) => {
    b.text(
      'The kinetic energy is a quadratic form in the velocities, $T = \\tfrac12 \\dot q^{T} M(q)\\,\\dot q$, so the inertia matrix is its Hessian. It is symmetric and positive definite.',
    );
    b.math(String.raw`M_{ij}(q) = \frac{\partial^2 T}{\partial \dot q_i\,\partial \dot q_j}`);
    b.math(String.raw`${cls('t-m', 'M(q)')} = ${texMat(d.M)}`);
  });

  step('Christoffel symbols & Coriolis matrix', 'MLS (4.22)–(4.23)', (b) => {
    b.text(
      "Substituting $L = T - V$ into Lagrange's equations $\\frac{d}{dt}\\frac{\\partial L}{\\partial \\dot q_i} - \\frac{\\partial L}{\\partial q_i} = \\Upsilon_i$ produces velocity-product terms whose coefficients are the Christoffel symbols of $M$:",
    );
    b.math(
      String.raw`\Gamma_{ijk} = \frac12\left(\frac{\partial M_{ij}}{\partial q_k} + \frac{\partial M_{ik}}{\partial q_j} - \frac{\partial M_{kj}}{\partial q_i}\right),\qquad \Gamma_{ijk} = \Gamma_{ikj}`,
    );
    if (d.gamma.length === 0) {
      b.text(
        'All Christoffel symbols vanish: $M$ does not depend on $q$, so there are no Coriolis or centrifugal forces.',
      );
    } else {
      const rows = d.gamma.map((g) => String.raw`\Gamma_{${g.i}${g.j}${g.k}} &= ${g.tex}`);
      b.math(String.raw`\begin{aligned}${rows.join(String.raw`\\`)}\end{aligned}`, {
        caption: 'non-zero symbols with j ≤ k',
      });
    }
    b.math(String.raw`C_{ij}(q, \dot q) = \sum_k \Gamma_{ijk}\,\dot q_k`);
    b.math(String.raw`${cls('t-c', String.raw`C(q,\dot q)`)} = ${texMat(d.C)}`);
    b.text(
      'With this particular $C$, $\\dot M - 2C$ is skew-symmetric (MLS Lemma 4.2): the Coriolis forces do no work, which is why the energy of the unforced system is conserved.',
    );
  });

  step('Gravity & friction', 'MLS (4.24)', (b) => {
    b.text('Conservative forces come from the potential; viscous joint friction adds $\\beta_i \\dot q_i$:');
    b.math(
      String.raw`${cls('t-n', String.raw`N(q,\dot q)`)} = \frac{\partial V}{\partial q} + \operatorname{diag}(\beta)\,\dot q = ${col(d.N)}`,
    );
  });

  step('Equations of motion', 'MLS (4.24)', (b) => {
    b.math(
      String.raw`${cls('t-m', String.raw`M(q)\,\ddot q`)} + ${cls('t-c', String.raw`C(q,\dot q)\,\dot q`)} + ${cls('t-n', String.raw`N(q,\dot q)`)} = ${cls('t-tau', String.raw`\tau`)}`,
    );
    b.text('Written out row by row for this mechanism:');
    const rows = d.eom.map((e, i) => String.raw`${e} &= \tau_{${i + 1}}`);
    b.math(String.raw`\begin{aligned}${rows.join(String.raw`\\`)}\end{aligned}`);
    if (d.nu === n) {
      b.text('Every joint is actuated ($\\tau = u$): the system is <b>fully actuated</b>.');
    } else if (d.nu === 0) {
      b.text('No joint is actuated ($\\tau = 0$): the motion is purely passive.');
    } else {
      const passive = d.coords.filter((c) => !c.actuated).map((c) => `$${c.tex}$`);
      b.math(String.raw`\tau = B\,u,\qquad B = ${texMat(d.B)}`);
      b.text(
        `Only ${d.nu} of ${n} joints are actuated: the system is <b>underactuated</b>. The rows of ${passive.join(', ')} have no input: they can only be influenced through the inertial coupling in $M$.`,
      );
    }
  });

  if (d.closures.length) {
    const m = d.phi.length;
    step('Loop-closure constraints', 'MLS §6.1', (b) => {
      b.text(
        'Each loop-closing pin requires two points to coincide. Written in the tree coordinates, this gives <b>holonomic constraints</b> $\\varphi(q) = 0$:',
      );
      d.closures.forEach((c, k) => {
        const where = (body: string, frac: number) =>
          body === 's' ? String.raw`p^{s}` : String.raw`p_{${body}}(${frac === 1 ? 'l' : `${frac},l`})`;
        b.math(String.raw`${where(c.a, c.sa)} = ${col(c.pa)},\quad ${where(c.b, c.sb)} = ${col(c.pb)}`, {
          caption: `pin ${k + 1}: ${c.a === 's' ? 'ground' : `body ${c.a}`} ↔ body ${c.b}`,
        });
      });
      b.math(String.raw`\varphi(q) = p_a - p_b = ${col(d.phi)} = 0`);
      b.text(
        'Differentiating, admissible velocities satisfy $A(q)\\,\\dot q = 0$ with the constraint Jacobian',
      );
      b.math(String.raw`A(q) = \frac{\partial \varphi}{\partial q} = ${texMat(d.A)}`);
      b.text(
        `The constraints act through forces $-A^T\\lambda$ that do no work along admissible motions. With $m = ${m}$ Lagrange multipliers $\\lambda$, the equations of motion become a differential-algebraic system:`,
      );
      b.math(
        String.raw`M(q)\,\ddot q + C(q,\dot q)\,\dot q + N(q,\dot q) + A^{T}(q)\,\lambda = \tau,\qquad A(q)\,\ddot q + \dot A\,\dot q = 0`,
      );
      b.math(String.raw`\dot A\,\dot q = ${col(d.adot)}`);
      b.text(
        'The simulation solves for $\\ddot q$ and $\\lambda$ together and adds Baumgarte stabilisation, $A\\ddot q + \\dot A \\dot q = -2\\alpha A \\dot q - \\alpha^2 \\varphi$, so numerical drift cannot open the loop:',
      );
      b.math(
        String.raw`\begin{bmatrix} M & A^{T} \\ A & 0 \end{bmatrix} \begin{bmatrix} \ddot q \\ \lambda \end{bmatrix} = \begin{bmatrix} \tau - C\dot q - N \\ -\dot A \dot q - 2\alpha A\dot q - \alpha^{2}\varphi \end{bmatrix}`,
      );
    });
  }

  const tool = d.tool;
  if (tool) {
    step('Tool kinematics & Jacobians', 'MLS (3.3), §3.4', (b) => {
      b.text(
        `The tool is the tip of body ${tool.body}. Its position and the Jacobians map joint velocities to tool velocities:`,
      );
      b.math(String.raw`p_{st}(q) = ${col(tool.p)},\qquad \varphi_{t} = ${tool.phi}`);
      b.math(
        String.raw`\dot p_{st} = \frac{\partial p_{st}}{\partial q}\,\dot q,\qquad \frac{\partial p_{st}}{\partial q} = ${texMat(tool.Jp)}`,
      );
      const xi = d.coords.map((_, k) => texMat(tool.twists.map((r) => [r[k] ?? '0'])));
      b.text(
        'Product of exponentials: each joint contributes a twist $\\xi_i$, written here as $(v_x, v_y, \\omega)$; a revolute joint about $q_i$ has $\\xi = (-\\omega \\times q_i, \\omega)$, a prismatic one $\\xi = (v, 0)$.',
      );
      b.math(
        String.raw`g_{st}(q) = ${d.coords.map((c, k) => String.raw`e^{\hat\xi_{${k + 1}} ${c.tex}}`).join('')}\,g_{st}(0),\quad g_{st}(0) = ${texMat(tool.gst0)},\quad ${xi.map((x, k) => String.raw`\xi_{${k + 1}} = ${x}`).join(', ')}`,
      );
      b.math(String.raw`J^{s}_{st}(q) = ${texMat(tool.Js)}`, { caption: 'spatial manipulator Jacobian' });
    });
  }

  step('Notation', undefined, (b) => {
    if (d.trig.length) {
      b.math(
        String.raw`\begin{aligned}${d.trig.map(([s, m]) => String.raw`${s} &= ${m}`).join(String.raw`\\`)}\end{aligned}`,
      );
    }
    b.text(
      'Coordinates: $\\theta_i$ for revolute joints, $d_i$ for prismatic ones. Coefficients are kept symbolic throughout; their numeric values only enter the simulation.',
    );
  });

  return steps;
}
