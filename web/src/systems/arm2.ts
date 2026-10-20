import * as THREE from 'three/webgpu';
import { arm2 as model } from '../generated';
import { Arrow, dashedCircle, glow, matte, metal, TorqueArc, Trail, triad } from '../scene/primitives';
import { css, palette } from '../scene/theme';
import type { SystemDef } from './types';

const MODE_PASSIVE = 0;
const MODE_PD = 1;
const MODE_PDG = 2;
const MODE_CT = 3;

export const arm2System: SystemDef = {
  id: 1,
  key: 'arm2',
  title: 'Two-link arm',
  model,
  q: String.raw`\theta`,
  qNames: ['θ₁', 'θ₂'],
  view: { cx: 0, cy: 0, width: 4.8, height: 4.8 },
  modes: [
    {
      id: MODE_PASSIVE,
      label: 'Passive',
      hint: 'No controller: gravity and friction only. Apply joint torques with A D (joint 1) and ← → (joint 2).',
    },
    {
      id: MODE_PD,
      label: 'PD',
      hint: 'τ = Kp e + Kv ė. Gravity makes it sag: there is a steady-state error.',
    },
    {
      id: MODE_PDG,
      label: 'PD + gravity',
      hint: 'Adding ∂V/∂θ cancels gravity: zero steady-state error, but the transient still depends on M(θ).',
    },
    {
      id: MODE_CT,
      label: 'Computed torque',
      hint: 'Feedback linearisation: the error dynamics become linear and identical everywhere.',
    },
  ],
  defaultMode: MODE_CT,
  params: [
    { key: 'm1', tex: 'm_1', unit: 'kg', min: 0.1, max: 5, value: 1, group: 'physical', hint: 'link 1 mass' },
    { key: 'm2', tex: 'm_2', unit: 'kg', min: 0.1, max: 5, value: 1, group: 'physical', hint: 'link 2 mass' },
    {
      key: 'l1',
      tex: 'l_1',
      unit: 'm',
      min: 0.3,
      max: 1.5,
      value: 1,
      group: 'physical',
      hint: 'uniform link: r = l/2, I_z = m l²/12',
    },
    { key: 'l2', tex: 'l_2', unit: 'm', min: 0.3, max: 1.5, value: 1, group: 'physical' },
    { key: 'g', tex: 'g', unit: 'm/s²', min: 0, max: 20, value: 9.81, group: 'physical' },
    {
      key: 'b1',
      tex: String.raw`\beta_1`,
      unit: 'N·m·s',
      min: 0,
      max: 1,
      value: 0.05,
      group: 'physical',
      hint: 'joint friction',
    },
    { key: 'b2', tex: String.raw`\beta_2`, unit: 'N·m·s', min: 0, max: 1, value: 0.05, group: 'physical' },
    {
      key: 'umax',
      tex: String.raw`\tau_{\max}`,
      unit: 'N·m',
      min: 5,
      max: 200,
      value: 80,
      group: 'control',
      hint: 'actuator saturation',
    },
    {
      key: 'push',
      tex: String.raw`\tau_{\text{key}}`,
      unit: 'N·m',
      min: 1,
      max: 60,
      value: 20,
      group: 'control',
      hint: 'torque while holding keys',
    },
    {
      key: 'refwn',
      tex: String.raw`\omega_{\text{ref}}`,
      unit: 'rad/s',
      min: 0.5,
      max: 15,
      value: 4,
      group: 'control',
      hint: 'speed of the reference filter',
    },
    {
      key: 'ctwn',
      tex: String.raw`\omega_{c}`,
      unit: 'rad/s',
      min: 2,
      max: 40,
      value: 12,
      group: 'control',
      hint: 'computed torque: K_p = ω², K_v = 2ω',
    },
    {
      key: 'elbow',
      tex: String.raw`\text{elbow}`,
      unit: '±',
      min: -1,
      max: 1,
      value: 1,
      step: 2,
      group: 'control',
      hint: 'inverse-kinematics branch (E toggles)',
    },
  ],
  keys: [
    { keys: 'Click / drag', action: 'move the target' },
    { keys: 'A D', action: 'torque on joint 1' },
    { keys: '← →', action: 'torque on joint 2' },
    { keys: 'E', action: 'flip elbow' },
    { keys: 'Space', action: 'random kick' },
  ],

  modelParams: (u) => [
    u.m1!,
    u.m2!,
    u.l1!,
    u.l2!,
    u.l1! / 2,
    u.l2! / 2,
    (u.m1! * u.l1! * u.l1!) / 12,
    (u.m2! * u.l2! * u.l2!) / 12,
    u.g!,
    u.b1!,
    u.b2!,
  ],
  flags: (u) => [
    ['Umax', u.umax!],
    ['RefWn', u.refwn!],
    ['CtWn', u.ctwn!],
    ['Elbow', u.elbow!],
  ],

  onKey(e, down, api) {
    const k = e.key.toLowerCase();
    const joint = k === 'a' || k === 'd' ? 0 : k === 'arrowleft' || k === 'arrowright' ? 1 : -1;
    if (joint >= 0) {
      const dir = k === 'a' || k === 'arrowleft' ? 1 : -1; // left = counter-clockwise
      api.input(joint, down ? dir * api.ui.push! : 0);
      return true;
    }
    if (!down) return false;
    if (k === 'e') {
      api.ui.elbow = -(api.ui.elbow ?? 1);
      api.flag('Elbow', api.ui.elbow);
      return true;
    }
    if (k === ' ') {
      api.kick(0, (Math.random() - 0.5) * 6);
      api.kick(1, (Math.random() - 0.5) * 10);
      return true;
    }
    return false;
  },

  sections({ snap }, api) {
    const m = () => api.mode();
    const err = (i: number) => snap.v('Q_REF', i) - snap.v('Q', i);
    return [
      {
        id: 'arm-ik',
        title: 'Inverse kinematics',
        cite: 'MLS §3.3',
        build(b) {
          b.text(
            'Given a tool position $p = (x, y)$, the law of cosines gives the elbow angle; the sign picks the elbow branch:',
          );
          b.math(String.raw`\cos\theta_2 = \frac{x^2 + y^2 - l_1^2 - l_2^2}{2 l_1 l_2},\qquad
            \theta_1 = \operatorname{atan2}(y, x) - \operatorname{atan2}(l_2 s_2,\ l_1 + l_2 c_2)`);
          b.math(
            String.raw`p^{*} = ${b.live.vec(2, (i) => snap.v('TARGET', i))}\ \mapsto\ \theta^{*} = ${b.live.vec(2, (i) => snap.v('Q_GOAL', i))},\quad
            \text{reachable: } ${b.live.num(
              () => snap.v('REACHABLE'),
              (v) => (v ? 'yes' : 'no — projected to the boundary'),
            )}`,
          );
          b.text(
            'The goal is smoothed by a critically damped filter $\\ddot\\theta_d = \\omega_{\\text{ref}}^2(\\theta^{*} - \\theta_d) - 2\\omega_{\\text{ref}}\\dot\\theta_d$, which provides $\\theta_d, \\dot\\theta_d, \\ddot\\theta_d$ (the ghost arm).',
          );
        },
      },
      {
        id: 'arm-control',
        title: 'Joint-space control',
        cite: 'MLS §4.5',
        build(b) {
          b.math(
            String.raw`e = \theta_d - \theta = ${b.live.vec(2, err)},\quad
            \dot e = ${b.live.vec(2, (i) => snap.v('QD_REF', i) - snap.v('QD', i))},\quad
            u = ${b.live.vec(2, (i) => snap.v('U', i))}`,
          );
          b.math(String.raw`\textbf{PD:}\quad \tau = K_p e + K_v \dot e`, { active: () => m() === MODE_PD });
          b.math(
            String.raw`\textbf{PD + gravity:}\quad \tau = K_p e + K_v \dot e + \frac{\partial V}{\partial \theta}(\theta)`,
            {
              active: () => m() === MODE_PDG,
            },
          );
          b.math(
            String.raw`\textbf{Computed torque:}\quad \tau = M(\theta)\big(\ddot\theta_d + K_v\dot e + K_p e\big) + C(\theta,\dot\theta)\dot\theta + N(\theta,\dot\theta)`,
            { active: () => m() === MODE_CT },
          );
          b.text(
            'Substituting computed torque into the equations of motion and cancelling $M(\\theta) \\succ 0$ gives linear, decoupled error dynamics, exponentially stable for $K_p, K_v \\succ 0$:',
          );
          b.math(
            String.raw`\ddot e + K_v \dot e + K_p e = 0,\qquad K_p = \omega_c^2 I,\ K_v = 2\omega_c I,\quad
            \omega_c = ${b.live.num(() => api.ui.ctwn!)}\ \mathrm{rad/s}`,
            { active: () => m() === MODE_CT },
          );
        },
      },
    ];
  },

  plots(api) {
    const s = api.snap;
    return [
      {
        title: 'Joint angles',
        unit: 'rad',
        series: [
          { label: 'θ₁', color: css(palette.accent), get: () => s.v('Q', 0) },
          { label: 'θ₂', color: css(palette.input), get: () => s.v('Q', 1) },
          { label: 'θd₁', color: css(palette.accent), get: () => s.v('Q_REF', 0), dash: [4, 4] },
          { label: 'θd₂', color: css(palette.input), get: () => s.v('Q_REF', 1), dash: [4, 4] },
        ],
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
        title: 'Joint 1 torque balance',
        unit: 'N·m',
        series: [
          { label: 'Mθ̈', color: css(palette.inertial), get: () => s.v('MQDD', 0) },
          { label: 'Cθ̇', color: css(palette.coriolis), get: () => s.v('CQD', 0) },
          { label: 'N', color: css(palette.gravity), get: () => s.v('N', 0) },
          { label: 'τ', color: css(palette.input), get: () => s.v('TAU', 0), dash: [4, 4] },
        ],
      },
    ];
  },

  scene(stage, api) {
    const root = stage.root;
    const s = api.snap;

    const pedestal = new THREE.Mesh(
      new THREE.CylinderGeometry(0.16, 0.22, 0.12, 40),
      matte(palette.graphite),
    );
    pedestal.rotation.x = Math.PI / 2;
    pedestal.position.z = -0.2;
    root.add(pedestal);

    const linkMat = [metal(0x5b6fa8, 0.35, 0.55), metal(0x7a8fcf, 0.35, 0.55)];
    const ghostMat = glow(palette.reference, 0.18);
    const frame1 = new THREE.Group();
    const frame2 = new THREE.Group();
    const ghost1 = new THREE.Group();
    const ghost2 = new THREE.Group();
    const link1 = new THREE.Mesh(undefined, linkMat[0]);
    const link2 = new THREE.Mesh(undefined, linkMat[1]);
    const glink1 = new THREE.Mesh(undefined, ghostMat);
    const glink2 = new THREE.Mesh(undefined, ghostMat);
    frame1.add(link1);
    frame2.add(link2);
    ghost1.add(glink1);
    ghost2.add(glink2);
    for (const [g, z] of [
      [ghost1, -0.3],
      [ghost2, -0.3],
    ] as const) {
      g.position.z = z;
    }
    root.add(frame1, frame2, ghost1, ghost2);

    const jointGeo = new THREE.CylinderGeometry(0.075, 0.075, 0.2, 32);
    const jointMat = metal(palette.steel, 0.2, 0.9);
    const joint1 = new THREE.Mesh(jointGeo, jointMat);
    const joint2 = new THREE.Mesh(jointGeo, jointMat);
    for (const j of [joint1, joint2]) {
      j.rotation.x = Math.PI / 2;
      j.position.z = 0.05;
    }
    frame1.add(joint1);
    frame2.add(joint2);
    const toolBall = new THREE.Mesh(new THREE.SphereGeometry(0.055, 24, 16), metal(0xe6ebf5, 0.25, 0.7));
    root.add(toolBall);

    const coms = [0, 1].map(
      () => new THREE.Mesh(new THREE.SphereGeometry(0.03, 16, 12), glow(palette.gravity)),
    );
    for (const c of coms) root.add(c);

    const target = new THREE.Group();
    const ring = new THREE.Mesh(new THREE.TorusGeometry(0.09, 0.008, 8, 48), glow(palette.reference));
    const dot = new THREE.Mesh(new THREE.CircleGeometry(0.018, 24), glow(palette.reference));
    target.add(ring, dot);
    target.position.z = 0.4;
    root.add(target);

    const outer = dashedCircle(2, 0xaab6d3, 0.22);
    const inner = dashedCircle(0, 0xaab6d3, 0.22);
    root.add(outer, inner);

    const arcs = [new TorqueArc(palette.input, 0.16), new TorqueArc(palette.input, 0.14)];
    for (const a of arcs) root.add(a);

    const trail = new Trail(palette.accent, 600);
    root.add(trail);

    const triads = [triad(), triad(), triad(), triad()];
    for (const t of triads) root.add(t);
    for (const [t, name] of [
      [triads[0], 's'],
      [triads[1], '1'],
      [triads[2], '2'],
      [triads[3], 't'],
    ] as const) {
      stage.label(t!, String.raw`\{${name}\}`, 'frame');
    }

    const gArrow = new Arrow(palette.gravity, 0.01, 0.05);
    gArrow.angle = -Math.PI / 2;
    gArrow.position.set(2.25, -1.55, 0);
    root.add(gArrow);
    stage.label(gArrow, 'g', 'gravity', 10, 12);

    const capsule = (len: number, r: number) => {
      const geo = new THREE.CapsuleGeometry(r, len, 8, 24);
      geo.rotateZ(-Math.PI / 2);
      geo.translate(len / 2, 0, 0);
      return geo;
    };
    let lengths = '';
    const setLengths = (l1: number, l2: number) => {
      const key = `${l1},${l2}`;
      if (key === lengths) return;
      lengths = key;
      for (const mesh of [link1, glink1]) {
        mesh.geometry.dispose();
        mesh.geometry = capsule(l1, 0.055);
      }
      for (const mesh of [link2, glink2]) {
        mesh.geometry.dispose();
        mesh.geometry = capsule(l2, 0.045);
      }
      outer.scale.setScalar((l1 + l2) / 2);
      inner.scale.setScalar(Math.max(Math.abs(l1 - l2), 1e-3));
      inner.visible = Math.abs(l1 - l2) > 0.02;
    };

    let dragging = false;
    return {
      update() {
        const { l1, l2 } = api.ui as { l1: number; l2: number };
        setLengths(l1, l2);
        const f = (k: number, i: number) => s.v('FRAMES', 3 * k + i);
        frame1.position.set(f(1, 0), f(1, 1), 0);
        frame1.rotation.z = f(1, 2);
        frame2.position.set(f(2, 0), f(2, 1), 0.1);
        frame2.rotation.z = f(2, 2);
        toolBall.position.set(f(3, 0), f(3, 1), 0.15);
        coms.forEach((c, i) => {
          c.position.set(s.v('COMS', 2 * i), s.v('COMS', 2 * i + 1), 0.3);
        });

        const show = api.ui.showFrames !== 0;
        triads.forEach((t, k) => {
          t.visible = show;
          t.position.set(f(k, 0), f(k, 1), 0.35);
          t.rotation.z = f(k, 2);
        });

        const qr = [s.v('Q_REF', 0), s.v('Q_REF', 1)] as const;
        const ghostOn = api.mode() !== MODE_PASSIVE;
        ghost1.visible = ghostOn;
        ghost2.visible = ghostOn;
        ghost1.rotation.z = qr[0];
        ghost2.position.set(l1 * Math.cos(qr[0]), l1 * Math.sin(qr[0]), -0.3);
        ghost2.rotation.z = qr[0] + qr[1];

        target.visible = ghostOn;
        target.position.set(s.v('TARGET', 0), s.v('TARGET', 1), 0.4);
        const reach = s.v('REACHABLE') !== 0;
        (ring.material as THREE.MeshBasicMaterial).color.setHex(reach ? palette.reference : palette.warn);

        const umax = s.v('UMAX') || 1;
        arcs[0]!.position.set(f(1, 0), f(1, 1), 0.35);
        arcs[0]!.set(s.v('U', 0) / umax, f(1, 2));
        arcs[1]!.position.set(f(2, 0), f(2, 1), 0.35);
        arcs[1]!.set(s.v('U', 1) / umax, f(2, 2));

        gArrow.set(Math.min(0.5, (api.ui.g! / 9.81) * 0.35));
        trail.visible = api.ui.showTrail !== 0;
        trail.push(f(3, 0), f(3, 1));
      },
      pointer(x, y, kind) {
        if (kind === 'down') dragging = true;
        if (kind === 'up') dragging = false;
        if (!dragging && kind !== 'down') return;
        api.flag('TargetX', x);
        api.flag('TargetY', y);
      },
      reset() {
        trail.reset();
      },
    };
  },
};
