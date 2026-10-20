import * as THREE from 'three/webgpu';
import { cartpole as model } from '../generated';
import { Arrow, glow, matte, metal, Trail, triad } from '../scene/primitives';
import { css, palette } from '../scene/theme';
import type { SystemDef } from './types';

const MODE_MANUAL = 0;
const MODE_LQR = 1;
const MODE_SWING = 2;
const PHASE_SWING = 1;
const PHASE_BALANCE = 2;
const RAIL = 2.4;

export const cartpoleSystem: SystemDef = {
  id: 0,
  key: 'cartpole',
  title: 'Cart-pole',
  model,
  q: 'q',
  qNames: ['x', 'θ'],
  view: { cx: 0, cy: 0.45, width: 5.9, height: 2.7 },
  modes: [
    {
      id: MODE_MANUAL,
      label: 'Manual',
      hint: 'Balance it yourself: push the cart with ← → (or A D), or press and hold beside the cart.',
    },
    {
      id: MODE_LQR,
      label: 'LQR balance',
      hint: 'Linear-quadratic regulator about the upright. Click the rail to move the set-point.',
    },
    {
      id: MODE_SWING,
      label: 'Swing-up',
      hint: 'Energy pumping brings the pole up, then LQR catches it. Press B to drop the pole.',
    },
  ],
  defaultMode: MODE_LQR,
  params: [
    { key: 'mc', tex: 'm_c', unit: 'kg', min: 0.2, max: 5, value: 1, group: 'physical', hint: 'cart mass' },
    {
      key: 'mp',
      tex: 'm_p',
      unit: 'kg',
      min: 0.05,
      max: 2,
      value: 0.3,
      group: 'physical',
      hint: 'pole mass',
    },
    {
      key: 'L',
      tex: 'L',
      unit: 'm',
      min: 0.3,
      max: 2,
      value: 1,
      group: 'physical',
      hint: 'pole length (uniform rod: r = L/2, I_p = m_p L²/12)',
    },
    { key: 'g', tex: 'g', unit: 'm/s²', min: 0, max: 20, value: 9.81, group: 'physical' },
    {
      key: 'bx',
      tex: String.raw`\beta_x`,
      unit: 'N·s/m',
      min: 0,
      max: 3,
      value: 0.1,
      group: 'physical',
      hint: 'cart friction',
    },
    {
      key: 'bth',
      tex: String.raw`\beta_\theta`,
      unit: 'N·m·s',
      min: 0,
      max: 0.2,
      value: 0.002,
      step: 0.001,
      group: 'physical',
      hint: 'pivot friction',
    },
    {
      key: 'umax',
      tex: String.raw`u_{\max}`,
      unit: 'N',
      min: 5,
      max: 100,
      value: 40,
      group: 'control',
      hint: 'actuator saturation',
    },
    {
      key: 'push',
      tex: String.raw`u_{\text{key}}`,
      unit: 'N',
      min: 1,
      max: 50,
      value: 12,
      group: 'control',
      hint: 'force while holding ← →',
    },
    {
      key: 'qx',
      tex: 'Q_x',
      unit: '',
      min: 0.1,
      max: 1000,
      value: 10,
      log: true,
      group: 'control',
      hint: 'LQR weight on cart position',
    },
    {
      key: 'qth',
      tex: String.raw`Q_\theta`,
      unit: '',
      min: 1,
      max: 10000,
      value: 100,
      log: true,
      group: 'control',
      hint: 'LQR weight on pole angle',
    },
    {
      key: 'r',
      tex: 'R',
      unit: '',
      min: 0.001,
      max: 10,
      value: 0.1,
      log: true,
      group: 'control',
      hint: 'LQR weight on effort',
    },
  ],
  keys: [
    { keys: '← →  /  A D', action: 'push the cart' },
    { keys: 'Space', action: 'knock the pole' },
    { keys: 'B', action: 'drop the pole to the bottom' },
  ],

  modelParams: (u) => [u.mc!, u.mp!, u.L! / 2, (u.mp! * u.L! * u.L!) / 12, u.g!, u.bx!, u.bth!],
  flags: (u) => [
    ['Umax', u.umax!],
    ['LqrQx', u.qx!],
    ['LqrQth', u.qth!],
    ['LqrR', u.r!],
    ['Rail', RAIL],
  ],

  onKey(e, down, api) {
    const k = e.key.toLowerCase();
    if (k === 'arrowleft' || k === 'a' || k === 'arrowright' || k === 'd') {
      const dir = k === 'arrowleft' || k === 'a' ? -1 : 1;
      api.input(0, down ? dir * api.ui.push! : 0);
      return true;
    }
    if (!down) return false;
    if (k === ' ') {
      api.kick(1, (Math.random() < 0.5 ? -1 : 1) * 1.6);
      return true;
    }
    if (k === 'b') {
      api.state(1, Math.PI - 0.02, 0);
      api.state(0, api.snap.v('Q', 0), 0);
      return true;
    }
    return false;
  },

  sections({ snap }, api) {
    const swinging = () => api.mode() === MODE_SWING && snap.v('PHASE') === PHASE_SWING;
    const balancing = () =>
      api.mode() === MODE_LQR || (api.mode() === MODE_SWING && snap.v('PHASE') === PHASE_BALANCE);
    return [
      {
        id: 'cp-underactuation',
        title: 'Underactuation',
        build(b) {
          b.text(
            'Only the cart is driven: $B = (1, 0)^{T}$ has rank $1 < n = 2$. The pole can only be influenced through the inertial coupling $M_{12} = -m_p r\\cos\\theta$, which vanishes when the pole is horizontal.',
          );
          b.math(
            String.raw`\begin{aligned}
            (m_c + m_p)\,\ddot x - m_p r\cos\theta\,\ddot\theta + m_p r \sin\theta\,\dot\theta^2 + \beta_x \dot x &= u\\
            -m_p r\cos\theta\,\ddot x + (I_p + m_p r^2)\,\ddot\theta - m_p g r \sin\theta + \beta_\theta\dot\theta &= 0
            \end{aligned}`,
          );
        },
      },
      {
        id: 'cp-lqr',
        title: 'Linearisation & LQR',
        cite: 'about θ = 0',
        build(b) {
          b.text(
            'Around the upright equilibrium $x = (x, \\theta, \\dot x, \\dot\\theta) = 0$, $C\\dot q$ is second order and drops out, leaving $\\dot x = A x + B u$:',
          );
          b.math(String.raw`A = \begin{bmatrix} 0 & I \\ -M_0^{-1}\frac{\partial N}{\partial q} & -M_0^{-1}\operatorname{diag}\beta \end{bmatrix}
            = ${b.live.mat(4, 4, (i, j) => snap.m('LIN_A', i, j, 4))},\quad
            B = \begin{bmatrix} 0 \\ M_0^{-1}B_u \end{bmatrix} = ${b.live.vec(4, (i) => snap.v('LIN_B', i))}`);
          b.text(
            'The gain minimises $\\int_0^\\infty x^T Q x + u^T R u\\,dt$ (solved as a discrete Riccati equation at the 1 kHz control rate by the doubling algorithm):',
          );
          b.math(
            String.raw`u = -K\,(x - x^{*}),\quad K = ${b.live.mat(1, 4, (_i, j) => snap.v('LQR_K', j))},\quad
            x^{*} = (${b.live.num(() => snap.v('X_REF'))}, 0, 0, 0)`,
            { active: balancing },
          );
        },
      },
      {
        id: 'cp-swing',
        title: 'Energy-based swing-up',
        cite: 'Åström & Furuta',
        build(b) {
          b.text(
            'Pole energy relative to the upright orbit, with $J = I_p + m_p r^2$ and cart acceleration $a = \\ddot x$:',
          );
          b.math(
            String.raw`E_p = \tfrac12 J\dot\theta^2 + m_p g r(\cos\theta - 1) = ${b.live.num(() => snap.v('E_POLE'))}\,\mathrm{J},\qquad
            \dot E_p = m_p r\, a\,\dot\theta\cos\theta`,
          );
          b.math(
            String.raw`a = -k_E\,\frac{E_p}{2 m_p g r}\,\dot\theta\cos\theta - k_x x - k_v \dot x
            \ \Rightarrow\ \dot E_p = -\frac{k_E}{2g}\,E_p\,(\dot\theta\cos\theta)^2 \ \ (\text{centring terms aside})`,
            { active: swinging },
          );
          b.text(
            'The desired acceleration becomes a cart force by collocated partial feedback linearisation, with $h = C\\dot q + N$:',
          );
          b.math(
            String.raw`u = \Big(M_{11} - \frac{M_{12}M_{21}}{M_{22}}\Big)a + h_1 - \frac{M_{12}}{M_{22}}h_2`,
            { active: swinging },
          );
        },
      },
    ];
  },

  plots(api) {
    const s = api.snap;
    return [
      {
        title: 'Configuration',
        unit: 'm, rad',
        series: [
          { label: 'x', color: css(palette.accent), get: () => s.v('Q', 0) },
          { label: 'θ', color: css(palette.input), get: () => s.v('Q', 1) },
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
        title: 'Pole row (θ) of the EoM',
        unit: 'N·m',
        series: [
          { label: 'Mq̈', color: css(palette.inertial), get: () => s.v('MQDD', 1) },
          { label: 'Cq̇', color: css(palette.coriolis), get: () => s.v('CQD', 1) },
          { label: 'N', color: css(palette.gravity), get: () => s.v('N', 1) },
          { label: 'u (cart)', color: css(palette.input), get: () => s.v('U', 0), dash: [4, 4] },
        ],
      },
    ];
  },

  scene(stage, api) {
    const root = stage.root;
    const s = api.snap;

    const railLen = 2 * RAIL + 0.36;
    const rail = new THREE.Mesh(new THREE.BoxGeometry(railLen, 0.035, 0.08), metal(palette.steel, 0.25, 0.8));
    rail.position.y = -0.16;
    const base = new THREE.Mesh(new THREE.BoxGeometry(railLen + 0.2, 0.05, 0.3), matte(palette.graphite));
    base.position.set(0, -0.2, -0.12);
    root.add(rail, base);
    for (const sgn of [-1, 1]) {
      const stop = new THREE.Mesh(new THREE.BoxGeometry(0.05, 0.16, 0.14), metal(palette.warn, 0.5, 0.2));
      stop.position.set(sgn * (RAIL + 0.205), -0.1, 0);
      root.add(stop);
    }

    const cart = new THREE.Group();
    const body = new THREE.Mesh(new THREE.BoxGeometry(0.36, 0.16, 0.22), metal(0x3b4a6b, 0.4, 0.5));
    body.position.y = -0.03;
    cart.add(body);
    for (const sx of [-0.11, 0.11]) {
      const wheel = new THREE.Mesh(new THREE.CylinderGeometry(0.04, 0.04, 0.24, 24), matte(0x11151e));
      wheel.rotation.x = Math.PI / 2;
      wheel.position.set(sx, -0.12, 0);
      cart.add(wheel);
    }
    const pivot = new THREE.Mesh(
      new THREE.CylinderGeometry(0.035, 0.035, 0.3, 24),
      metal(palette.steel, 0.2, 0.9),
    );
    pivot.rotation.x = Math.PI / 2;
    cart.add(pivot);
    root.add(cart);

    const poleFrame = new THREE.Group();
    const pole = new THREE.Mesh(new THREE.CylinderGeometry(0.022, 0.022, 1, 24), metal(0xe6ebf5, 0.3, 0.6));
    pole.position.z = 0.12;
    const com = new THREE.Mesh(new THREE.SphereGeometry(0.035, 24, 16), glow(palette.gravity));
    com.position.z = 0.16;
    const tip = new THREE.Mesh(new THREE.SphereGeometry(0.03, 24, 16), metal(0xe6ebf5, 0.3, 0.6));
    tip.position.z = 0.12;
    poleFrame.add(pole, com, tip);
    root.add(poleFrame);

    const force = new Arrow(palette.input, 0.016, 0.07);
    force.position.z = 0.4;
    root.add(force);

    const refMarker = new THREE.Mesh(new THREE.ConeGeometry(0.04, 0.08, 3), glow(palette.reference, 0.9));
    refMarker.rotation.z = Math.PI;
    root.add(refMarker);

    const trail = new Trail(palette.input, 500);
    root.add(trail);

    const frameS = triad();
    const frameC = triad();
    const frameP = triad();
    root.add(frameS, frameC, frameP);
    stage.label(frameS, '\\{s\\}', 'frame', -30, 6);
    stage.label(frameC, '\\{c\\}', 'frame', 14, 8);
    stage.label(frameP, '\\{p\\}', 'frame', 12, -30);
    stage.label(com, 'm_p', 'com', 10, -20);

    const gArrow = new Arrow(palette.gravity, 0.01, 0.05);
    gArrow.angle = -Math.PI / 2;
    gArrow.position.set(-RAIL, 1.45, 0);
    root.add(gArrow);
    stage.label(gArrow, 'g', 'gravity', 10, 12);

    let lastL = -1;
    let pushing = false;
    return {
      update() {
        const L = api.ui.L!;
        if (L !== lastL) {
          pole.scale.y = L;
          pole.position.y = L / 2;
          com.position.y = L / 2;
          tip.position.y = L;
          lastL = L;
        }
        const x = s.v('FRAMES', 3);
        const th = s.v('FRAMES', 8);
        cart.position.x = x;
        frameC.position.set(x, 0, 0.3);
        poleFrame.position.set(x, 0, 0);
        poleFrame.rotation.z = th;
        frameP.position.set(x, 0, 0.32);
        frameP.rotation.z = th;
        frameS.visible = api.ui.showFrames !== 0;
        frameC.visible = frameS.visible;
        frameP.visible = frameS.visible;
        trail.visible = api.ui.showTrail !== 0;

        const umax = s.v('UMAX') || 1;
        force.position.set(x + (s.v('U', 0) >= 0 ? 0.2 : -0.2), -0.03, 0.4);
        force.set((s.v('U', 0) / umax) * 0.9);
        refMarker.visible = api.mode() !== MODE_MANUAL;
        refMarker.position.set(s.v('X_REF'), -0.05, 0.3);
        gArrow.set(Math.min(0.5, (api.ui.g! / 9.81) * 0.35));

        trail.push(x - L * Math.sin(th), L * Math.cos(th));
      },
      pointer(px, _py, kind) {
        if (api.mode() === MODE_MANUAL) {
          // press and hold beside the cart to push it towards the pointer (touch friendly)
          if (kind === 'down') pushing = true;
          if (kind === 'up') pushing = false;
          if (pushing) api.input(0, Math.sign(px - s.v('Q', 0)) * api.ui.push!);
          else if (kind === 'up') api.input(0, 0);
          return;
        }
        if (kind === 'up') return;
        api.flag('XRef', Math.max(-RAIL + 0.3, Math.min(RAIL - 0.3, px)));
      },
      reset() {
        trail.reset();
      },
    };
  },
};
