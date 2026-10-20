/** Page of the energy-based swing-up for underactuated mechanisms. */
import { idleNote, type Page, type PageContext } from './page';

export const PHASE = { IDLE: 0, PUMPING: 1, CATCHING: 2 } as const;

export const swingUp = (c: PageContext): Page => {
  const { loops } = c;
  return {
    lead: 'Pump energy through the actuated joints until the passive ones reach the target orbit, then catch with LQR.',
    cite: 'Åström & Furuta 2000; Spong 1995',
    idea(b) {
      b.text(
        'An underactuated mechanism cannot be pushed straight to an inverted pose, but its energy can be steered: moving the actuated joints at the right moments does work on the passive ones, like pumping a swing. Once the energy matches that of the target, the motion passes close to it and a local controller takes over.',
      );
    },
    generic(b) {
      b.math(
        loops
          ? String.raw`E = T + V - V(q^{*})`
          : String.raw`E_u = \tfrac12\dot q_u^{T}M_{uu}\dot q_u + V(q) - V(q^{*})`,
        {
          caption: loops
            ? 'energy relative to the target'
            : 'energy of the passive motion relative to the target',
        },
      );
      b.text(
        'An actuated acceleration v = q̈<sub>a</sub> feeds the power −q̇<sub>u</sub><sup>T</sup>M<sub>ua</sub>v into the passive coordinates. Choosing v along w = −M<sub>ua</sub><sup>T</sup>q̇<sub>u</sub> with the sign of −E drives E to zero; a PD term keeps the actuated joints near q*:',
      );
      b.math(
        String.raw`v = k_E\,\frac{-E}{E_{\text{ref}}}\,\frac{w}{\|M_{ua}\|} - k_c\,(q_a - q_a^{*}) - k_v\,\dot q_a,\qquad |v| \le a_{\max}`,
      );
      b.text(
        'Collocated partial feedback linearisation turns v into inputs: solve the equations of motion for the passive accelerations and the inputs, with q̈<sub>a</sub> = v imposed:',
      );
      b.math(
        loops
          ? String.raw`M\ddot q + h + A^{T}\lambda = B u,\quad A\ddot q + \dot A\dot q = 0,\quad \ddot q_a = v\ \Rightarrow\ (\ddot q_u, \lambda, u)`
          : String.raw`\ddot q_u = -M_{uu}^{-1}\big(h_u + M_{ua}\,v\big),\qquad \tau_a = M_{aa}\,v + M_{au}\,\ddot q_u + h_a`,
      );
      b.text(
        'Close to q* the LQR (with its default weights) takes over, and hands back if the state leaves its region of attraction.',
      );
    },
    concrete(b) {
      b.math(
        String.raw`E = ${b.live.num(() => c.tele('energy'))}\,\mathrm{J},\quad E_{\text{ref}} = ${b.live.num(() => c.tele('energy_ref'))}\,\mathrm{J},\quad \text{phase: }${b.live.num(
          () => c.snap.slotOf(c.spec.id)?.v('PHASE') ?? Number.NaN,
          (v) => (v === PHASE.PUMPING ? 'pumping' : v === PHASE.CATCHING ? 'LQR catch' : 'idle'),
        )}`,
      );
      idleNote(b, c);
    },
    stability(b) {
      b.text(
        'Along the pumping law Ė has the sign of −E (until a<sub>max</sub> saturates), so the energy converges to the target level; the homoclinic orbit through q* is reached, and the LQR region of attraction is entered on some pass. The switch itself is a hybrid system: a hysteresis between “near” and “far” prevents chattering.',
      );
    },
    limits(b) {
      b.text(
        'Needs the target to be an energy level the passive motion can reach, and the actuated joints to have room (end stops!). The catch only succeeds when the orbit passes through the LQR region of attraction.',
      );
    },
  };
};
