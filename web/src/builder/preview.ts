/** Geometry of an incomplete assembly, without pretending it has simulated dynamics. */
import { coordIndex, dof, type Mechanism, normalise } from '../model/description';
import { compose } from '../scene/geometry';
import { initialFrames } from '../scene/kinematics';
import { type Layout, Snapshot } from '../sim/layout';

export function preview(mech: Mechanism, layout: Layout): Snapshot {
  const snap = new Snapshot(layout);
  const frames = initialFrames(mech);
  frames.forEach((frame, i) => {
    snap.data.set(frame, layout.off.FRAMES + 3 * (i + 1));
  });
  const coordinates = coordIndex(mech);
  mech.bodies.forEach((b, i) => {
    if (coordinates[i]! >= 0) snap.data[layout.off.Q + coordinates[i]!] = b.q0;
  });
  snap.data[layout.off.NQ] = dof(mech);
  if (mech.tool >= 0) {
    const tip = compose(frames[mech.tool]!, [normalise(mech.bodies[mech.tool]!).l, 0, 0]);
    snap.data.set(tip, layout.off.FRAMES + 3 * (mech.bodies.length + 1));
    snap.data.set(tip.slice(0, 2), layout.off.TOOL_P);
  }
  return snap;
}
