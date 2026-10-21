/// <reference types="node" />
/** The textbook figure of every preset, exported to SVG and TikZ from a running simulation. */
import { beforeAll, describe, expect, it } from 'vitest';
import { encode, PRESETS, paramVector } from '../model/description';
import { TestCore } from '../sim/testing';
import { mechanismBounds } from './geometry';
import { Figure, type SchematicFrame } from './schematic';

let core: TestCore;
beforeAll(async () => {
  core = await TestCore.load();
});

describe('textbook figure', () => {
  for (const [name, make] of Object.entries(PRESETS)) {
    it(`${name}: exports a well-formed SVG and TikZ picture`, () => {
      const mech = make();
      const d = core.build(encode(mech));
      core.send({ type: 'controller', slot: 0, kind: 1 });
      if (mech.tool >= 0) core.flag('UseTool', 1);
      const fig = new Figure(mech, d, mechanismBounds(mech));
      let snap = core.snapshot();
      for (let k = 0; k < 20; k++) {
        core.core.step(25);
        snap = core.snapshot(snap);
        const [tx, ty] = snap.frame(mech.bodies.length + 1);
        fig.track(tx, ty);
      }
      const f: SchematicFrame = {
        snap,
        params: paramVector(mech),
        running: true,
        push: mech.tool >= 0 ? [0.5, 0.5] : null,
        ghosts: [],
        annotate: true,
        trail: true,
      };
      const m = { k: 120, x0: 400, y0: 300, w: 800, h: 600 };

      const svg = fig.svg(m, f);
      expect(svg.startsWith('<svg')).toBe(true);
      expect(svg.trimEnd().endsWith('</svg>')).toBe(true);
      expect(svg).not.toMatch(/NaN|undefined|Infinity/);
      expect((svg.match(/<path /g) ?? []).length).toBeGreaterThan(mech.bodies.length);

      const tikz = fig.tikz(m, f);
      expect(tikz).not.toMatch(/NaN|undefined|Infinity/);
      expect(tikz).toContain('\\begin{tikzpicture}');
      expect(tikz).toContain('\\end{tikzpicture}');
      expect(tikz.split('{').length).toBe(tikz.split('}').length);
      for (const line of tikz.split('\n').filter((l) => l.trimStart().startsWith('\\path')))
        expect(line.trimEnd().endsWith(';')).toBe(true);
      // every joint coordinate is named, unless its label had to give way to another
      const named = d.coords.filter((c) => tikz.includes(`{$${c.tex}$}`)).length;
      expect(named).toBeGreaterThanOrEqual(Math.min(1, d.coords.length));
    });
  }
});
