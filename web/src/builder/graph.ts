/** A small, deterministic assembly graph; no layout engine or separate graph state. */
import type { Mechanism } from '../model/description';
import { button, el } from '../ui/widgets';

export function connectionGraph(
  mech: Mechanism,
  selected: number,
  inspect: (kind: 'body' | 'joint', index: number) => void,
): HTMLElement {
  const scroll = el('div', 'connection-graph');
  const graph = el('div', 'graph-canvas');
  const points = new Map<number, [number, number]>();
  let column = 0;
  const visit = (i: number, depth: number): number => {
    const children = mech.bodies.flatMap((b, k) => (b.parent === i ? [k] : []));
    const xs = children.map((k) => visit(k, depth + 1));
    const x = xs.length ? (xs[0]! + xs[xs.length - 1]!) / 2 : 100 + column++ * 180;
    points.set(i, [x, 45 + depth * 140]);
    return x;
  };
  const roots = mech.bodies.flatMap((b, i) => (b.parent < 0 ? [i] : []));
  const xs = roots.map((i) => visit(i, 1));
  points.set(-1, [xs.length ? (xs[0]! + xs[xs.length - 1]!) / 2 : 100, 30]);
  const width = Math.max(220, column * 180 + 20);
  const height = Math.max(100, ...Array.from(points.values(), (p) => p[1] + 40));
  graph.style.width = `${width}px`;
  graph.style.height = `${height}px`;
  const svg = document.createElementNS('http://www.w3.org/2000/svg', 'svg');
  svg.setAttribute('width', String(width));
  svg.setAttribute('height', String(height));
  svg.setAttribute('aria-hidden', 'true');
  graph.append(svg);
  const edge = (a: number, b: number, label?: string) => {
    const [ax, ay] = points.get(a)!;
    const [bx, by] = points.get(b)!;
    const path = document.createElementNS(svg.namespaceURI, 'path');
    path.setAttribute(
      'd',
      label
        ? `M${ax + 60},${ay} Q${width + 20},${(ay + by) / 2} ${bx + 60},${by}`
        : `M${ax},${ay + 20} L${bx},${by - 65} L${bx},${by - 20}`,
    );
    path.setAttribute('class', label ? 'graph-edge cross-edge' : 'graph-edge');
    if (label) {
      const title = document.createElementNS(svg.namespaceURI, 'title');
      title.textContent = label;
      path.append(title);
    }
    svg.append(path);
  };
  const node = (text: string, x: number, y: number, cls: string, action?: () => void) => {
    const n = action ? button(text, action, cls) : el('div', cls, text);
    n.style.left = `${x}px`;
    n.style.top = `${y}px`;
    graph.append(n);
    return n;
  };
  node('Ground · 0 DOF', ...points.get(-1)!, 'graph-node ground-node');
  mech.bodies.forEach((b, i) => {
    const [x, y] = points.get(i)!;
    if (!b.detached) {
      edge(b.parent, i);
      const kind =
        b.joint === 'revolute' ? 'Rotational' : b.joint === 'prismatic' ? 'Translational' : 'Fixed';
      const label = b.joint === 'fixed' ? '0 DOF' : `1 DOF · ${b.actuated ? 'motor' : 'passive'}`;
      const joint = node(`J${i + 1} ${kind}\n${label}`, x, y - 65, 'graph-node joint-node', () =>
        inspect('joint', i),
      );
      joint.dataset.joint = String(i);
      joint.title = `${b.parent < 0 ? 'Ground' : `Body ${b.parent + 1} at ${Math.round(b.mountS * 100)}% of its length`} → Body ${i + 1}. ${label} relative to its parent.`;
    }
    const n = node(
      `Body ${i + 1} · ${b.shape === 'link' ? 'bar' : 'block'}${b.detached ? '\nUnconnected' : ''}`,
      x,
      y,
      `graph-node body-node${selected === i ? ' on' : ''}${b.detached ? ' detached' : ''}`,
      () => inspect('body', i),
    );
    n.dataset.body = String(i);
  });
  const connections = el('div', 'graph-connections');
  mech.closures.forEach((c, i) => {
    const label = `Loop ${i + 1}: ${c.a < 0 ? 'Ground' : `Body ${c.a + 1}`} ↔ Body ${c.b + 1} (${c.kind})`;
    edge(c.a, c.b, label);
    connections.append(el('div', '', label));
  });
  mech.elements.forEach((e, i) => {
    const label = `Spring/damper ${i + 1}: ${e.a < 0 ? 'Ground' : `Body ${e.a + 1}`} ↔ Body ${e.b + 1}`;
    edge(e.a, e.b, label);
    connections.append(el('div', '', label));
  });
  scroll.append(graph);
  return el('div', '', scroll, connections);
}
