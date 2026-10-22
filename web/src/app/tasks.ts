import { MAX_TASKS } from '../generated/contracts';
import type { Mechanism } from '../model/description';
import type { SimClient } from '../sim/client';
import type { Snapshot } from '../sim/layout';
import type { TaskGoal } from '../sim/tasks';
import { button, card, DEG, el, note, row } from '../ui/widgets';
import { GOAL_W } from './settings';

/** Ordered runtime targets; editing them never changes authored bodies or parameter defaults. */
export function taskList(
  mech: Mechanism,
  snap: Snapshot,
  client: SimClient,
  refresh: () => void,
): HTMLElement {
  const host = card('Additional ordered goals');
  host.append(
    note(
      'These run in order after the enabled tool-orientation and elbow goals. Posture and manipulability follow the whole list. Body points are frame origins.',
    ),
  );
  const commit = (tasks: TaskGoal[], render = false) => {
    client.setTasks(tasks);
    if (render) refresh();
  };
  const here = (frame: number) => snap.frame(frame === -2 ? mech.bodies.length + 1 : frame + 1);
  const patch = (i: number, change: Partial<TaskGoal>, render = false) =>
    commit(
      client.tasks.map((t, k) => (k === i ? { ...t, ...change } : t)),
      render,
    );
  const numeric = (value: number, unit: string, set: (value: number) => void, min?: number) => {
    const box = el('span');
    const input = el('input');
    input.type = 'number';
    input.value = String(Number(value.toPrecision(6)));
    input.step = '.01';
    if (min !== undefined) input.min = String(min);
    input.style.width = '7em';
    input.setAttribute('aria-label', unit);
    input.addEventListener('change', () => {
      const n = Number(input.value);
      if (Number.isFinite(n) && (min === undefined || n >= min)) set(n);
      else input.value = String(value);
    });
    box.append(input, ` ${unit}`);
    return box;
  };
  client.tasks.forEach((task, i) => {
    const box = card(`${i + 1} · ${task.kind === 'point' ? 'point' : 'orientation'}`);
    const frame = el('select');
    const options: [number, string][] = [
      [-1, 'ground frame'],
      ...mech.bodies.map((_, k): [number, string] => [k, `body ${k + 1} frame`]),
    ];
    if (mech.tool >= 0) options.push([-2, 'tool frame']);
    for (const [id, label] of options) {
      const o = el('option');
      o.value = String(id);
      o.textContent = label;
      frame.append(o);
    }
    frame.value = String(task.frame);
    frame.addEventListener('change', () => patch(i, { frame: Number(frame.value) }));
    box.append(row('frame', frame));
    if (task.kind === 'point') {
      box.append(
        row(
          'target x',
          numeric(task.x, 'm', (x) => patch(i, { x })),
        ),
        row(
          'target y',
          numeric(task.y, 'm', (y) => patch(i, { y })),
        ),
      );
    } else
      box.append(
        row(
          'target angle',
          numeric(task.x * DEG, '°', (x) => patch(i, { x: x / DEG })),
        ),
      );
    box.append(
      row(
        'frequency',
        numeric(task.wn, 'rad/s (0 = off)', (wn) => patch(i, { wn }), 0),
      ),
    );
    const actions = el('div', 'card-actions');
    const move = (offset: number) => {
      const tasks = [...client.tasks];
      const other = i + offset;
      if (other < 0 || other >= tasks.length) return;
      [tasks[i], tasks[other]] = [tasks[other]!, tasks[i]!];
      commit(tasks, true);
    };
    actions.append(
      button('here', () => {
        const [x, y, a] = here(client.tasks[i]!.frame);
        patch(i, { x: task.kind === 'point' ? x : a, y }, true);
      }),
      button('↑', () => move(-1), 'btn small', 'higher priority'),
      button('↓', () => move(1), 'btn small', 'lower priority'),
      button('remove', () =>
        commit(
          client.tasks.filter((_, k) => k !== i),
          true,
        ),
      ),
    );
    box.append(actions);
    host.append(box);
  });
  const add = (kind: TaskGoal['kind']) => {
    if (client.tasks.length >= MAX_TASKS) return;
    const frame = mech.tool >= 0 ? -2 : 0;
    const [x, y, a] = here(frame);
    commit([...client.tasks, { kind, frame, x: kind === 'point' ? x : a, y, wn: GOAL_W }], true);
  };
  const actions = el('div', 'card-actions');
  for (const [kind, label] of [
    ['point', 'Add point goal'],
    ['orientation', 'Add orientation goal'],
  ] as const) {
    const b = button(label, () => add(kind));
    b.disabled = client.tasks.length >= MAX_TASKS;
    actions.append(b);
  }
  host.append(actions);
  return host;
}
