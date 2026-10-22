import { MAX_TASKS, TASK_WIDTH } from '../generated/contracts';

/** Runtime targets belong to the installed mechanism. Frames: −2 tool, −1 ground, otherwise body index. */
export interface TaskGoal {
  kind: 'orientation' | 'point';
  frame: number;
  x: number;
  y: number;
  wn: number;
}
export function encodeTasks(tasks: readonly TaskGoal[]): number[] {
  if (tasks.length > MAX_TASKS) throw new Error(`At most ${MAX_TASKS} additional tasks`);
  return tasks.flatMap((t) => [t.kind === 'orientation' ? 0 : 1, t.frame, t.x, t.y, t.wn]);
}
export function decodeTasks(flat: ArrayLike<number>): TaskGoal[] {
  return Array.from({ length: flat.length / TASK_WIDTH }, (_, i) => ({
    kind: flat[TASK_WIDTH * i] === 0 ? 'orientation' : 'point',
    frame: flat[TASK_WIDTH * i + 1]!,
    x: flat[TASK_WIDTH * i + 2]!,
    y: flat[TASK_WIDTH * i + 3]!,
    wn: flat[TASK_WIDTH * i + 4]!,
  }));
}
