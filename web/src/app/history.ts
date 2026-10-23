/** Undo/redo of immutable documents; bursts of edits (slider drags) within `coalesceMs` form one step. */
export class History<T> {
  private readonly undoStack: T[] = [];
  private readonly redoStack: T[] = [];
  private last = -Infinity;

  constructor(
    private readonly clone: (v: T) => T,
    private readonly coalesceMs = 600,
    private readonly limit = 200,
  ) {}

  /** Remember `current` before it changes. `force` starts a new step even inside a burst. */
  record(current: T, force = false): void {
    const now = performance.now();
    if (!force && now - this.last < this.coalesceMs) return;
    this.last = now;
    this.undoStack.push(this.clone(current));
    if (this.undoStack.length > this.limit) this.undoStack.shift();
    this.redoStack.length = 0;
  }

  /** Step back (or forward with `redo`); returns the document to show, or null. */
  step(current: T, redo = false): T | null {
    const [from, to] = redo ? [this.redoStack, this.undoStack] : [this.undoStack, this.redoStack];
    const v = from.pop();
    if (v === undefined) return null;
    to.push(this.clone(current));
    this.last = -Infinity;
    return v;
  }

  get canUndo(): boolean {
    return this.undoStack.length > 0;
  }

  get canRedo(): boolean {
    return this.redoStack.length > 0;
  }
}
