import { cloneMechanism, encode, type Mechanism } from '../model/description';
import type { Built, SimClient } from '../sim/client';

type Outcome =
  | { kind: 'installed'; document: Mechanism; result: Built }
  | { kind: 'empty' | 'stale' }
  | { kind: 'error'; error: string };

/** Owns authored/installed lifetimes. A prepared revision is accepted before the worker commits it;
 * an accepted reply remains authoritative even if another edit arrives while its commit is in flight. */
export class DocumentSession {
  private runtimeDocument: Mechanism | null = null;
  get installed(): Mechanism | null {
    return this.runtimeDocument;
  }
  private revision = 0;
  private timer: ReturnType<typeof setTimeout> | null = null;

  constructor(
    public draft: Mechanism,
    private readonly client: Pick<SimClient, 'build' | 'install'>,
  ) {}

  edit(document: Mechanism): void {
    this.draft = document;
    this.revision++;
    this.cancelScheduled();
  }

  schedule(rebuild: () => void, delay: number): void {
    this.cancelScheduled();
    this.timer = setTimeout(rebuild, delay);
  }

  cancelScheduled(): void {
    if (this.timer !== null) clearTimeout(this.timer);
    this.timer = null;
  }

  async rebuild(): Promise<Outcome> {
    this.cancelScheduled();
    const revision = this.revision;
    const document = cloneMechanism(this.draft);
    const current = () => revision === this.revision;
    if (!document.bodies.length) {
      this.runtimeDocument = null;
      return { kind: 'empty' };
    }
    const result = await this.client.build(encode(document), current);
    if (!result.ok) return current() ? { kind: 'error', error: result.error } : { kind: 'stale' };
    this.client.install(result);
    if (!this.draft.bodies.length) {
      this.runtimeDocument = null;
      return { kind: 'empty' };
    }
    this.runtimeDocument = document;
    return { kind: 'installed', document, result };
  }
}
