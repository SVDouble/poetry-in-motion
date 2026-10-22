import { cloneMechanism, encode, type Mechanism } from '../model/description';
import type { Built, SimClient } from '../sim/client';
import { Revisions } from './revision';

type Outcome =
  | { kind: 'installed'; document: Mechanism; result: Built }
  | { kind: 'empty' | 'stale' }
  | { kind: 'error'; error: string };

/** Owns authored/installed lifetimes. A prepared revision is accepted before the worker commits it;
 * an accepted reply remains authoritative even if another edit arrives while its commit is in flight. */
export class DocumentSession {
  installed: Mechanism | null = null;
  private readonly revisions = new Revisions(cloneMechanism);
  private timer: ReturnType<typeof setTimeout> | null = null;

  constructor(
    public draft: Mechanism,
    private readonly client: Pick<SimClient, 'build' | 'install'>,
  ) {}

  edit(document: Mechanism): void {
    this.draft = document;
    this.revisions.change();
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
    const ticket = this.revisions.capture(this.draft);
    if (!ticket.document.bodies.length) return { kind: 'empty' };
    const result = await this.client.build(encode(ticket.document), ticket.current);
    if (!result.ok) return ticket.current() ? { kind: 'error', error: result.error } : { kind: 'stale' };
    this.client.install(result);
    if (!this.draft.bodies.length) return { kind: 'empty' };
    this.installed = ticket.document;
    return { kind: 'installed', document: ticket.document, result };
  }
}
