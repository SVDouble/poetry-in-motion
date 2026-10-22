/** Build tickets capture a document before awaiting work; edits invalidate uncommitted tickets immediately. */
export class Revisions<T> {
  private revision = 0;
  constructor(private readonly clone: (document: T) => T) {}
  change(): void {
    this.revision++;
  }
  capture(document: T): { document: T; current: () => boolean } {
    const revision = this.revision;
    return { document: this.clone(document), current: () => revision === this.revision };
  }
}
