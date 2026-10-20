/**
 * The equations panel: collapsible sections of typeset maths whose numbers follow the simulation.
 * Only sections that are open *and* on screen are refreshed.
 */
import { LiveScope, tex } from './tex';

export interface Block {
  el: HTMLElement;
  /** When given, the block is highlighted while this returns true. */
  active?: () => boolean;
}

export class SectionBuilder {
  readonly live = new LiveScope();
  readonly blocks: Block[] = [];

  constructor(private readonly body: HTMLElement) {}

  /** Prose with inline `$…$` maths. */
  text(html: string, cls = 'prose'): HTMLElement {
    const p = document.createElement('p');
    p.className = cls;
    p.innerHTML = html.replace(
      /\$([^$]+)\$/g,
      (_m, t: string) => `<span data-tex="${encodeURIComponent(t)}"></span>`,
    );
    for (const span of p.querySelectorAll<HTMLElement>('[data-tex]')) {
      tex(span, decodeURIComponent(span.dataset.tex ?? ''));
    }
    this.body.append(p);
    return p;
  }

  /** Display maths. Use `this.live` inside `src` for live numbers. */
  math(src: string, opts: { active?: () => boolean; caption?: string } = {}): HTMLElement {
    const wrap = document.createElement('div');
    wrap.className = 'math';
    const m = document.createElement('div');
    m.className = 'math-body';
    wrap.append(m);
    if (opts.caption) {
      const c = document.createElement('div');
      c.className = 'math-caption';
      c.textContent = opts.caption;
      wrap.append(c);
    }
    this.body.append(wrap);
    this.live.render(m, src);
    this.blocks.push(opts.active ? { el: wrap, active: opts.active } : { el: wrap });
    return wrap;
  }

  /** Static display maths (no live numbers). */
  formula(src: string, caption?: string): HTMLElement {
    return this.math(src, caption ? { caption } : {});
  }
}

export interface SectionSpec {
  id: string;
  title: string;
  /** Short reference shown next to the title, e.g. "MLS (4.24)". */
  cite?: string;
  open?: boolean;
  build(b: SectionBuilder): void;
}

interface Section {
  details: HTMLDetailsElement;
  builder: SectionBuilder;
  visible: boolean;
}

export class EquationPanel {
  private sections: Section[] = [];
  private observer: IntersectionObserver;

  private root: HTMLElement;

  constructor(private readonly home: HTMLElement) {
    this.root = home;
    this.observer = new IntersectionObserver(
      (entries) => {
        for (const e of entries) {
          const s = this.sections.find((x) => x.details === e.target);
          if (s) s.visible = e.isIntersecting;
        }
      },
      { root: null, rootMargin: '64px' },
    );
  }

  /** Render `specs` into `into`, or into the panel's own element (cleared first). */
  mount(specs: SectionSpec[], into?: HTMLElement): void {
    this.clear(!into);
    this.root = into ?? this.home;
    const openState = loadOpenState();
    for (const spec of specs) {
      const details = document.createElement('details');
      details.className = 'eq-section';
      details.open = openState[spec.id] ?? spec.open ?? true;
      details.addEventListener('toggle', () => {
        openState[spec.id] = details.open;
        saveOpenState(openState);
      });
      const summary = document.createElement('summary');
      summary.innerHTML = `<span class="eq-title">${spec.title}</span>${spec.cite ? `<span class="eq-cite">${spec.cite}</span>` : ''}`;
      const body = document.createElement('div');
      body.className = 'eq-body';
      details.append(summary, body);
      this.root.append(details);
      const builder = new SectionBuilder(body);
      spec.build(builder);
      this.sections.push({ details, builder, visible: true });
      this.observer.observe(details);
    }
  }

  clear(empty = true): void {
    this.observer.disconnect();
    this.sections = [];
    if (empty) this.home.replaceChildren();
  }

  update(): void {
    for (const s of this.sections) {
      if (!s.visible || !s.details.open) continue;
      s.builder.live.update();
      for (const b of s.builder.blocks) {
        if (b.active) b.el.classList.toggle('is-active', b.active());
      }
    }
  }
}

const OPEN_KEY = 'motion.sections';

function loadOpenState(): Record<string, boolean> {
  try {
    return JSON.parse(localStorage.getItem(OPEN_KEY) ?? '{}') as Record<string, boolean>;
  } catch {
    return {};
  }
}

function saveOpenState(s: Record<string, boolean>): void {
  try {
    localStorage.setItem(OPEN_KEY, JSON.stringify(s));
  } catch {
    // storage unavailable (private mode) — collapsing state is a convenience only
  }
}
