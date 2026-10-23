/**
 * The parts palette on the builder viewport: every part drawn as its textbook symbol. Press on a part and
 * drag it into the view, or click it and then click where it goes.
 */
import { el } from '../ui/widgets';
import { PARTS, type PartKind } from './parts';

/** Each symbol in a 44 × 26 box, stroked in the current colour. */
const ICONS: Record<PartKind, string> = {
  bar: '<path d="M8 18 L38 6" stroke-width="3.2"/>',
  ground: '<path d="M7 12 H37 M10 12 l-5 8 M18 12 l-5 8 M26 12 l-5 8 M34 12 l-5 8"/>',
  block: '<rect x="9" y="5" width="26" height="17"/>',
  revolute: '<circle cx="22" cy="13" r="9"/><circle cx="22" cy="13" r="4"/>',
  prismatic: '<path d="M3 8 H41 M3 19 H41"/><rect x="17" y="8" width="10" height="11" fill="var(--card)"/>',
  fixed: '<path d="M10 13 H34 M22 1 V25"/><rect x="17" y="8" width="10" height="10" fill="var(--card)"/>',
  spring: '<path d="M3 13 H9 L12 7 L17 19 L22 7 L27 19 L32 7 L35 13 H41"/>',
  damper: '<path d="M3 13 H12 M24 7 H12 V19 H24 M20 8.5 V17.5 M20 13 H41"/>',
  springDamper:
    '<path d="M3 13 H7 M7 5 V21 M37 5 V21 M37 13 H41"/><path d="M7 6 H11 L14 2.5 L18 9.5 L22 2.5 L26 9.5 L29 6 H37" stroke-width="1"/><path d="M7 20 H13 M22 16 H13 V24 H22 M19 17 V23 M19 20 H37" stroke-width="1"/>',
  pin: '<path d="M5 20 L22 13 M22 13 L39 20" stroke-width="2.6"/><circle cx="22" cy="13" r="3.4" fill="var(--card)"/><circle cx="22" cy="13" r="1.2" fill="currentColor"/>',
  slot: '<path d="M4 9 H40 M4 19 H40"/><rect x="16" y="10" width="12" height="8" fill="var(--card)"/><circle cx="22" cy="14" r="2" fill="var(--card)"/>',
};

export interface PaletteEvents {
  /** A part was picked up (the viewport takes over the pointer from here). */
  start(kind: PartKind): void;
}

export class Palette {
  readonly el = el('div', 'palette overlay');

  constructor(events: PaletteEvents) {
    let group = '';
    for (const p of PARTS) {
      if (p.group !== group) {
        group = p.group;
        this.el.append(el('div', 'palette-title', group));
      }
      const b = el('button', 'palette-item');
      b.type = 'button';
      b.dataset.part = p.kind;
      b.setAttribute('aria-label', `Place ${p.label}`);
      b.addEventListener('keydown', (e) => {
        if (e.key === 'Enter' || e.key === ' ') {
          e.preventDefault();
          events.start(p.kind);
        }
      });
      b.title = `${p.label}: ${p.hint}${p.twoEnded ? ' — drop one end, then click the other' : ''}`;
      b.innerHTML = `<svg viewBox="0 0 44 26" width="44" height="26" fill="none" stroke="currentColor" stroke-width="1.4" stroke-linecap="round" stroke-linejoin="round">${ICONS[p.kind]}</svg><span>${p.label}</span>`;
      b.addEventListener('pointerdown', (e) => {
        e.preventDefault();
        events.start(p.kind);
      });
      this.el.append(b);
    }
  }
}
