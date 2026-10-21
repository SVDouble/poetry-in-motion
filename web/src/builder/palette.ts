/**
 * The parts palette on the builder viewport: every part drawn as its textbook symbol. Press on a part and
 * drag it into the view, or click it and then click where it goes.
 */
import { el } from '../ui/widgets';
import { PARTS, type PartKind } from './parts';

/** Each symbol in a 44 × 26 box, stroked in the current colour. */
const ICONS: Record<PartKind, string> = {
  bar: '<circle cx="8" cy="18" r="3.4" fill="var(--card)"/><path d="M11 16.5 L38 6" stroke-width="3.2"/>',
  slider:
    '<path d="M4 20 H40" stroke-dasharray="3 2"/><rect x="15" y="15" width="12" height="10" fill="var(--card)"/><path d="M21 15 V3" stroke-width="3"/>',
  cart: '<rect x="10" y="6" width="24" height="11" fill="var(--card)"/><circle cx="15" cy="19.5" r="2.5" fill="var(--card)"/><circle cx="29" cy="19.5" r="2.5" fill="var(--card)"/><path d="M3 22.5 H41"/><path d="M6 22.5 l-3 3 M12 22.5 l-3 3 M18 22.5 l-3 3 M24 22.5 l-3 3 M30 22.5 l-3 3 M36 22.5 l-3 3 M42 22.5 l-3 3" stroke-width="0.8"/>',
  weld: '<path d="M6 19 L38 7" stroke-width="4"/><rect x="3" y="15.5" width="7" height="7" fill="currentColor" transform="rotate(45 6.5 19)"/>',
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
    this.el.append(el('div', 'palette-title', 'Parts'));
    for (const p of PARTS) {
      const b = el('button', 'palette-item');
      b.type = 'button';
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
