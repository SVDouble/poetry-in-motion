/** One floating menu at a time, anchored to an element; closes on outside click, Escape or `close()`. */

let current: { el: HTMLElement; anchor: HTMLElement; off: () => void } | null = null;

export function closePopover(): void {
  current?.off();
  current?.el.remove();
  current = null;
}

/** Show `content` next to `anchor` (toggles when the same anchor is clicked again). */
export function popover(anchor: HTMLElement, content: HTMLElement, align: 'left' | 'right' = 'left'): void {
  const same = current?.anchor === anchor;
  closePopover();
  if (same) return;
  const el = document.createElement('div');
  el.className = 'menu';
  el.append(content);
  document.body.append(el);
  const r = anchor.getBoundingClientRect();
  const w = el.offsetWidth;
  const h = el.offsetHeight;
  const left = align === 'left' ? r.left : r.right - w;
  el.style.left = `${Math.max(8, Math.min(left, innerWidth - w - 8))}px`;
  const below = r.bottom + 6 + h < innerHeight;
  el.style.top = `${below ? r.bottom + 6 : Math.max(8, r.top - 6 - h)}px`;
  const outside = (e: PointerEvent) => {
    const t = e.target as Node;
    if (!el.contains(t) && !anchor.contains(t)) closePopover();
  };
  const esc = (e: KeyboardEvent) => {
    if (e.key === 'Escape') closePopover();
  };
  window.addEventListener('pointerdown', outside, true);
  window.addEventListener('keydown', esc);
  current = {
    el,
    anchor,
    off: () => {
      window.removeEventListener('pointerdown', outside, true);
      window.removeEventListener('keydown', esc);
    },
  };
}
