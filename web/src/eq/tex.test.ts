import { describe, expect, it } from 'vitest';
import { fix, sci, texMat } from './tex';

describe('number formatting', () => {
  it('keeps about four significant digits and uses a true minus sign', () => {
    expect(fix(1.23456)).toBe('1.235');
    expect(fix(-12.3456)).toBe('−12.35');
    expect(fix(123.456)).toBe('123.5');
    expect(fix(-0.0001)).toBe('0.000');
    expect(fix(Number.NaN)).toBe('—');
  });

  it('prints scientific notation compactly', () => {
    expect(sci(1.5e-16)).toBe('1.5e−16');
    expect(sci(0)).toBe('0');
  });

  it('builds bmatrix sources', () => {
    expect(
      texMat([
        ['a', 'b'],
        ['c', 'd'],
      ]),
    ).toBe(String.raw`\begin{bmatrix}a & b\\c & d\end{bmatrix}`);
  });
});
