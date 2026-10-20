"""Regenerate the SymPy reference kernels used as a test oracle: `uv run motion-derive`.

The browser derives every mechanism with its own computer-algebra system (`core/src/cas`). This package
derives the demo systems independently with SymPy and emits Rust kernels into `core/tests/oracle/`, so
`cargo test` can check that both derivations agree.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from . import emit_rust
from .model import derive
from .systems import ALL

ROOT = Path(__file__).resolve().parents[3]
ORACLE = ROOT / "core" / "tests" / "oracle"


def render() -> dict[Path, str]:
    files: dict[Path, str] = {}
    for make in ALL:
        system = make()
        files[ORACLE / f"{system.ident}.rs"] = emit_rust.emit(system, derive(system))
    return files


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--check", action="store_true", help="fail if generated code is out of date")
    args = ap.parse_args()

    stale = []
    for path, text in render().items():
        if args.check:
            if not path.exists() or path.read_text() != text:
                stale.append(path)
            continue
        path.parent.mkdir(parents=True, exist_ok=True)
        if not path.exists() or path.read_text() != text:
            path.write_text(text)
            print(f"wrote {path.relative_to(ROOT)}")
    if stale:
        print("out of date (run `make gen`):", *(str(p.relative_to(ROOT)) for p in stale), sep="\n  ")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
