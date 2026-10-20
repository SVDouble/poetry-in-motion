"""Generate Rust kernels and LaTeX modules for every system: `uv run meowfer-derive`."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from . import emit_rust, emit_ts
from .model import derive
from .systems import ALL

ROOT = Path(__file__).resolve().parents[3]
RUST_OUT = ROOT / "core" / "src" / "generated"
TS_OUT = ROOT / "web" / "src" / "generated"


def render() -> dict[Path, str]:
    files: dict[Path, str] = {}
    idents = []
    for make in ALL:
        system = make()
        d = derive(system)
        files[RUST_OUT / f"{system.ident}.rs"] = emit_rust.emit(system, d)
        files[TS_OUT / f"{system.ident}.ts"] = emit_ts.emit(system, d)
        idents.append(system.ident)
    files[RUST_OUT / "mod.rs"] = emit_rust.HEADER + "".join(f"pub mod {i};\n" for i in idents)
    files[TS_OUT / "types.ts"] = emit_ts.TYPES
    files[TS_OUT / "index.ts"] = (
        emit_ts.HEADER
        + "".join(f"export {{ {i} }} from './{i}';\n" for i in idents)
        + "export type * from './types';\n"
    )
    return files


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--check", action="store_true", help="fail if generated files are out of date")
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
