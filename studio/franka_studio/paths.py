"""Workspace locations and path helpers."""

from __future__ import annotations

import sys
from pathlib import Path

# studio/franka_studio/paths.py -> workspace root
WORKSPACE = Path(__file__).resolve().parents[2]
STATE_DIR = WORKSPACE / ".studio"
STATIC_DIR = Path(__file__).resolve().parent / "server" / "static"


def resolve(path: str | Path, base: Path = WORKSPACE) -> Path:
    """Expands ``~`` and makes a relative path absolute against ``base``."""
    p = Path(str(path)).expanduser()
    return p if p.is_absolute() else (base / p).resolve()


def add_import_paths(dirs: list[str]) -> list[Path]:
    """Puts directories on ``sys.path`` (first wins), e.g. franka_client's flat modules."""
    added = []
    for d in reversed(dirs):
        p = resolve(d)
        if p.is_dir() and str(p) not in sys.path:
            sys.path.insert(0, str(p))
            added.append(p)
    return added
