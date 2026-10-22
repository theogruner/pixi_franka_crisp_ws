"""Minimal stand-ins for ``gymnasium.spaces`` (gymnasium is not a workspace dependency).

When gymnasium is installed the real spaces are used, so a SceneEnv can be handed to gym tooling.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

try:  # pragma: no cover - optional
    from gymnasium import spaces as _gym_spaces
except ImportError:  # pragma: no cover
    _gym_spaces = None


@dataclass
class Box:
    """An n-dimensional array space; bounds are unknown for most ROS-derived features."""

    shape: tuple
    dtype: str = "float32"
    low: float = -np.inf
    high: float = np.inf

    def to_dict(self) -> dict:
        return {"type": "box", "shape": list(self.shape), "dtype": self.dtype}


@dataclass
class Text:
    """A free-form language instruction."""

    max_length: int = 1024

    def to_dict(self) -> dict:
        return {"type": "text"}


@dataclass
class Dict:
    """A named collection of spaces."""

    spaces: dict = field(default_factory=dict)

    def __getitem__(self, key: str) -> object:
        return self.spaces[key]

    def to_dict(self) -> dict:
        return {"type": "dict", "spaces": {k: v.to_dict() for k, v in self.spaces.items()}}


def box(shape: tuple, dtype: str) -> object:
    """A Box, as a gymnasium space when gymnasium is available."""
    if _gym_spaces is not None and dtype != "video":
        return _gym_spaces.Box(-np.inf, np.inf, shape=tuple(shape), dtype=np.dtype(dtype))
    if _gym_spaces is not None:
        return _gym_spaces.Box(0, 255, shape=tuple(shape), dtype=np.uint8)
    return Box(tuple(shape), "uint8" if dtype == "video" else dtype)


def describe(space: object) -> dict:
    """JSON-friendly description of either kind of space."""
    if hasattr(space, "to_dict"):
        return space.to_dict()
    if hasattr(space, "spaces"):
        return {"type": "dict", "spaces": {k: describe(v) for k, v in space.spaces.items()}}
    return {"type": type(space).__name__.lower(), "shape": list(getattr(space, "shape", ()) or ()),
            "dtype": str(getattr(space, "dtype", ""))}
