"""The observation space of a project: which modalities exist and which are switched on.

Declared in the project's ``env.yaml``::

    modalities:
      state:      {label: Robot state, keys: [observation.state], required: true}
      rgb:        {label: RGB cameras, keys: ["observation.images.*"], required: true}
      pointcloud:
        label: Point cloud
        keys: [observation.pointcloud]       # mapping keys (fnmatch patterns) of this modality
        point_clouds: [ffs]                  # scene.point_clouds entries shown with it
        record: [/ffs/points]                # extra topics the data collector records with it
        component: pointcloud                # started when switched on, stopped when off
        default: false

A switched-off modality drops its mappings from the project env (so ``observation_space`` and
the observation shown are exactly what is on), its scene clouds and its recorded topics. Mappings
that match no modality are always on. Policies keep their own mappings unchanged: a policy is
fed what it was trained on.
"""

from __future__ import annotations

import copy
import fnmatch
from dataclasses import dataclass, field


@dataclass
class Modality:
    id: str
    label: str
    keys: list[str] = field(default_factory=list)
    point_clouds: list[str] = field(default_factory=list)
    record: list[str] = field(default_factory=list)
    component: str | None = None
    required: bool = False
    default: bool = True
    description: str = ""

    def matches(self, key: str) -> bool:
        return any(fnmatch.fnmatchcase(key, pat) for pat in self.keys)

    def to_dict(self) -> dict:
        return {"id": self.id, "label": self.label, "keys": self.keys, "record": self.record,
                "component": self.component, "required": self.required, "description": self.description}


def load_modalities(env_config: dict) -> dict[str, Modality]:
    out = {}
    for mid, m in (env_config.get("modalities") or {}).items():
        m = m or {}
        out[mid] = Modality(
            id=mid, label=m.get("label", mid), keys=list(m.get("keys") or []),
            point_clouds=list(m.get("point_clouds") or []), record=list(m.get("record") or []),
            component=m.get("component"), required=bool(m.get("required", False)),
            default=bool(m.get("default", True)), description=m.get("description", ""))
    return out


def resolve_enabled(modalities: dict[str, Modality], remembered: list | None) -> set[str]:
    """The remembered choice (or the defaults), always with the required modalities."""
    if remembered is None:
        on = {m.id for m in modalities.values() if m.default}
    else:
        on = {mid for mid in remembered if mid in modalities}
    return on | {m.id for m in modalities.values() if m.required}


def apply(env_config: dict, modalities: dict[str, Modality], enabled: set[str]) -> dict:
    """A copy of ``env_config`` without the mappings and scene clouds of switched-off modalities."""
    off = [m for mid, m in modalities.items() if mid not in enabled]
    cfg = copy.deepcopy(env_config)
    if not off:
        return cfg
    cfg["mappings"] = {k: v for k, v in (cfg.get("mappings") or {}).items()
                       if not any(m.matches(k) for m in off)}
    scene = cfg.get("scene") or {}
    hidden = {name for m in off for name in m.point_clouds}
    if scene.get("point_clouds"):
        scene["point_clouds"] = [c for c in scene["point_clouds"] if c.get("name") not in hidden]
    return cfg


def record_topics(modalities: dict[str, Modality], enabled: set[str]) -> list[str]:
    topics: list[str] = []
    for mid in sorted(enabled):
        for t in modalities[mid].record if mid in modalities else []:
            if t not in topics:
                topics.append(t)
    return topics
