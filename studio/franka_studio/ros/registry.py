"""Observation extractors and action processors, shared with the policy clients.

The studio uses the very same classes the policy clients use (franka_client's
``msg_extractors.MSG_EXTRACTOR`` and ``action_processors.ACTION_PROCESSORS``), so what the scene
env produces is what a policy sees. More modules can register extra classes through
``plugin_modules`` in studio.yaml (e.g. another client package with more action types).

Lookups are tolerant to how a type is spelled (``sensor_msgs/Image`` vs ``sensor_msgs/msg/Image``)
and to extractors registered under a wrong key: each class is also indexed under the type its own
``msgtype`` property reports.
"""

from __future__ import annotations

import importlib
import logging

from franka_studio.ros.msgs import normalize

log = logging.getLogger(__name__)

_extractors: dict[str, type] = {}
_processors: dict[str, type] = {}
_loaded = False


def _index_extractors(registry: dict[str, type]) -> None:
    for key, cls in registry.items():
        _extractors[normalize(key)] = cls
    for cls in registry.values():  # registered keys win over what a class reports
        try:  # the type the class says it handles (a constant property in all extractors)
            own = cls.msgtype.fget(cls)
        except Exception:
            own = None
        if isinstance(own, str):
            _extractors.setdefault(normalize(own), cls)


def load(plugin_modules: list[str] | None = None) -> None:
    """Imports the registries once (``client_paths`` must already be on sys.path)."""
    global _loaded
    if _loaded:
        return
    for name in ["msg_extractors", "action_processors", *(plugin_modules or [])]:
        try:
            module = importlib.import_module(name)
        except Exception as e:
            log.warning("could not import %s: %s", name, e)
            continue
        _index_extractors(getattr(module, "MSG_EXTRACTOR", {}) or {})
        _processors.update(getattr(module, "ACTION_PROCESSORS", {}) or {})
    _loaded = True


def extractor_class(msgtype: str) -> type | None:
    return _extractors.get(normalize(msgtype))


def processor_class(name: str) -> type | None:
    return _processors.get(name)


def available() -> dict[str, list[str]]:
    return {"extractors": sorted(_extractors), "processors": sorted(_processors)}
