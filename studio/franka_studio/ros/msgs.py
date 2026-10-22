"""Message type strings -> ROS message classes."""

from __future__ import annotations

import importlib
from functools import lru_cache


def normalize(msgtype: str) -> str:
    """``pkg/Type`` and ``pkg.msg.Type`` -> ``pkg/msg/Type``."""
    if "/" not in msgtype and "." in msgtype:
        parts = msgtype.split(".")
        return f"{parts[0]}/msg/{parts[-1]}"
    parts = msgtype.split("/")
    if len(parts) == 2:
        return f"{parts[0]}/msg/{parts[1]}"
    return msgtype


@lru_cache(maxsize=None)
def import_msg(msgtype: str) -> type:
    """Imports the class of e.g. ``sensor_msgs/msg/Image``."""
    pkg, kind, cls = normalize(msgtype).split("/")
    return getattr(importlib.import_module(f"{pkg}.{kind}"), cls)
