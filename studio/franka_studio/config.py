"""Studio configuration (``config/studio.yaml``) with defaults."""

from __future__ import annotations

import copy
from pathlib import Path
from typing import Any

import yaml

from franka_studio.paths import WORKSPACE, resolve

DEFAULT_CONFIG_PATH = WORKSPACE / "config" / "studio.yaml"

DEFAULTS: dict[str, Any] = {
    "server": {"host": "127.0.0.1", "port": 8090},
    "viewer": {"host": "127.0.0.1", "port": 8091, "rate_hz": 15.0, "public_url": None},
    "monitor_rate_hz": 10.0,
    "projects_dir": "projects",
    "default_project": None,
    "client_paths": ["src/franka_py/franka_client/franka_client"],
    "plugin_modules": [],
    "wire": {
        "observation_module": "franka_client.client",
        "policy_config_module": "lerobot_utils",
    },
    "components": {},
    "data_collector": {
        "trigger_service": "/data_collector/record_data_trigger",
        "component": "data_collection",
        "node": "/data_collector",        # its extra_topics parameter: what the modalities add
    },
    "pointcloud_topics": {"status": "/ffs/status", "extrinsics": "/ffs/extrinsics", "align": "/ffs/align"},
    # Automatic camera calibration (calibration.py): the board on the table, the cameras, motion limits
    "calibration": {
        "board": {"dict": "DICT_4X4_50", "cols": 3, "rows": 4, "first_id": 0, "marker": 0.0565, "sep": 0.0055},
        "arms": {"zed_right_arm": "franka_right", "zed_left_arm": "franka_left"},   # wrist camera -> arm
        "static": ["zed_middle"],
        "topic_prefix": "/zed_multi",
        "views": 8,                  # per arm, after 3 probes
        "speed": 0.05,               # m/s of the grasp point
        "rotation_speed_deg": 15.0,  # deg/s
        "settle_s": 1.0,             # still time before each capture
        "max_offset": 0.20,          # m from the start pose
        "max_rotation_deg": 45.0,    # from the start orientation
        "min_clearance": 0.08,       # m of the grasp point above the board
        "min_distance": 0.30,        # m camera - board centre
        "max_distance": 0.55,
    },
}


def deep_merge(base: dict, override: dict) -> dict:
    """Returns ``base`` updated recursively with ``override`` (neither is modified)."""
    out = copy.deepcopy(base)
    for key, value in (override or {}).items():
        if isinstance(value, dict) and isinstance(out.get(key), dict):
            out[key] = deep_merge(out[key], value)
        else:
            out[key] = copy.deepcopy(value)
    return out


def load_config(path: str | Path | None = None) -> dict:
    """Loads the studio config, falling back to the defaults for missing keys."""
    path = resolve(path) if path else DEFAULT_CONFIG_PATH
    user: dict = {}
    if path.exists():
        with open(path) as f:
            user = yaml.safe_load(f) or {}
    cfg = deep_merge(DEFAULTS, user)
    cfg["_path"] = str(path)
    return cfg
