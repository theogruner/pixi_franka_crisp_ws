"""Small persisted UI state: last project, and per project the last dataset/task/policy."""

from __future__ import annotations

import json
import threading
from typing import Any

from franka_studio.paths import STATE_DIR

STATE_FILE = STATE_DIR / "state.json"


class UiState:
    """Thread-safe JSON key/value store under ``.studio/state.json``."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        try:
            self._data: dict = json.loads(STATE_FILE.read_text())
        except (OSError, ValueError):
            self._data = {}

    def get(self, key: str, default: Any = None) -> Any:
        with self._lock:
            return self._data.get(key, default)

    def project(self, project: str) -> dict:
        """The remembered selections of one project (a copy)."""
        with self._lock:
            return dict(self._data.get("projects", {}).get(project, {}))

    def set(self, key: str, value: Any) -> None:
        with self._lock:
            self._data[key] = value
            self._save()

    def set_project(self, project: str, **values: Any) -> None:
        with self._lock:
            self._data.setdefault("projects", {}).setdefault(project, {}).update(values)
            self._save()

    def _save(self) -> None:
        try:
            STATE_DIR.mkdir(parents=True, exist_ok=True)
            tmp = STATE_FILE.with_suffix(".tmp")
            tmp.write_text(json.dumps(self._data, indent=2))
            tmp.replace(STATE_FILE)
        except OSError:
            pass  # convenience only: the UI works without it
