"""Drives the data collector (``pixi run data-collection``) and files its episodes.

The collector node records into its own ``storage_path`` and is toggled by its
``record_data_trigger`` service; it knows nothing about datasets or tasks. The studio adds that:

1. Record -> trigger; the answer names the episode directory (``episode_<ts>``).
2. Stop -> trigger; the collector stops the bag and encodes the videos in the background.
3. Once nothing writes into the episode any more (the bag is closed and no ``ffmpeg`` still has the
   episode in its command line), it is moved to ``<data_root>/<dataset>/<task>/episode_<ts>`` and a
   ``studio_episode.json`` with task, instruction, operator and, when a policy is loaded (DAgger),
   that policy is written next to it.
4. Discard deletes the last episode, wherever it is in that pipeline.
"""

from __future__ import annotations

import json
import os
import re
import shutil
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable

from franka_studio.paths import resolve

STARTED_RE = re.compile(r"Started recording to (.+)$")
SETTLE_S = 1.5          # files must stay unchanged this long before an episode is moved
FINALIZE_TIMEOUT_S = 900.0
NAME_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._ -]*$")


@dataclass
class Episode:
    """One recorded episode on its way from the collector's folder to the dataset."""

    source: Path
    dataset: str
    task: str
    meta: dict
    started_at: float
    stopped_at: float | None = None
    status: str = "recording"     # recording | finalizing | saved | discarded | failed
    target: Path | None = None
    error: str = ""
    discard_requested: bool = False
    extra: dict = field(default_factory=dict)

    def to_dict(self) -> dict:
        end = self.stopped_at or time.time()
        return {"name": self.source.name, "dataset": self.dataset, "task": self.task,
                "status": self.status, "duration": end - self.started_at, "error": self.error,
                "path": str(self.target or self.source), "kind": self.meta.get("kind", "demo")}


def valid_name(name: str) -> bool:
    return bool(name) and bool(NAME_RE.match(name)) and ".." not in name


def _writers_active(path: Path) -> bool:
    """True while an ffmpeg (the collector's encoder) has the episode in its command line."""
    needle = str(path)
    for pid in os.listdir("/proc"):
        if not pid.isdigit():
            continue
        try:
            cmd = Path(f"/proc/{pid}/cmdline").read_bytes()
        except OSError:
            continue
        if b"ffmpeg" in cmd and needle.encode() in cmd:
            return True
    return False


def _tree_signature(path: Path) -> tuple:
    sig = []
    for p in sorted(path.rglob("*")):
        try:
            st = p.stat()
            sig.append((str(p), st.st_size, st.st_mtime_ns))
        except OSError:
            pass
    return tuple(sig)


class DataCollection:
    """Recording state machine on top of the collector's trigger service."""

    def __init__(self, call_trigger: Callable[[], tuple[bool, str]], expected_videos: int = 0) -> None:
        self._call_trigger = call_trigger
        self._lock = threading.Lock()
        self.current: Episode | None = None
        self.history: list[Episode] = []   # newest first
        self.message = ""
        self.expected_videos = expected_videos

    @property
    def recording(self) -> bool:
        return self.current is not None and self.current.status == "recording"

    def start(self, data_root: Path, dataset: str, task: str, meta: dict) -> tuple[bool, str]:
        if not (valid_name(dataset) and valid_name(task)):
            return False, "Dataset and task need simple names (letters, digits, - _ . and spaces)."
        with self._lock:
            if self.recording:
                return False, "Already recording."
            ok, msg = self._call_trigger()
            if not ok:
                return False, f"Data collector: {msg}"
            m = STARTED_RE.search(msg.strip())
            if not m:  # the trigger toggles: we were out of sync and just stopped something
                return False, f"Unexpected answer from the data collector ({msg!r}); press Record again."
            ep = Episode(resolve(m.group(1).strip()), dataset, task, meta, time.time())
            ep.target = data_root / dataset / task / ep.source.name
            self.current = ep
            self.history.insert(0, ep)
            del self.history[20:]
        return True, f"Recording {ep.source.name} → {dataset}/{task}"

    def stop(self) -> tuple[bool, str]:
        with self._lock:
            ep = self.current
            if ep is None or ep.status != "recording":
                return False, "Not recording."
            ok, msg = self._call_trigger()
            if not ok:
                return False, f"Data collector: {msg}"
            ep.stopped_at = time.time()
            ep.status = "finalizing"
            self.current = None
        threading.Thread(target=self._finalize, args=(ep,), daemon=True).start()
        return True, f"Stopped after {ep.stopped_at - ep.started_at:.1f} s — saving to {ep.dataset}/{ep.task}"

    def discard_last(self) -> tuple[bool, str]:
        with self._lock:
            ep = next((e for e in self.history if e.status not in ("discarded", "failed")), None)
            if ep is None:
                return False, "Nothing to discard."
            if ep.status == "recording":
                return False, "Stop the recording first."
            if ep.status == "finalizing":
                ep.discard_requested = True
                return True, f"{ep.source.name} will be discarded once the collector has finished writing it."
        return self._delete(ep)

    def _delete(self, ep: Episode) -> tuple[bool, str]:
        for path in {ep.source, ep.target}:
            if path is not None and path.exists():
                try:
                    shutil.rmtree(path)
                except OSError as e:
                    ep.error = str(e)
                    return False, f"Could not delete {path}: {e}"
        ep.status = "discarded"
        return True, f"Discarded {ep.source.name}."

    def _finalize(self, ep: Episode) -> None:
        deadline = time.time() + FINALIZE_TIMEOUT_S
        last_sig, stable_since = None, time.time()
        while time.time() < deadline:
            time.sleep(0.5)
            if not ep.source.exists():
                continue
            sig = _tree_signature(ep.source)
            if sig != last_sig:
                last_sig, stable_since = sig, time.time()
                continue
            bag_closed = any(ep.source.glob("mcap/metadata.yaml"))
            if bag_closed and not _writers_active(ep.source) and time.time() - stable_since > SETTLE_S:
                break
        else:
            ep.status, ep.error = "failed", "timed out waiting for the collector to finish writing"
            return
        if ep.discard_requested:
            ok, msg = self._delete(ep)
            self.message = msg
            return
        try:
            ep.target.parent.mkdir(parents=True, exist_ok=True)
            if ep.target != ep.source:
                shutil.move(str(ep.source), str(ep.target))
            meta = {**ep.meta, "episode": ep.source.name, "dataset": ep.dataset, "task": ep.task,
                    "started_at": ep.started_at, "stopped_at": ep.stopped_at,
                    "duration_s": (ep.stopped_at or ep.started_at) - ep.started_at}
            (ep.target / "studio_episode.json").write_text(json.dumps(meta, indent=2, default=str))
            ep.status = "saved"
            self.message = f"Saved {ep.source.name} to {ep.dataset}/{ep.task}."
        except Exception as e:
            ep.status, ep.error = "failed", str(e)
            self.message = f"Saving {ep.source.name} failed: {e}"

    def snapshot(self) -> dict:
        cur = self.current
        return {
            "recording": self.recording,
            "current": cur.to_dict() if cur else None,
            "history": [e.to_dict() for e in self.history[:8]],
            "can_discard": any(e.status in ("finalizing", "saved") and not e.discard_requested
                               for e in self.history[:1]),
            "message": self.message,
        }


def list_datasets(data_root: Path) -> dict[str, dict[str, int]]:
    """``{dataset: {task: episode_count}}`` below ``data_root``."""
    out: dict[str, dict[str, int]] = {}
    if not data_root.is_dir():
        return out
    for ds in sorted(p for p in data_root.iterdir() if p.is_dir() and not p.name.startswith(".")):
        tasks = {}
        for task in sorted(p for p in ds.iterdir() if p.is_dir() and not p.name.startswith(".")):
            tasks[task.name] = sum(1 for e in task.iterdir() if e.is_dir() and e.name.startswith("episode_"))
        out[ds.name] = tasks
    return out
