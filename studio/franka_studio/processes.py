"""Start/stop the stack's long-running commands (robot, cameras, teleop, data collector, ...).

Each component is one shell command run in its own process group, so stopping it reaches every
child (``pixi run`` -> bash -> ``ros2 launch`` -> nodes). Output is kept in a ring buffer that the
UI streams. Stop sends SIGINT (ros2 launch shuts its nodes down cleanly on it), then SIGTERM, then
SIGKILL.
"""

from __future__ import annotations

import collections
import os
import signal
import subprocess
import threading
import time
from dataclasses import dataclass, field
from typing import Callable

from franka_studio.paths import WORKSPACE, resolve

LOG_LINES = 3000
STOP_TIMEOUTS_S = (12.0, 5.0)  # after SIGINT, after SIGTERM

STOPPED, STARTING, RUNNING, STOPPING, EXITED, FAILED = (
    "stopped", "starting", "running", "stopping", "exited", "failed")


@dataclass
class ComponentSpec:
    """One startable command, from ``components`` in studio.yaml or a project."""

    name: str
    label: str
    cmd: str | None
    cwd: str = str(WORKSPACE)
    env: dict = field(default_factory=dict)
    description: str = ""
    icon: str = "box"
    # ROS name whose presence means the component is up (also when started outside the UI)
    probe_topic: str | None = None
    probe_service: str | None = None
    order: int = 100

    @classmethod
    def from_dict(cls, name: str, d: dict, order: int = 100) -> "ComponentSpec":
        return cls(
            name=name,
            label=d.get("label", name.replace("_", " ").title()),
            cmd=d.get("cmd"),
            cwd=str(resolve(d.get("cwd", WORKSPACE))),
            env={k: str(v) for k, v in (d.get("env") or {}).items()},
            description=d.get("description", ""),
            icon=d.get("icon", "box"),
            probe_topic=d.get("probe_topic"),
            probe_service=d.get("probe_service"),
            order=d.get("order", order),
        )


class ManagedProcess:
    """A component's process and its captured output."""

    def __init__(self, spec: ComponentSpec, on_exit: Callable[[str, int], None] | None = None):
        self.spec = spec
        self._on_exit = on_exit
        self._lock = threading.Lock()
        self._proc: subprocess.Popen | None = None
        self.state = STOPPED
        self.started_at: float | None = None
        self.exit_code: int | None = None
        self.detected = False  # probe found it running (possibly started outside the UI)
        self._lines: collections.deque[tuple[int, str]] = collections.deque(maxlen=LOG_LINES)
        self._seq = 0
        self._stop_requested = False

    # ---- output ---------------------------------------------------------------------------

    def _append(self, line: str) -> None:
        with self._lock:
            self._seq += 1
            self._lines.append((self._seq, line.rstrip("\n")))

    def lines_since(self, seq: int) -> list[tuple[int, str]]:
        with self._lock:
            if not self._lines or self._lines[-1][0] <= seq:
                return []
            return [item for item in self._lines if item[0] > seq]

    @property
    def last_seq(self) -> int:
        with self._lock:
            return self._seq

    # ---- lifecycle --------------------------------------------------------------------------

    @property
    def alive(self) -> bool:
        """True while the command or anything left in its process group runs."""
        if self._proc is None:
            return False
        return self._proc.poll() is None or self._group_alive()

    def _group_alive(self) -> bool:
        try:
            os.killpg(self._proc.pid, 0)
            return True
        except ProcessLookupError:
            return False
        except PermissionError:
            return True

    def start(self) -> tuple[bool, str]:
        if not self.spec.cmd:
            return False, f"{self.spec.label}: no command configured (see config/studio.yaml)."
        if self.alive:
            return False, f"{self.spec.label} is already running."
        env = {**os.environ, **self.spec.env, "PYTHONUNBUFFERED": "1", "RCUTILS_COLORIZED_OUTPUT": "0"}
        self._append(f"$ {self.spec.cmd}")
        try:
            self._proc = subprocess.Popen(
                ["bash", "-c", self.spec.cmd], cwd=self.spec.cwd, env=env,
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
                start_new_session=True, text=True, bufsize=1, errors="replace",
            )
        except OSError as e:
            self.state = FAILED
            self._append(f"[studio] failed to start: {e}")
            return False, str(e)
        self._stop_requested = False
        self.state, self.started_at, self.exit_code = STARTING, time.time(), None
        threading.Thread(target=self._pump, args=(self._proc,), daemon=True).start()
        return True, f"{self.spec.label} started."

    def _pump(self, proc: subprocess.Popen) -> None:
        assert proc.stdout is not None
        for line in proc.stdout:
            self._append(line)
            if self.state == STARTING:
                self.state = RUNNING
        code = proc.wait()
        while self._group_alive():  # children that outlive the leader (e.g. pixi run exits first)
            time.sleep(0.2)
        self.exit_code = code
        self.state = STOPPED if self._stop_requested else (EXITED if code == 0 else FAILED)
        self._append(f"[studio] process exited with code {code}")
        if self._on_exit:
            self._on_exit(self.spec.name, code)

    def stop(self) -> tuple[bool, str]:
        proc = self._proc
        if proc is None or proc.poll() is not None:
            self.state = STOPPED
            return False, f"{self.spec.label} is not running."
        self._stop_requested = True
        self.state = STOPPING
        threading.Thread(target=self._terminate, args=(proc,), daemon=True).start()
        return True, f"Stopping {self.spec.label}…"

    def _terminate(self, proc: subprocess.Popen) -> None:
        """Signals the whole group, escalating until every process in it is gone."""
        for sig, timeout in ((signal.SIGINT, STOP_TIMEOUTS_S[0]), (signal.SIGTERM, STOP_TIMEOUTS_S[1]),
                             (signal.SIGKILL, 2.0)):
            try:
                os.killpg(proc.pid, sig)
            except ProcessLookupError:
                return
            deadline = time.time() + timeout
            while time.time() < deadline:
                if not self._group_alive():
                    return
                time.sleep(0.1)
            self._append(f"[studio] still running after {sig.name}, escalating")

    def snapshot(self) -> dict:
        s = self.spec
        running = self.alive
        return {
            "name": s.name, "label": s.label, "description": s.description, "icon": s.icon,
            "configured": bool(s.cmd), "cmd": s.cmd,
            "state": self.state if (running or self.state not in (RUNNING, STARTING)) else STOPPED,
            "owned": running, "detected": self.detected,
            "uptime": (time.time() - self.started_at) if (running and self.started_at) else None,
            "exit_code": self.exit_code, "log_seq": self.last_seq,
        }


class ProcessManager:
    """All components, keyed by name, in display order."""

    def __init__(self) -> None:
        self.components: dict[str, ManagedProcess] = {}
        self._probe: Callable[[ComponentSpec], bool] | None = None

    def configure(self, specs: dict[str, dict]) -> None:
        """(Re)declares the components. Running ones keep running; new specs replace idle ones."""
        wanted = {}
        for i, (name, d) in enumerate(specs.items()):
            if d is None:
                continue
            wanted[name] = ComponentSpec.from_dict(name, d, order=i)
        for name in list(self.components):
            if name not in wanted and not self.components[name].alive:
                del self.components[name]
        for name, spec in wanted.items():
            current = self.components.get(name)
            if current is None:
                self.components[name] = ManagedProcess(spec)
            elif not current.alive:
                current.spec = spec
        self.components = dict(sorted(self.components.items(), key=lambda kv: kv[1].spec.order))

    def set_probe(self, probe: Callable[[ComponentSpec], bool]) -> None:
        self._probe = probe

    def refresh_probes(self) -> None:
        if self._probe is None:
            return
        for proc in self.components.values():
            try:
                proc.detected = self._probe(proc.spec)
            except Exception:
                proc.detected = False

    def get(self, name: str) -> ManagedProcess | None:
        return self.components.get(name)

    def is_up(self, name: str) -> bool:
        proc = self.components.get(name)
        return bool(proc and (proc.alive or proc.detected))

    def start(self, name: str) -> tuple[bool, str]:
        proc = self.components.get(name)
        return proc.start() if proc else (False, f"Unknown component {name!r}.")

    def stop(self, name: str) -> tuple[bool, str]:
        proc = self.components.get(name)
        return proc.stop() if proc else (False, f"Unknown component {name!r}.")

    def stop_all(self, wait: bool = False) -> None:
        for proc in self.components.values():
            proc.stop()
        if wait:
            deadline = time.time() + sum(STOP_TIMEOUTS_S) + 3
            while time.time() < deadline and any(p.alive for p in self.components.values()):
                time.sleep(0.2)

    def snapshot(self) -> list[dict]:
        return [p.snapshot() for p in self.components.values()]
