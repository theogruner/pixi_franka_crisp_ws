"""Runs a remote policy on a SceneEnv with action chunking.

Same execution semantics as franka_client's RobiClient: a control loop at ``frequency`` pops one
action per tick and executes it with ``env.step``; when the queue is down to
``chunk_size_threshold`` actions a new observation (``env.observe(prompt)``) is sent; incoming
chunks replace queued actions of the same timestep and drop those already executed.
"""

from __future__ import annotations

import logging
import math
import threading
import time
from queue import Empty, Queue

import numpy as np

from franka_studio.env import SceneEnv
from franka_studio.policy.registry import PolicySpec
from franka_studio.policy.remote import RemotePolicy, TimedAction, TimedObservation

log = logging.getLogger(__name__)

UNLOADED, LOADING, READY, RUNNING, ERROR = "unloaded", "loading", "ready", "running", "error"


class PolicyRunner:
    """One loaded policy: its env, its server connection and the run state."""

    def __init__(self, spec: PolicySpec, env: SceneEnv) -> None:
        self.spec = spec
        self.env = env
        cfg = spec.config
        pcfg = cfg.get("policy") or {}
        self.frequency = float(cfg.get("frequency", 30.0))
        self.actions_per_chunk = int(pcfg.get("actions_per_chunk", 32))
        act_while_inference = max(1, math.ceil(self.frequency / float(cfg.get("policy_inference_fps", 10.0))))
        self.threshold = int(pcfg.get("chunk_size_threshold", self.actions_per_chunk - act_while_inference))
        self.state = LOADING
        self.error = ""
        self.prompt = ""
        self.remote: RemotePolicy | None = None
        self._queue: Queue[TimedAction] = Queue()
        self._qlock = threading.Lock()
        self._latest_action = -1
        self._must_go = threading.Event()
        self._must_go.set()
        self._active = threading.Event()
        self._alive = True
        self.stats = {"steps": 0, "chunks": 0, "observations": 0, "run_started": None,
                      "last_chunk_at": None, "inference_hz": None, "control_hz": None}
        self._tick_times: list[float] = []
        self._chunk_times: list[float] = []

    # ---- lifecycle --------------------------------------------------------------------------------

    def load(self) -> None:
        """Connects and makes the server load the checkpoint (blocking; run in a worker)."""
        try:
            self.remote = RemotePolicy(self.spec.server_address, self.spec.config.get("policy") or {},
                                       self.frequency, log)
            self.remote.connect()
        except Exception as e:
            self.state, self.error = ERROR, f"Could not reach the policy server at {self.spec.server_address}: {e}"
            return
        threading.Thread(target=self._receive_loop, daemon=True, name="policy-recv").start()
        threading.Thread(target=self._control_loop, daemon=True, name="policy-ctrl").start()
        self.state = READY

    def run(self, prompt: str) -> tuple[bool, str]:
        if self.state not in (READY, RUNNING):
            return False, f"Policy is {self.state}."
        if self.spec.language and not prompt.strip():
            return False, "This policy needs a prompt."
        self.prompt = prompt.strip() if self.spec.language else (self.spec.prompts[0] if self.spec.prompts else "")
        self._drain()
        self._must_go.set()
        self.stats.update(steps=0, chunks=0, observations=0, run_started=time.time())
        self._active.set()
        self.state = RUNNING
        return True, "Policy running."

    def stop(self) -> tuple[bool, str]:
        was = self._active.is_set()
        self._active.clear()
        self._drain()
        if self.state == RUNNING:
            self.state = READY
        return was, "Policy stopped." if was else "Policy was not running."

    def unload(self) -> None:
        self.stop()
        self._alive = False
        if self.remote:
            self.remote.close()
        self.state = UNLOADED

    @property
    def running(self) -> bool:
        return self._active.is_set()

    # ---- loops --------------------------------------------------------------------------------------

    def _drain(self) -> None:
        with self._qlock:
            self._queue = Queue()
            self._latest_action = -1

    def _receive_loop(self) -> None:
        while self._alive:
            try:
                actions = self.remote.get_actions()
            except Exception as e:
                if not self._alive:
                    return
                log.warning("receiving actions failed: %s", e)
                time.sleep(0.5)
                continue
            if not actions or not self._active.is_set():
                continue
            self._aggregate(actions)
            now = time.time()
            self._chunk_times = [t for t in self._chunk_times if now - t < 5.0] + [now]
            self.stats["chunks"] += 1
            self.stats["last_chunk_at"] = now
            self._must_go.set()

    def _aggregate(self, incoming: list[TimedAction]) -> None:
        """Newer chunk wins for timesteps already queued; executed timesteps are dropped."""
        with self._qlock:
            latest = self._latest_action
            queued = {a.get_timestep(): a for a in list(self._queue.queue)}
            for a in incoming:
                if a.get_timestep() > latest:
                    queued[a.get_timestep()] = a
            fresh: Queue[TimedAction] = Queue()
            for ts in sorted(queued):
                fresh.put(queued[ts])
            self._queue = fresh

    def _control_loop(self) -> None:
        period = 1.0 / self.frequency
        next_t = time.perf_counter()
        while self._alive:
            next_t += period
            if self._active.is_set():
                try:
                    self._tick()
                except Exception as e:  # keep the loop alive, surface the problem
                    self.error = f"control loop: {e}"
                    log.exception("control tick failed")
            delay = next_t - time.perf_counter()
            if delay > 0:
                time.sleep(delay)
            else:
                next_t = time.perf_counter()

    def _tick(self) -> None:
        action: TimedAction | None = None
        with self._qlock:
            try:
                action = self._queue.get_nowait()
            except Empty:
                pass
            qsize = self._queue.qsize()
        if action is not None:
            self.env.step(np.asarray(action.get_action()), task=self.prompt or None)
            with self._qlock:
                self._latest_action = action.get_timestep()
            self.stats["steps"] += 1
            now = time.time()
            self._tick_times = [t for t in self._tick_times if now - t < 2.0] + [now]
        if qsize <= self.threshold:
            obs = self.env.observe(self.prompt)
            if obs is None:
                return
            with self._qlock:
                timestep = max(self._latest_action, 0)
                must_go = self._must_go.is_set() and self._queue.empty()
            try:
                self.remote.send_observation(TimedObservation(time.time(), timestep, obs, must_go))
                self.stats["observations"] += 1
            except Exception as e:
                self.error = f"sending observation failed: {e}"
                return
            if must_go:
                self._must_go.clear()

    def snapshot(self) -> dict:
        def rate(ts: list[float], window: float) -> float | None:
            ts = [t for t in ts if time.time() - t < window]
            return round(len(ts) / window, 1) if ts else None

        with self._qlock:
            queued = self._queue.qsize()
        return {
            "id": self.spec.id, "name": self.spec.name, "kind": self.spec.kind, "state": self.state,
            "error": self.error, "prompt": self.prompt, "queued": queued, "threshold": self.threshold,
            "actions_per_chunk": self.actions_per_chunk, "frequency": self.frequency,
            "steps": self.stats["steps"], "chunks": self.stats["chunks"],
            "observations": self.stats["observations"],
            "run_time": (time.time() - self.stats["run_started"]) if self.running and self.stats["run_started"] else None,
            "control_hz": rate(self._tick_times, 2.0), "inference_hz": rate(self._chunk_times, 5.0),
        }
