"""Gym-style env over the real robot scene.

A ``SceneEnv`` is built from the same config a policy client uses:

* ``mappings`` -> the observation: one entry per feature key (``observation.state``,
  ``observation.images.<cam>``, ...), each fed by one or more ROS topics through the client's
  message extractors (crop/resize/encoding included). Several topics under one key are
  concatenated, exactly like the policy client does.
* ``actions`` -> the action: slices of the action vector turned into ROS messages by the
  client's action processors.
* ``home_pose`` -> ``reset()``.
* ``scene`` -> extra information that is not part of the observation (robot models, poses,
  point clouds; see ``env/scene.py``), returned by ``render()``.

So ``env.observe()`` is literally what the policy is sent and ``env.step(action)`` is what is done
with its output. Every ``step`` (and every passive ``tick`` while nothing acts) is broadcast to
listeners as a ``StepRecord``; the 3D viewer is one of them, which is what makes it a debugger for
"what does the policy see / what are we recording".

The interface follows gymnasium (``reset``/``step``/``render``/``close``, ``observation_space``,
``action_space``), but a real robot has no reward or termination: those are always 0 / False.
"""

from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass, field
from typing import Callable

import numpy as np
from geometry_msgs.msg import PoseStamped
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from franka_studio.env import spaces
from franka_studio.env.scene import SceneSensors, SceneState
from franka_studio.ros import registry
from franka_studio.ros.msgs import import_msg
from franka_studio.ros.qos import AdaptiveSubscriptions

log = logging.getLogger(__name__)

STALE_AFTER_S = 1.0


@dataclass
class StepRecord:
    """One observation (and the action taken on it, if any), as broadcast to listeners."""

    step: int
    t: float
    mode: str                       # "policy" (env.step) or "monitor" (passive tick)
    observation: dict | None        # None while a feature has not been received yet
    action: np.ndarray | None = None
    info: dict = field(default_factory=dict)


@dataclass
class _Feature:
    key: str
    parts: list = field(default_factory=list)   # [(topic, msgtype, extractor)]
    values: list = field(default_factory=list)  # latest array per part
    stamps: list = field(default_factory=list)  # receive time per part
    is_image: bool = False
    errors: int = 0


class SceneEnv:
    """The scene as a policy sees it, plus the scene info around it."""

    metadata = {"render_modes": ["scene"]}

    def __init__(self, runtime: object, config: dict, name: str = "scene") -> None:
        self.config = config
        self.name = name
        self.frequency = float(config.get("frequency", 30.0))
        self.runtime = runtime
        slug = "".join(c if c.isalnum() else "_" for c in name.lower())[:40]
        self.node = Node(f"studio_env_{slug}_{int(time.time() * 1000) % 100000}")
        self._cb = ReentrantCallbackGroup()
        self._lock = threading.Lock()
        self._listeners: list[Callable[[StepRecord], None]] = []
        self._step = 0
        self._features: dict[str, _Feature] = {}
        self._pubs: list = []
        self._processors: list = []
        self.warnings: list[str] = []
        self.last_action: np.ndarray | None = None
        self.subs = AdaptiveSubscriptions(self.node, self._cb)

        self._setup_observations(config.get("mappings") or {})
        self._setup_actions(config.get("actions") or [])
        self._setup_home(config.get("home_pose"))
        self.scene = SceneSensors(self.node, config.get("scene"), self.subs)
        self.observation_space = spaces.Dict(self._observation_spaces())
        self.action_space = spaces.box((self.action_dim,), "float32") if self._processors else None
        runtime.add_node(self.node)

    # ---- setup ----------------------------------------------------------------------------------

    def _setup_observations(self, mappings: dict) -> None:
        for key, mapping in mappings.items():
            parts = mapping if isinstance(mapping, list) else [mapping]
            feat = _Feature(key=key, is_image=".images." in key or key.startswith("observation.image"))
            for idx, params in enumerate(parts):
                params = dict(params)
                msgtype, topic = params.pop("msgtype"), params.pop("topic")
                qos = params.pop("qos", "auto")
                cls = registry.extractor_class(msgtype)
                if cls is None:
                    self.warnings.append(f"{key}: no extractor for {msgtype}")
                    continue
                try:
                    extractor = cls(**params)
                    msg_cls = import_msg(msgtype)
                except Exception as e:
                    self.warnings.append(f"{key}: {e}")
                    continue
                feat.parts.append((topic, msgtype, extractor))
                feat.values.append(None)
                feat.stamps.append(0.0)
                self.subs.add(msg_cls, topic, lambda msg, f=feat, i=len(feat.parts) - 1: self._on_msg(msg, f, i), qos)
            if feat.parts:
                self._features[key] = feat

    def _on_msg(self, msg: object, feat: _Feature, idx: int) -> None:
        try:
            value = feat.parts[idx][2].extract(msg)
        except Exception as e:
            feat.errors += 1
            if feat.errors in (1, 100):
                log.warning("%s: extractor failed: %s", feat.key, e)
            return
        if value is None:
            return
        with self._lock:
            feat.values[idx] = value
            feat.stamps[idx] = time.time()

    def _setup_actions(self, actions: list) -> None:
        self.action_dim = 0
        for cfg in actions:
            cls = registry.processor_class(cfg.get("type", ""))
            if cls is None:
                self.warnings.append(f"action: unknown processor type {cfg.get('type')!r}")
                continue
            try:
                processor = cls(**{"frequency": self.frequency, **cfg})
                pub = self.node.create_publisher(import_msg(cfg["msgtype"]), cfg["topic"], 1)
            except Exception as e:
                self.warnings.append(f"action {cfg.get('type')}: {e}")
                continue
            self._processors.append(processor)
            self._pubs.append(pub)
            self.action_dim = max(self.action_dim, int(cfg.get("end_idx", 0)))

    def _setup_home(self, home: dict | list | None) -> None:
        """Per entry a joint-space home when the arm has home_joints (the entry's own, or its
        teleop config's; franka_client/joint_home.py), else the Cartesian pose on ``topic``."""
        self._home = [home] if isinstance(home, dict) else list(home or [])
        self._home_pubs = []
        self._joint_homes = []
        for h in self._home:
            # Arm namespace: ``ns``, or from the topic (/franka_right/target_pose -> franka_right)
            ns = h.get("ns") or (h.get("topic") or "").strip("/").split("/")[0]
            joint_home = None
            if ns:
                try:
                    from franka_client.joint_home import JointHome, load_home
                    cfg = {k: h[k] for k in ("home_joints", "home_joint_max_speed", "home_joint_min_duration") if k in h}
                    spec = load_home(ns, cfg if "home_joints" in cfg else None)
                    if spec is not None:
                        joint_home = JointHome(self.node, ns, spec, callback_group=ReentrantCallbackGroup())
                except Exception as e:
                    log.warning("joint home of %s disabled: %s", ns, e)
            self._joint_homes.append(joint_home)
            pose_pub = (self.node.create_publisher(PoseStamped, h["topic"], 1)
                        if h.get("topic") and joint_home is None else None)
            grip_pub = (self.node.create_publisher(JointTrajectory, h["gripper_topic"], 1)
                        if h.get("gripper_topic") else None)
            self._home_pubs.append((pose_pub, grip_pub))

    def _observation_spaces(self) -> dict:
        out = {}
        for key, feat in self._features.items():
            try:
                shapes = [p[2].shape for p in feat.parts]
                dtype = str(feat.parts[0][2].dtype)
                if feat.is_image or dtype == "video":
                    h, w, c = shapes[0]
                    out[key] = spaces.box((c, h, w), "video")  # extractors return CHW
                elif len(shapes) == 1 and len(shapes[0]) > 1:  # e.g. a point cloud (N, 3 | 6)
                    out[key] = spaces.box(tuple(int(v) for v in shapes[0]), dtype)
                else:
                    out[key] = spaces.box((int(sum(s[0] for s in shapes)),), "float32")
            except Exception:
                out[key] = spaces.Box(shape=(), dtype="unknown")
        if self.is_language_conditioned:
            out["task"] = spaces.Text()
        return out

    @property
    def is_language_conditioned(self) -> bool:
        return bool(self.config.get("_language", False))

    # ---- gym API ----------------------------------------------------------------------------

    def observe(self, task: str | None = None) -> dict | None:
        """The current observation in the policy's format, or None if a feature is missing."""
        obs: dict = {}
        with self._lock:
            for key, feat in self._features.items():
                if any(v is None for v in feat.values):
                    return None
                if len(feat.values) == 1:
                    obs[key] = feat.values[0]
                else:
                    obs[key] = np.concatenate(feat.values, axis=-1).astype(np.float32)
        if task is not None:
            obs["task"] = task
        return obs

    def step(self, action: np.ndarray, task: str | None = None) -> tuple:
        """Sends ``action`` to the robot and returns ``(obs, reward, terminated, truncated, info)``."""
        action = np.asarray(action)
        stamp = self.node.get_clock().now().to_msg()
        for pub, processor in zip(self._pubs, self._processors):
            msg = processor.process(action, stamp)
            if msg is not None:
                pub.publish(msg)
        self.last_action = action
        obs = self.observe(task)
        info = {"missing": self.missing() if obs is None else []}
        self._emit("policy", obs, action, info)
        return obs, 0.0, False, False, info

    def reset(self, *, seed: int | None = None, options: dict | None = None) -> tuple:
        """Sends the configured home pose(s) and returns ``(obs, info)``."""
        self.home()
        return self.observe(), {"homed": bool(self._home)}

    def render(self) -> SceneState:
        return self.scene.snapshot()

    def close(self) -> None:
        self.stop_home()
        self._listeners.clear()
        self.runtime.retire_node(self.node)

    # ---- extras -----------------------------------------------------------------------------------

    def tick(self, task: str | None = None) -> StepRecord:
        """Passive step: broadcasts the current observation without acting."""
        obs = self.observe(task)
        return self._emit("monitor", obs, None, {"missing": self.missing() if obs is None else []})

    def home(self) -> bool:
        stamp = self.node.get_clock().now().to_msg()
        for h, (pose_pub, grip_pub), joint_home in zip(self._home, self._home_pubs, self._joint_homes):
            if joint_home is not None:
                joint_home.start()
            elif pose_pub is not None:
                msg = PoseStamped()
                msg.header.stamp, msg.header.frame_id = stamp, h.get("frame_id", "world")
                p = h.get("position", [0.3, 0.0, 0.5])
                q = h.get("orientation", [1.0, 0.0, 0.0, 0.0])  # x y z w, like the client
                msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = map(float, p)
                (msg.pose.orientation.x, msg.pose.orientation.y,
                 msg.pose.orientation.z, msg.pose.orientation.w) = map(float, q)
                pose_pub.publish(msg)
            if grip_pub is not None:
                traj = JointTrajectory()
                traj.header.stamp = stamp
                traj.joint_names = [h.get("gripper_joint_name", "")]
                point = JointTrajectoryPoint()
                point.positions = [float(h.get("gripper_open_val", 0.0))]
                point.time_from_start.nanosec = int(1e9 / self.frequency)
                traj.points = [point]
                grip_pub.publish(traj)
        return bool(self._home)

    def stop_home(self) -> bool:
        """Aborts running joint homings; True if one was running."""
        running = [j for j in self._joint_homes if j is not None and j.active]
        for j in running:
            j.stop("stopped from the studio")
        return bool(running)

    def home_message(self) -> str:
        return " ".join(j.message for j in self._joint_homes if j is not None and j.message)

    def missing(self) -> list[str]:
        with self._lock:
            return [f"{k} ← {feat.parts[i][0]}" for k, feat in self._features.items()
                    for i, v in enumerate(feat.values) if v is None]

    def feature_status(self) -> list[dict]:
        """Per feature: topics, age of the newest message, shape."""
        now = time.time()
        out = []
        with self._lock:
            for key, feat in self._features.items():
                ages = [now - s if s else None for s in feat.stamps]
                worst = None if any(a is None for a in ages) else max(ages)
                shape = None
                if all(v is not None for v in feat.values):
                    shape = (list(feat.values[0].shape) if len(feat.values) == 1
                             else [int(sum(v.shape[-1] for v in feat.values))])
                out.append({
                    "key": key, "image": feat.is_image, "topics": [p[0] for p in feat.parts],
                    "age": worst, "ok": worst is not None and worst < STALE_AFTER_S, "shape": shape,
                })
        return out

    def describe(self) -> dict:
        return {
            "name": self.name, "frequency": self.frequency, "action_dim": self.action_dim,
            "observation_space": spaces.describe(self.observation_space),
            "actions": [{"type": c.get("type"), "topic": c.get("topic"),
                         "slice": [c.get("start_idx"), c.get("end_idx")]}
                        for c in self.config.get("actions") or []],
            "warnings": self.warnings,
        }

    def add_listener(self, fn: Callable[[StepRecord], None]) -> None:
        self._listeners.append(fn)

    def _emit(self, mode: str, obs: dict | None, action: np.ndarray | None, info: dict) -> StepRecord:
        self._step += 1
        record = StepRecord(self._step, time.time(), mode, obs, action, info)
        for fn in list(self._listeners):
            try:
                fn(record)
            except Exception as e:  # a broken viewer must never stop the robot
                log.warning("step listener failed: %s", e)
        return record
