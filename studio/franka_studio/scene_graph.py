"""The studio's side of the object detection node (../pose_tracking ``detect_node``,
``config/scene_graph.yaml``, the Object detection component): what to look for, and the objects and
scene graph it finds.

* ``/scene_graph/request`` (JSON, latched): ``{"enabled", "prompts"}`` -- the Objects card's Detect.
* ``/scene_graph/status`` (JSON): state, rate, per camera what it found, objects, relations.
* ``/scene_graph/points`` (PointCloud2): the objects' points in their colours (the Scene graph view).

The detected objects get the tracked poses (``/pose_tracking``) of the objects nearby: the frame is
drawn when the pose tracking has one.
"""

from __future__ import annotations

import json
import re
import threading
import time

import numpy as np
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String

from franka_studio.env.scene import decode_pointcloud2

STATUS_STALE_S = 3.0
MAX_PROMPTS = 16
POSE_MATCH_M = 0.08    # a tracked pose this close to a detected object's box centre is its frame


def clean_prompts(prompts: object) -> tuple[list[str], str]:
    """Prompts of the page -> ([prompt], error): trimmed, unique, at most MAX_PROMPTS."""
    if isinstance(prompts, str):
        prompts = re.split(r"[,\n;]", prompts)
    out: list[str] = []
    for p in prompts if isinstance(prompts, list) else []:
        p = re.sub(r"\s+", " ", str(p or "")).strip()
        if not p or p.lower() in (o.lower() for o in out):
            continue
        if len(p) > 80:
            return [], f"{p[:20]}…: at most 80 characters."
        out.append(p)
    if len(out) > MAX_PROMPTS:
        return [], f"At most {MAX_PROMPTS} objects."
    return out, ""


class SceneGraphLink:
    def __init__(self, ros: object, topics: dict | None = None) -> None:
        topics = {"request": "/scene_graph/request", "status": "/scene_graph/status",
                  "points": "/scene_graph/points", **(topics or {})}
        self._lock = threading.Lock()
        self._status: dict | None = None
        self._status_t = 0.0
        self._cloud: tuple | None = None   # (xyz, rgb, stamp)
        self.enabled = False
        self.prompts: list[str] = []
        ros.subscribe(String, topics["status"], self._on_status, depth=1)
        ros.subscribe(PointCloud2, topics["points"], self._on_points, depth=1)
        self._request_pub = ros.publisher(String, topics["request"], latched=True)

    def request(self, enabled: bool, prompts: list[str]) -> None:
        self.enabled, self.prompts = bool(enabled), list(prompts)
        self._request_pub.publish(String(data=json.dumps({"enabled": self.enabled, "prompts": self.prompts})))

    def _on_status(self, msg: String) -> None:
        try:
            status = json.loads(msg.data)
        except ValueError:
            return
        with self._lock:
            self._status, self._status_t = status, time.time()

    def _on_points(self, msg: PointCloud2) -> None:
        xyz, rgb = decode_pointcloud2(msg, 200_000)
        with self._lock:
            self._cloud = (xyz, rgb, time.time())

    def live_status(self) -> dict | None:
        with self._lock:
            status, t = self._status, self._status_t
        return status if status is not None and time.time() - t < STATUS_STALE_S else None

    def cloud(self) -> tuple | None:
        """(xyz, rgb, stamp) of the detected objects, None while detection is off or not live."""
        status = self.live_status()
        if not self.enabled or status is None or not status.get("enabled"):
            return None
        with self._lock:
            return self._cloud

    def graph(self, poses: dict | None = None) -> dict:
        """Objects and relations of the live status, each object with ``pose`` (the tracked pose name)
        when a tracked object is at it. ``poses``: {name: (position, wxyz)} of the tracked objects."""
        status = self.live_status()
        if not self.enabled or status is None or not status.get("enabled"):
            return {"objects": [], "relations": []}
        objects = [dict(o) for o in status.get("objects") or []]
        free = dict(poses or {})
        for o in objects:
            box = o.get("box") or {}
            if not free or "center" not in box:
                continue
            c = np.asarray(box["center"], float)
            name, d = min(((n, float(np.linalg.norm(np.asarray(p[0]) - c))) for n, p in free.items()),
                          key=lambda x: x[1])
            if d < max(POSE_MATCH_M, 0.5 * max(box.get("size", [0, 0])[:2])):
                o["pose"] = name
                free.pop(name)
        return {"objects": objects, "relations": list(status.get("relations") or [])}

    def snapshot(self, poses: dict | None = None) -> dict:
        status = self.live_status()
        out = {"enabled": self.enabled, "prompts": self.prompts, "live": status is not None}
        if status is not None:
            out.update({k: status.get(k) for k in ("state", "hz", "seg_ms", "fuse_ms", "cameras", "error",
                                                   "model_loaded", "views")})
            out["node_enabled"] = bool(status.get("enabled"))
        out.update(self.graph(poses))
        return out
