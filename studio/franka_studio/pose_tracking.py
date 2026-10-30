"""The studio's side of the object pose tracking node (../pose_tracking, ``config/pose_tracking.yaml``,
the Object poses component): which objects to track, its status, and the objects in the scene.

Objects are listed in the project's ``objects/objects.yaml`` (written by the studio's Objects card;
it replaces the list of ``env.yaml`` when present) or its ``env.yaml``::

    objects:
      - name: mug
        prompt: white mug                # text for the segmenter (default: name)
        mesh: objects/mug/mesh.ply       # relative to the project (default: objects/<name>/mesh.ply)
        scale: 1.0                       # mesh units -> m
      - box                              # just a name: prompt and mesh by default

* ``/pose_tracking/objects`` (JSON, latched): the active project's objects with absolute mesh paths
  (``mesh`` null when the file is missing: the node reports "scan it first").
* ``/pose_tracking/status`` (JSON): rate, segmentation time, per object state, shown on the page.
* ``/pose_tracking/scan`` (JSON): live scan commands (object_scan.py), meshes of unknown objects.
* ``/pose_tracking/poses`` (geometry_msgs/PoseArray, objects in list order): drawn in the viewer as
  scene poses ``object/<name>`` with their meshes (``scene_poses``), and the observation
  ``observation.object_poses`` when mapped.
"""

from __future__ import annotations

import json
import re
import threading
import time
from pathlib import Path

import yaml
from std_msgs.msg import String

STATUS_STALE_S = 3.0
AXES_LENGTH = 0.15                   # m, of the object frames in the viewer
OBJECTS_FILE = "objects/objects.yaml"   # in the project
POSE_COLORS = [(255, 99, 71), (65, 105, 225), (50, 205, 50), (255, 215, 0), (186, 85, 211), (0, 206, 209)]


def object_specs(objects: list | None, root: Path) -> list[dict]:
    """``objects`` of an env.yaml -> [{name, prompt, mesh (absolute | None), scale}]."""
    specs = []
    for entry in objects or []:
        entry = {"name": entry} if isinstance(entry, str) else dict(entry)
        name = str(entry["name"])
        mesh = Path(entry.get("mesh") or f"objects/{name}/mesh.ply")
        mesh = mesh if mesh.is_absolute() else root / mesh
        specs.append({"name": name, "prompt": str(entry.get("prompt") or name),
                      "mesh": str(mesh) if mesh.is_file() else None, "scale": float(entry.get("scale", 1.0))})
    return specs


def object_list(env_config: dict, root: Path) -> list:
    """The project's objects: ``objects/objects.yaml`` if it exists, else ``env.yaml`` ``objects``."""
    path = root / OBJECTS_FILE
    if path.is_file():
        return (yaml.safe_load(path.read_text()) or {}).get("objects") or []
    return list(env_config.get("objects") or [])


def clean_objects(entries: object) -> tuple[list[dict], str]:
    """Objects of the page -> ([{name, prompt}], error): names are folder names, unique."""
    out, seen = [], set()
    for e in entries if isinstance(entries, list) else []:
        e = {"name": e} if isinstance(e, str) else dict(e or {})
        name = str(e.get("name") or "").strip()
        prompt = str(e.get("prompt") or "").strip() or name
        if not name and not prompt:
            continue
        name = name or re.sub(r"[^A-Za-z0-9_-]+", "_", prompt).strip("_")
        if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]{0,63}", name):
            return [], f"Object name {name!r}: letters, digits, '_' or '-'."
        if name in seen:
            return [], f"Object {name!r} twice."
        seen.add(name)
        out.append({"name": name, "prompt": prompt})
    return out, ""


def save_object_list(root: Path, objects: list[dict]) -> Path:
    path = root / OBJECTS_FILE
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("# Objects to track (studio: Objects card). prompt: text for the segmenter; mesh by default\n"
                    "# objects/<name>/mesh.ply, made by the card's Scan (or ../pose_tracking `pixi run scan`).\n"
                    + yaml.safe_dump({"objects": objects}, sort_keys=False, allow_unicode=True))
    return path


def scene_poses(specs: list[dict], topic: str = "/pose_tracking/poses") -> list[dict]:
    """Scene ``poses`` entries drawing every object (frame + mesh) at its tracked pose."""
    return [{"name": f"object/{s['name']}", "topic": topic, "msgtype": "geometry_msgs/msg/PoseArray",
             "field": f"poses.{i}", "mesh": s["mesh"], "mesh_scale": s["scale"],
             "axes_length": AXES_LENGTH, "axes_radius": AXES_LENGTH / 20,
             "color": list(POSE_COLORS[i % len(POSE_COLORS)])}
            for i, s in enumerate(specs)]


def with_object_poses(env_config: dict, specs: list[dict], topic: str = "/pose_tracking/poses") -> dict:
    """A copy of ``env_config`` whose scene also draws the objects, and whose mappings of the poses
    topic take one pose per object (``n_poses``: the list decides, not the env.yaml)."""
    if not specs:
        return env_config
    scene = dict(env_config.get("scene") or {})
    scene["poses"] = list(scene.get("poses") or []) + scene_poses(specs, topic)
    mappings = {k: ({**m, "n_poses": len(specs)} if isinstance(m, dict) and m.get("topic") == topic else m)
                for k, m in (env_config.get("mappings") or {}).items()}
    return {**env_config, "scene": scene, **({"mappings": mappings} if "mappings" in env_config else {})}


class PoseTrackingLink:
    def __init__(self, ros: object, topics: dict | None = None) -> None:
        topics = {"status": "/pose_tracking/status", "objects": "/pose_tracking/objects",
                  "scan": "/pose_tracking/scan", **(topics or {})}
        self._lock = threading.Lock()
        self._status: dict | None = None
        self._status_t = 0.0
        self.specs: list[dict] = []
        ros.subscribe(String, topics["status"], self._on_status, depth=1)
        self._objects_pub = ros.publisher(String, topics["objects"], latched=True)
        self._scan_pub = ros.publisher(String, topics["scan"])

    def set_objects(self, specs: list[dict]) -> None:
        self.specs = specs
        self._objects_pub.publish(String(data=json.dumps({"objects": specs})))

    def send_scan(self, cmd: dict) -> None:
        self._scan_pub.publish(String(data=json.dumps(cmd)))

    def scan_status(self) -> dict | None:
        """The node's live scan progress (None when the node is not live)."""
        with self._lock:
            status, t = self._status, self._status_t
        return (status or {}).get("scan") if status and time.time() - t < STATUS_STALE_S else None

    def _on_status(self, msg: String) -> None:
        try:
            status = json.loads(msg.data)
        except ValueError:
            return
        with self._lock:
            self._status, self._status_t = status, time.time()

    def snapshot(self) -> dict:
        with self._lock:
            status, t = self._status, self._status_t
        age = time.time() - t if status else None
        return {"objects": [s["name"] for s in self.specs],
                "specs": [{"name": s["name"], "prompt": s["prompt"], "mesh": bool(s["mesh"])} for s in self.specs],
                "missing_mesh": [s["name"] for s in self.specs if not s["mesh"]],
                "status": status, "age": age, "live": age is not None and age < STATUS_STALE_S}

