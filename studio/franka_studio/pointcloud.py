"""The studio's side of the point cloud node on the Jetson (Fast-FoundationStereo, see
``config/cameras.yaml`` ``pointcloud``): its status, the camera extrinsics it merges with and
the alignment of static cameras.

* ``/ffs/status`` (JSON): rate, points, per-camera timing and errors, shown on the page.
* ``/ffs/extrinsics`` (JSON, latched): where every camera is, from the active project's
  ``scene.cameras``: a URDF link is a TF frame for the node (wrist cameras), a fixed pose is sent
  as is (static cameras). While a camera is being placed in the viewer its unsaved pose is sent
  too, so the merged cloud moves with the gizmo.
* ``/ffs/align/request`` -> ``/ffs/align/result`` (JSON): ICP of one camera's cloud against the
  others, run on the Jetson.
"""

from __future__ import annotations

import itertools
import json
import threading
import time

from std_msgs.msg import String

STATUS_STALE_S = 3.0
PREVIEW_PERIOD_S = 0.2


def camera_extrinsics(cameras: dict) -> dict:
    """``scene.cameras`` -> the node's extrinsics: {camera: {frame} | {position, wxyz}}."""
    out = {}
    for name, cam in (cameras or {}).items():
        cam = cam or {}
        if cam.get("link"):
            out[name] = {"frame": cam["link"]}
        elif "position" in cam:
            out[name] = {"position": [float(v) for v in cam["position"]],
                         "wxyz": [float(v) for v in cam.get("wxyz", (1, 0, 0, 0))]}
    return out


class PointCloudLink:
    def __init__(self, ros: object, topics: dict | None = None) -> None:
        topics = {"status": "/ffs/status", "extrinsics": "/ffs/extrinsics", "align": "/ffs/align", **(topics or {})}
        self.ros = ros
        self._lock = threading.Lock()
        self._status: dict | None = None
        self._status_t = 0.0
        self._base: dict = {}
        self._overrides: dict = {}
        self._last_preview = 0.0
        self._pending_preview: threading.Timer | None = None
        self._ids = itertools.count(1)
        self._results: dict[int, dict] = {}
        self._result_cv = threading.Condition()
        ros.subscribe(String, topics["status"], self._on_status, depth=1)
        ros.subscribe(String, topics["align"] + "/result", self._on_result)
        self._ext_pub = ros.publisher(String, topics["extrinsics"], latched=True)
        self._align_pub = ros.publisher(String, topics["align"] + "/request")

    # ---- status ------------------------------------------------------------------------------------

    def _on_status(self, msg: String) -> None:
        try:
            status = json.loads(msg.data)
        except ValueError:
            return
        with self._lock:
            self._status, self._status_t = status, time.time()

    def snapshot(self) -> dict | None:
        with self._lock:
            if self._status is None:
                return None
            age = time.time() - self._status_t
            return {**self._status, "age": age, "live": age < STATUS_STALE_S}

    # ---- extrinsics ------------------------------------------------------------------------------------

    def set_cameras(self, cameras: dict) -> None:
        """The saved camera poses of the active env; drops unsaved previews."""
        with self._lock:
            self._base = camera_extrinsics(cameras)
            self._overrides.clear()
        self._publish()

    def preview(self, name: str, position: list, wxyz: list) -> None:
        """An unsaved pose of ``name`` (gizmo / alignment), sent at most every PREVIEW_PERIOD_S."""
        with self._lock:
            self._overrides[name] = {"position": [float(v) for v in position], "wxyz": [float(v) for v in wxyz]}
            due = time.time() - self._last_preview >= PREVIEW_PERIOD_S
            if not due and self._pending_preview is None:  # the last move still gets sent
                self._pending_preview = threading.Timer(PREVIEW_PERIOD_S, self._flush_preview)
                self._pending_preview.daemon = True
                self._pending_preview.start()
        if due:
            self._publish()

    def _flush_preview(self) -> None:
        with self._lock:
            self._pending_preview = None
        self._publish()

    def _publish(self) -> None:
        with self._lock:
            ext = {**self._base, **self._overrides}
            self._last_preview = time.time()
        self._ext_pub.publish(String(data=json.dumps(ext)))

    # ---- alignment -----------------------------------------------------------------------------------

    def _on_result(self, msg: String) -> None:
        try:
            result = json.loads(msg.data)
        except ValueError:
            return
        with self._result_cv:
            self._results[result.get("id")] = result
            self._result_cv.notify_all()

    def align(self, name: str, position: list, wxyz: list, timeout_s: float = 30.0) -> dict:
        """Refines ``name``'s world pose by ICP on the Jetson; blocks until the answer."""
        status = self.snapshot()
        if not status or not status.get("live"):
            return {"ok": False, "message": "The point cloud node is not running."}
        rid = next(self._ids)
        self._align_pub.publish(String(data=json.dumps(
            {"id": rid, "camera": name, "position": [float(v) for v in position], "wxyz": [float(v) for v in wxyz]})))
        deadline = time.time() + timeout_s
        with self._result_cv:
            while rid not in self._results:
                left = deadline - time.time()
                if left <= 0:
                    return {"ok": False, "message": f"No answer from the point cloud node within {timeout_s:.0f} s."}
                self._result_cv.wait(left)
            return self._results.pop(rid)
