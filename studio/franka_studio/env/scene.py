"""Scene information beyond the policy observation: robot models, poses and point clouds.

Configured by the ``scene`` section of an env/policy config::

    scene:
      robots:                       # drawn from their URDF, posed by the joint states
        - name: franka_right
          description_topic: /franka_right/robot_description   # or urdf_file: path/to.urdf
          joint_states: [/franka_right/joint_states]
      frames: [franka_right_grasp_point]                     # URDF links drawn as axes
      poses:                        # any pose on a topic, e.g. EE / commanded targets
        - name: right_ee
          topic: /franka_right/fijk_debug
          msgtype: franka_custom_msgs/msg/FIJKDebug
          field: actual_pose        # attribute path inside the message ('' = the message)
          mesh: null                # optional mesh file drawn at the pose
      point_clouds:                 # sensor_msgs/PointCloud2
        - name: middle
          topic: /zed_middle/point_cloud
          max_points: 30000
          pose: {position: [0, 0, 0], wxyz: [1, 0, 0, 0]}   # cloud frame in world, or
          attach: {robot: franka_right, link: franka_right_fr3_link8}
      rig: {topic: /zed_rig/synced_raw_snapshot}      # every camera of the rig is shown
      camera_topics: {wrist_cam: /cam/image_raw}      # plain sensor_msgs/Image cameras
      cameras:                      # where to draw a camera (rig name, topic name or obs key)
        zed_right_arm: {robot: franka_right, link: zed_left_camera_optical_frame}
        zed_middle: {position: [x, y, z], wxyz: [w, x, y, z]}   # fixed pose; fov_deg optional

Everything is optional; a missing topic just leaves that element out of the view.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field

import numpy as np
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, JointState, PointCloud2
from std_msgs.msg import String

from franka_studio.ros.msgs import import_msg

LATCHED_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST, depth=1)

CAMERA_WIDTH = 320          # scene camera images are kept downscaled to this width
CAMERA_PERIOD_S = 0.1       # and decoded at most this often per camera

_PC2_DTYPES = {1: np.int8, 2: np.uint8, 3: np.int16, 4: np.uint16, 5: np.int32, 6: np.uint32,
               7: np.float32, 8: np.float64}


@dataclass
class Pose:
    """Position and wxyz quaternion in the world frame."""

    position: np.ndarray
    wxyz: np.ndarray
    stamp: float = 0.0


@dataclass
class SceneState:
    """A snapshot of everything the scene knows, for the viewer."""

    robots: dict = field(default_factory=dict)        # name -> {"urdf": str|None, "joints": {name: q}}
    poses: dict = field(default_factory=dict)         # name -> Pose
    point_clouds: dict = field(default_factory=dict)  # name -> (points Nx3, colors Nx3 uint8, stamp)
    cameras: dict = field(default_factory=dict)       # name -> (rgb HxWx3 uint8, fov_y rad|None, stamp)


def to_pose(obj: object, stamp: float) -> Pose | None:
    """Pose from a geometry_msgs Pose/PoseStamped/Transform(-Stamped) or a [x y z qx qy qz qw] array."""
    for attr in ("pose", "transform"):
        inner = getattr(obj, attr, None)
        if inner is not None and not isinstance(inner, (list, tuple, np.ndarray)):
            obj = inner
    pos = getattr(obj, "position", None) or getattr(obj, "translation", None)
    rot = getattr(obj, "orientation", None) or getattr(obj, "rotation", None)
    if pos is not None and rot is not None:
        return Pose(np.array([pos.x, pos.y, pos.z]), np.array([rot.w, rot.x, rot.y, rot.z]), stamp)
    arr = np.asarray(obj, dtype=np.float64).ravel() if isinstance(obj, (list, tuple, np.ndarray)) else None
    if arr is not None and arr.size >= 7:
        return Pose(arr[:3], np.array([arr[6], arr[3], arr[4], arr[5]]), stamp)
    return None


def image_to_rgb(msg: Image, width: int = CAMERA_WIDTH) -> np.ndarray | None:
    """A sensor_msgs/Image as a downscaled RGB uint8 array (None for unsupported encodings)."""
    import cv2

    enc = msg.encoding.lower()
    channels = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4, "mono8": 1}.get(enc)
    if channels is None:
        return None
    rows = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)  # step may pad rows
    img = rows[:, : msg.width * channels].reshape(msg.height, msg.width, channels)
    if msg.width > width:
        img = cv2.resize(img, (width, round(msg.height * width / msg.width)), interpolation=cv2.INTER_AREA)
    conv = {"bgr8": cv2.COLOR_BGR2RGB, "rgba8": cv2.COLOR_RGBA2RGB, "bgra8": cv2.COLOR_BGRA2RGB,
            "mono8": cv2.COLOR_GRAY2RGB}.get(enc)
    return np.ascontiguousarray(cv2.cvtColor(img, conv) if conv else img)


def fov_from_info(info: object) -> float | None:
    """Vertical field of view from a CameraInfo (None if it carries no intrinsics)."""
    fy = float(info.k[4]) if len(info.k) > 4 else 0.0
    return float(2 * np.arctan(info.height / (2 * fy))) if fy > 0 and info.height else None


def get_path(msg: object, path: str) -> object:
    for part in filter(None, (path or "").split(".")):
        msg = getattr(msg, part)
    return msg


def decode_pointcloud2(msg: PointCloud2, max_points: int) -> tuple[np.ndarray, np.ndarray]:
    """Xyz (float32 Nx3) and rgb (uint8 Nx3) from a PointCloud2, NaNs dropped, subsampled."""
    fields = {f.name: f for f in msg.fields}
    dtype = np.dtype({
        "names": [f.name for f in msg.fields],
        "formats": [np.dtype(_PC2_DTYPES[f.datatype]).newbyteorder(">" if msg.is_bigendian else "<")
                    for f in msg.fields],
        "offsets": [f.offset for f in msg.fields],
        "itemsize": msg.point_step,
    })
    n = msg.width * msg.height
    data = np.frombuffer(bytes(msg.data), dtype=dtype, count=n)
    xyz = np.stack([data["x"], data["y"], data["z"]], axis=-1).astype(np.float32)
    ok = np.isfinite(xyz).all(axis=1)
    if "rgb" in fields or "rgba" in fields:
        key = "rgb" if "rgb" in fields else "rgba"
        packed = np.ascontiguousarray(data[key]).view(np.uint32)
        rgb = np.stack([(packed >> 16) & 255, (packed >> 8) & 255, packed & 255], axis=-1).astype(np.uint8)
    else:  # colour by height
        z = xyz[:, 2]
        t = np.clip((z - np.nanmin(z[ok]) if ok.any() else 0) / 1.0, 0, 1)
        rgb = np.stack([60 + 150 * t, 200 - 80 * t, 255 - 60 * t], axis=-1).astype(np.uint8)
    xyz, rgb = xyz[ok], rgb[ok]
    if len(xyz) > max_points:
        idx = np.random.default_rng(0).choice(len(xyz), max_points, replace=False)
        xyz, rgb = xyz[idx], rgb[idx]
    return xyz, rgb


class SceneSensors:
    """Subscribes to the scene topics on ``node`` and keeps the latest values."""

    def __init__(self, node: Node, cfg: dict | None, subs: object) -> None:
        self.cfg = cfg or {}
        self._lock = threading.Lock()
        self._robots: dict[str, dict] = {}
        self._poses: dict[str, Pose] = {}
        self._clouds: dict[str, tuple] = {}
        self._cameras: dict[str, tuple] = {}
        self._camera_decoded: dict[str, float] = {}
        self.pose_cfg = {p["name"]: p for p in self.cfg.get("poses") or []}
        self.cloud_cfg = {c["name"]: c for c in self.cfg.get("point_clouds") or []}
        kw = {"callback_group": subs.group}

        for robot in self.cfg.get("robots") or []:
            name = robot["name"]
            urdf = None
            if robot.get("urdf_file"):
                urdf = _load_urdf_file(robot["urdf_file"])
            self._robots[name] = {"urdf": urdf, "joints": {}, "stamp": 0.0}
            if robot.get("description_topic") and urdf is None:
                node.create_subscription(String, robot["description_topic"],
                                         lambda m, n=name: self._on_description(n, m), LATCHED_QOS, **kw)
            for topic in robot.get("joint_states") or []:
                subs.add(JointState, topic, lambda m, n=name: self._on_joints(n, m))

        for name, p in self.pose_cfg.items():
            cls = import_msg(p.get("msgtype", "geometry_msgs/msg/PoseStamped"))
            subs.add(cls, p["topic"], lambda m, n=name, f=p.get("field", ""): self._on_pose(n, m, f))

        for name, c in self.cloud_cfg.items():
            subs.add(PointCloud2, c["topic"],
                     lambda m, n=name, mx=int(c.get("max_points", 30000)): self._on_cloud(n, m, mx))

        rig = self.cfg.get("rig")
        if rig:
            from franka_custom_msgs.msg import RigSnapshot

            subs.add(RigSnapshot, rig.get("topic", "/zed_rig/synced_raw_snapshot"), self._on_rig)
        for name, topic in (self.cfg.get("camera_topics") or {}).items():
            subs.add(Image, topic, lambda m, n=name: self._on_camera(n, m, None))

    def _on_description(self, name: str, msg: String) -> None:
        with self._lock:
            self._robots[name]["urdf"] = msg.data

    def _on_joints(self, name: str, msg: JointState) -> None:
        with self._lock:
            joints = self._robots[name]["joints"]
            for jn, q in zip(msg.name, msg.position):
                joints[jn] = float(q)
            self._robots[name]["stamp"] = time.time()

    def _on_pose(self, name: str, msg: object, path: str) -> None:
        try:
            pose = to_pose(get_path(msg, path), time.time())
        except AttributeError:
            pose = None
        if pose is not None:
            with self._lock:
                self._poses[name] = pose

    def _on_cloud(self, name: str, msg: PointCloud2, max_points: int) -> None:
        try:
            xyz, rgb = decode_pointcloud2(msg, max_points)
        except Exception:
            return
        with self._lock:
            self._clouds[name] = (xyz, rgb, time.time())

    def _due(self, name: str) -> bool:
        now = time.time()
        if now - self._camera_decoded.get(name, 0.0) < CAMERA_PERIOD_S:
            return False
        self._camera_decoded[name] = now
        return True

    def _on_camera(self, name: str, msg: Image, info: object) -> None:
        if not self._due(name):
            return
        try:
            img = image_to_rgb(msg)
        except Exception:
            return
        if img is not None:
            with self._lock:
                self._cameras[name] = (img, fov_from_info(info) if info is not None else None, time.time())

    def _on_rig(self, msg: object) -> None:
        infos = list(msg.cam_infos)
        for i, (name, img) in enumerate(zip(msg.camera_names, msg.rgbs)):
            self._on_camera(name, img, infos[i] if i < len(infos) else None)

    def snapshot(self) -> SceneState:
        with self._lock:
            return SceneState(
                robots={n: {"urdf": r["urdf"], "joints": dict(r["joints"]), "stamp": r["stamp"]}
                        for n, r in self._robots.items()},
                poses=dict(self._poses),
                point_clouds=dict(self._clouds),
                cameras=dict(self._cameras),
            )


def _load_urdf_file(path: str) -> str | None:
    from franka_studio.paths import resolve

    p = resolve(path)
    try:
        if p.suffix == ".xacro":
            import xacro

            return xacro.process_file(str(p)).toxml()
        return p.read_text()
    except Exception:
        return None
