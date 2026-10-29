"""Automatic camera calibration with the ArUco board (studio's Calibration card).

The board lies still on the table; every camera that sees it is calibrated, wrist cameras on
unknown holders included. Per arm (one after the other), starting from a pose where its wrist
camera sees the board:

1. probe: three small tilts about the grasp point; with the views so far, a hand-eye solution
   (``board_calibration.Problem``, which starts from Park & Martin and needs no prior mount)
   gives a first estimate of the camera on the flange and of the board in the world,
2. views: ``views`` camera poses on a cap above the board (same distance as at the start,
   tilted and rolled), looking at its centre, turned into grasp-point targets and driven to,
3. every still pose: one capture of all cameras (static cameras on every one), then back to the
   start pose. The final solve over all captures is the CLI's (scripts/python/board_calibration.py).

Motions are slow Cartesian targets through the safety node's ``target_pose`` service (the IK
follows), limited to ``max_offset`` / ``max_rotation_deg`` around the start pose and kept
``min_clearance`` above the board. The studio's Stop (``/robot_halt``) aborts at once.

Results are stored by name in ``config/calibrations/<name>.yaml`` (absolute values: mount and
base corrections per robot for ``src/franka_launch/config/dfki_fr3_<side>.yaml``, static camera
poses for the project) next to their captures; applying one writes them (restart the robots).
"""

from __future__ import annotations

import datetime
import json
import logging
import re
import sys
import threading
import time
from pathlib import Path

import numpy as np
import yaml
from scipy.spatial.transform import Rotation, Slerp

from franka_studio.paths import WORKSPACE

log = logging.getLogger(__name__)

sys.path.insert(0, str(WORKSPACE / "scripts" / "python"))
import board_calibration as bc  # noqa: E402

CALIB_DIR = WORKSPACE / "config" / "calibrations"
ARM_ID = "fr3"
RATE_HZ = 50.0
PROBES = ((("x", 10.0),), (("y", 10.0),), (("x", -8.0), ("y", -8.0)))   # tilts about the grasp point
VIEW_TILTS_DEG = (18.0, 30.0)
VIEW_ROLL_DEG = 15.0


def _matrix(tf) -> np.ndarray:
    return bc.to_matrix(tf)


def _interp(A: np.ndarray, B: np.ndarray, s: float) -> np.ndarray:
    T = np.eye(4)
    T[:3, 3] = (1 - s) * A[:3, 3] + s * B[:3, 3]
    T[:3, :3] = Slerp([0.0, 1.0], Rotation.from_matrix([A[:3, :3], B[:3, :3]]))(s).as_matrix()
    return T


def valid_name(name: str) -> bool:
    return bool(re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.-]{0,63}", name or ""))


class Aborted(Exception):
    pass


class AutoCalibration:
    def __init__(self, runtime, cfg: dict, save_static, notify) -> None:
        """``save_static(name, position, wxyz)`` stores a static camera pose in the project."""
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
        from sensor_msgs.msg import CameraInfo, CompressedImage
        from std_msgs.msg import Bool
        from tf2_ros import Buffer, TransformListener

        from franka_custom_msgs.srv import SetPoseStamped

        self.cfg = cfg
        self.runtime = runtime
        self.save_static = save_static
        self.notify = notify
        b = cfg["board"]
        self.board = bc.Board(b["dict"], int(b["cols"]), int(b["rows"]), int(b["first_id"]),
                              float(b["marker"]), float(b["sep"]))
        self.arms: dict[str, str] = dict(cfg["arms"])           # wrist camera -> robot namespace
        self.cameras = list(cfg["static"]) + list(self.arms)
        self.prefix = cfg["topic_prefix"].rstrip("/")
        self.node = Node("studio_calibration")
        runtime.add_node(self.node)
        self.tf = Buffer()
        self._tf_listener = TransformListener(self.tf, self.node)
        self._lock = threading.Lock()
        self._images: dict[str, object] = {}
        self._info: dict[tuple, object] = {}
        self._img_subs: list = []
        self._qos_images = qos_profile_sensor_data
        self._msgs = (CompressedImage, CameraInfo)
        for cam in self.cameras:     # camera_info is small: always subscribed
            for side in ("rgb", "right"):
                self.node.create_subscription(
                    CameraInfo, f"{self.prefix}/{cam}/{side}/color/rect/camera_info",
                    lambda m, k=(cam, side): self._store(self._info, k, m), qos_profile_sensor_data)
        self._pose_clients = {robot: self.node.create_client(SetPoseStamped, f"/{robot}/target_pose")
                              for robot in self.arms.values()}
        self._SetPoseStamped = SetPoseStamped
        self.halted = False
        self.node.create_subscription(
            Bool, "/robot_halt", self._on_halt,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))

        self.watching = False
        self.visibility: dict[str, int | None] = {c: None for c in self.cameras}
        self._watch_until = 0.0
        self.state = "idle"        # idle | running | done | failed | stopped
        self.phase = ""
        self.progress: list[str] = []
        self.result: dict | None = None
        self._data: dict | None = None
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        threading.Thread(target=self._watch_loop, daemon=True, name="calib-watch").start()

    # ---- subscriptions -----------------------------------------------------------------------------

    def _store(self, table: dict, key, msg) -> None:
        with self._lock:
            table[key] = msg

    def _on_halt(self, msg) -> None:
        self.halted = bool(msg.data)
        if self.halted:
            self._stop.set()

    def _set_images(self, on: bool) -> None:
        """Compressed images only while needed: the Jetson encodes them only while subscribed."""
        if on and not self._img_subs:
            CompressedImage = self._msgs[0]
            for cam in self.cameras:
                self._img_subs.append(self.node.create_subscription(
                    CompressedImage, f"{self.prefix}/{cam}/rgb/color/rect/image/compressed",
                    lambda m, c=cam: self._store(self._images, c, m), self._qos_images))
        elif not on and self._img_subs:
            for sub in self._img_subs:
                self.node.destroy_subscription(sub)
            self._img_subs = []
            with self._lock:
                self._images.clear()
            self.visibility = {c: None for c in self.cameras}

    def watch(self, on: object) -> dict:
        self.watching = on in (True, "true", "1", 1, "on")
        if not self.watching and self.state != "running":
            self._set_images(False)
        return {"ok": True, "message": "Board detection on." if self.watching else "Board detection off."}

    def _watch_loop(self) -> None:
        import cv2

        while True:
            time.sleep(1.0)
            if not (self.watching or self.state == "running"):
                continue
            self._set_images(True)
            with self._lock:
                images = dict(self._images)
            for cam in self.cameras:
                msg = images.get(cam)
                if msg is None or time.time() - bc.Capture._stamp(msg) > 2.0:
                    self.visibility[cam] = None
                    continue
                gray = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_GRAYSCALE)
                self.visibility[cam] = len(self.board.detect(gray)) if gray is not None else None

    # ---- ROS helpers ------------------------------------------------------------------------------------

    def _lookup(self, target: str, source: str, stamp: float | None = None, timeout: float = 0.5):
        from rclpy.duration import Duration
        from rclpy.time import Time
        t = Time() if stamp is None else Time(seconds=stamp)
        try:
            return _matrix(self.tf.lookup_transform(target, source, t, timeout=Duration(seconds=timeout)))
        except Exception:
            return None

    def _intrinsics(self) -> dict:
        out = {}
        with self._lock:
            for cam in self.cameras:
                info = self._info.get((cam, "rgb"))
                if info is None:
                    continue
                P = np.asarray(info.p, float).reshape(3, 4)
                out[cam] = {"K": P[:3, :3].tolist(), "baseline": None}
        return out

    def _robot_frames(self) -> dict:
        out = {}
        for cam, robot in self.arms.items():
            link8 = f"{robot}_{ARM_ID}_link8"
            optical = self._lookup(link8, f"{robot}_zed_left_camera_optical_frame", timeout=2.0)
            if optical is None:
                raise RuntimeError(f"no TF {link8} <- {robot}_zed_left_camera_optical_frame: are the robots running?")
            mount = self._lookup(link8, f"{robot}_zed_camera_mount")
            base = self._lookup("world", f"{robot}_{ARM_ID}_link0")
            out[cam] = {"robot": robot, "optical": optical.tolist(),
                        "mount": None if mount is None else mount.tolist(),
                        "base": None if base is None else base.tolist()}
        return out

    def _check(self) -> None:
        if self._stop.is_set() or self.halted:
            raise Aborted()

    # ---- capture ------------------------------------------------------------------------------------------

    def _capture(self) -> dict:
        """The next image of every camera with its detection; wrist cameras with the flange pose at
        the image stamp (dropped if the arm moved within 0.3 s before it). ``dropped``: why a camera
        is missing."""
        import cv2

        t0 = time.time()
        fresh: dict = {}
        while time.time() - t0 < 3.0 and len(fresh) < len(self.cameras):
            self._check()
            with self._lock:
                for cam, msg in self._images.items():
                    if cam not in fresh and bc.Capture._stamp(msg) >= t0:
                        fresh[cam] = msg
            time.sleep(0.02)
        sample = {"time": t0, "cams": {}}
        dropped = {cam: "no new image" for cam in self.cameras if cam not in fresh}
        for cam, msg in fresh.items():
            gray = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_GRAYSCALE)
            det = self.board.detect(gray)
            self.visibility[cam] = len(det)
            if len(det) < bc.MIN_MARKERS:
                dropped[cam] = f"{len(det)} markers"
                continue
            entry = {"stamp": bc.Capture._stamp(msg), "left": {str(i): c.tolist() for i, c in det.items()}}
            if cam in self.arms:
                link8 = f"{self.arms[cam]}_{ARM_ID}_link8"
                F = self._lookup("world", link8, entry["stamp"])
                F0 = self._lookup("world", link8, entry["stamp"] - 0.3)
                if F is None or F0 is None:
                    dropped[cam] = f"no TF of {link8} at the image time (clock offset?)"
                    continue
                D = np.linalg.inv(F0) @ F
                moved_mm, moved_deg = 1000 * np.linalg.norm(D[:3, 3]), bc.angle_deg(D[:3, :3])
                if moved_mm > 1000 * bc.STILL_M or moved_deg > bc.STILL_DEG:
                    dropped[cam] = f"arm still moving ({moved_mm:.1f} mm, {moved_deg:.2f} deg in 0.3 s)"
                    continue
                entry["flange"] = F.tolist()
            sample["cams"][cam] = entry
        sample["dropped"] = dropped
        return sample

    def _add_sample(self, label: str) -> dict:
        self._settle()
        sample = self._capture()
        dropped = sample.pop("dropped", {})
        if sample["cams"]:
            self._data["samples"].append(sample)
            self._save_samples()
        seen = ", ".join(f"{c} {len(e['left'])}" for c, e in sample["cams"].items()) or "nothing"
        self._log(f"{label}: board seen by {seen}")
        for cam, why in dropped.items():
            if cam in self.arms:
                self._log(f"  {cam} not used: {why}")
        return sample

    def _settle(self) -> None:
        """Waits settle_s, then until every arm's flange is still (as the capture requires), at most 5 s more."""
        t_end = time.time() + float(self.cfg["settle_s"])
        while time.time() < t_end:
            self._check()
            time.sleep(0.05)
        t_end = time.time() + 5.0
        while time.time() < t_end:
            self._check()
            if all(self._still(robot) for robot in self.arms.values()):
                return
            time.sleep(0.1)

    def _still(self, robot: str) -> bool:
        """No flange motion over the last 0.3 s (TF; True when unknown: the capture reports that)."""
        link8 = f"{robot}_{ARM_ID}_link8"
        now = self.node.get_clock().now().nanoseconds * 1e-9
        F0 = self._lookup("world", link8, now - 0.4)
        F1 = self._lookup("world", link8, now - 0.1)
        if F0 is None or F1 is None:
            return True
        D = np.linalg.inv(F0) @ F1
        return np.linalg.norm(D[:3, 3]) < 0.7 * bc.STILL_M and bc.angle_deg(D[:3, :3]) < 0.7 * bc.STILL_DEG

    # ---- motion ---------------------------------------------------------------------------------------------

    def _grasp(self, robot: str) -> np.ndarray:
        G = self._lookup("world", f"{robot}_grasp_point")
        if G is None:
            raise RuntimeError(f"no TF world <- {robot}_grasp_point")
        return G

    def _send(self, robot: str, T: np.ndarray) -> None:
        from geometry_msgs.msg import PoseStamped
        client = self._pose_clients[robot]
        if not client.service_is_ready():
            raise RuntimeError(f"/{robot}/target_pose is not available")
        req = self._SetPoseStamped.Request()
        req.pose = PoseStamped()
        req.pose.header.stamp = self.node.get_clock().now().to_msg()
        req.pose.header.frame_id = "world"
        x, y, z, w = Rotation.from_matrix(T[:3, :3]).as_quat()
        req.pose.pose.position.x, req.pose.pose.position.y, req.pose.pose.position.z = map(float, T[:3, 3])
        req.pose.pose.orientation.x, req.pose.pose.orientation.y = float(x), float(y)
        req.pose.pose.orientation.z, req.pose.pose.orientation.w = float(z), float(w)
        client.call_async(req)

    def _move(self, robot: str, target: np.ndarray) -> bool:
        """Slow straight-line move of the grasp point (slerped orientation); True when arrived."""
        start = self._grasp(robot)
        dist = float(np.linalg.norm(target[:3, 3] - start[:3, 3]))
        ang = bc.angle_deg(start[:3, :3].T @ target[:3, :3])
        T = max(dist / float(self.cfg["speed"]), ang / float(self.cfg["rotation_speed_deg"]), 1.0)
        n = max(int(T * RATE_HZ), 1)
        for k in range(1, n + 1):
            self._check()
            self._send(robot, _interp(start, target, k / n))
            time.sleep(1.0 / RATE_HZ)
        t_end = time.time() + 4.0     # the IK lags behind its target
        while time.time() < t_end:
            self._check()
            G = self._grasp(robot)
            if (np.linalg.norm(G[:3, 3] - target[:3, 3]) < 0.01
                    and bc.angle_deg(G[:3, :3].T @ target[:3, :3]) < 2.0):
                return True
            self._send(robot, target)
            time.sleep(0.1)
        return False

    # ---- planning ----------------------------------------------------------------------------------------------

    def _estimate(self, cam: str) -> tuple[np.ndarray, np.ndarray]:
        """(link8 <- camera, world <- board) from this arm's captures so far (small probe spread allowed)."""
        robot = self.arms[cam]
        data = dict(self._data, samples=[{**s, "cams": {c: e for c, e in s["cams"].items() if c == cam}}
                                         for s in self._data["samples"]])
        data["samples"] = [s for s in data["samples"] if s["cams"]]
        problem = bc.Problem(data, self.board, reference_robot=robot, min_samples=3, min_spread_deg=6.0)
        if cam not in problem.free_mount:
            raise RuntimeError(f"{cam}: too few probe views with the board in sight ({problem.count.get(cam, 0)})")
        result, rms = problem.solve()
        X = result["x"][f"C:{cam}"] @ problem.frames[cam]["optical"]
        self._log(f"{cam}: first estimate, reprojection {rms[cam]:.2f} px")
        return X, result["x"]["B"]

    def _plan(self, cam: str, X: np.ndarray, B: np.ndarray, G0: np.ndarray, F0: np.ndarray) -> list[np.ndarray]:
        """Grasp-point targets for camera poses looking at the board centre from a cap above it."""
        cfg = self.cfg
        centre = (B @ np.append(np.mean(list(self.board.obj.values()), axis=(0, 1)), 1.0))[:3]
        up = B[:3, 2] if B[2, 2] > 0 else -B[:3, 2]   # board normal, pointing up
        C0 = F0 @ X                                   # camera now
        u0 = C0[:3, 3] - centre
        d = float(np.clip(np.linalg.norm(u0), cfg["min_distance"], cfg["max_distance"]))
        u0 /= np.linalg.norm(u0)
        e1 = np.cross(u0, up if abs(u0 @ up) < 0.95 else np.array([1.0, 0, 0]))
        e1 /= np.linalg.norm(e1)
        e2 = np.cross(u0, e1)
        L8G = np.linalg.inv(F0) @ G0                  # link8 <- grasp point (fixed)
        targets = []
        views = int(cfg["views"])
        for k in range(views):
            phi = 2 * np.pi * k / views
            tilt = np.radians(VIEW_TILTS_DEG[k % 2])
            axis = np.cos(phi) * e1 + np.sin(phi) * e2
            u = Rotation.from_rotvec(tilt * axis).apply(u0)
            if u @ up < np.sin(np.radians(35)):       # stay well above the board
                continue
            p = centre + d * u
            z = centre - p
            z /= np.linalg.norm(z)
            x = C0[:3, 0] - (C0[:3, 0] @ z) * z      # keep the start's image orientation, rolled
            x /= np.linalg.norm(x)
            Rc = np.stack([x, np.cross(z, x), z], axis=1) @ Rotation.from_euler(
                "z", VIEW_ROLL_DEG * (1 if k % 2 else -1), degrees=True).as_matrix()
            Tc = np.eye(4)
            Tc[:3, :3], Tc[:3, 3] = Rc, p
            G = Tc @ np.linalg.inv(X) @ L8G
            off = np.linalg.norm(G[:3, 3] - G0[:3, 3])
            rot = bc.angle_deg(G0[:3, :3].T @ G[:3, :3])
            clearance = (G[:3, 3] - centre) @ up
            if off > cfg["max_offset"] or rot > cfg["max_rotation_deg"] or clearance < cfg["min_clearance"]:
                self._log(f"  view {k + 1} skipped (offset {100 * off:.0f} cm, rotation {rot:.0f} deg, "
                          f"{100 * clearance:.0f} cm above the board)")
                continue
            targets.append(G)
        # nearest neighbour order from the start
        ordered, here = [], G0
        while targets:
            i = int(np.argmin([np.linalg.norm(t[:3, 3] - here[:3, 3]) for t in targets]))
            here = targets.pop(i)
            ordered.append(here)
        return ordered

    # ---- run ------------------------------------------------------------------------------------------------------

    def _log(self, text: str) -> None:
        log.info(text)
        self.progress.append(text)
        del self.progress[:-60]

    def start(self, arms: list | None) -> dict:
        if self.state == "running":
            return {"ok": False, "message": "A calibration is running."}
        if self.halted:
            return {"ok": False, "message": "Halted: press Resume first."}
        cams = [c for c in (arms or list(self.arms)) if c in self.arms]
        if not cams:
            return {"ok": False, "message": "Choose at least one wrist camera."}
        self._stop.clear()
        self.state, self.phase, self.progress, self.result = "running", "starting", [], None
        self._thread = threading.Thread(target=self._run, args=(cams,), daemon=True, name="calibration")
        self._thread.start()
        return {"ok": True, "message": f"Calibrating {', '.join(cams)}."}

    def stop(self, reason: str = "stopped") -> bool:
        if self.state != "running":
            return False
        self._stop.set()
        self._log(f"Stop requested ({reason}).")
        return True

    def _run(self, cams: list[str]) -> None:
        starts: dict[str, np.ndarray] = {}
        try:
            self._set_images(True)
            time.sleep(1.0)
            self._data = {"board": self.board.to_dict(), "intrinsics": self._intrinsics(),
                          "robot_frames": self._robot_frames(), "samples": []}
            self._samples_path = CALIB_DIR / time.strftime("captures_%Y%m%d_%H%M%S.json")
            for cam in cams:
                robot = self.arms[cam]
                self.phase = f"{cam}: start"
                G0 = self._grasp(robot)
                F0 = self._lookup("world", f"{robot}_{ARM_ID}_link8")
                starts[robot] = G0
                first = self._add_sample(f"{cam} start")
                if cam not in first["cams"]:
                    raise RuntimeError(f"{cam} does not see the board: move {robot} so that it does and start again")
                for i, probe in enumerate(PROBES, 1):
                    self.phase = f"{cam}: probe {i}/{len(PROBES)}"
                    R = Rotation.identity()
                    for axis, deg in probe:
                        R = R * Rotation.from_euler(axis, deg, degrees=True)
                    T = G0.copy()
                    T[:3, :3] = G0[:3, :3] @ R.as_matrix()
                    if not self._move(robot, T):
                        self._log(f"  probe {i}: target not reached, capturing anyway")
                    self._add_sample(f"{cam} probe {i}")
                X, B = self._estimate(cam)
                targets = self._plan(cam, X, B, G0, F0)
                self._log(f"{cam}: {len(targets)} views planned")
                for i, T in enumerate(targets, 1):
                    self.phase = f"{cam}: view {i}/{len(targets)}"
                    if not self._move(robot, T):
                        self._log(f"  view {i}: target not reached, capturing anyway")
                    self._add_sample(f"{cam} view {i}")
                self.phase = f"{cam}: back to the start"
                self._move(robot, G0)
                starts.pop(robot, None)
            self.phase = "solving"
            self.result = self._solve()
            self.state, self.phase = "done", "done"
            self._log("Done: check the result and save it under a name.")
        except Aborted:
            self.state, self.phase = "stopped", "stopped"
            self._log("Stopped. The arms hold where they are.")
        except Exception as e:
            log.exception("calibration failed")
            self.state, self.phase = "failed", "failed"
            self._log(f"Failed: {e}")
            for robot, G0 in starts.items():    # bring a probing arm back where it started
                try:
                    if not self.halted:
                        self._move(robot, G0)
                except Exception:
                    pass
        finally:
            if not self.watching:
                self._set_images(False)
            self.notify(f"Calibration {self.state}.", "success" if self.state == "done" else "warn")

    def _save_samples(self) -> None:
        CALIB_DIR.mkdir(parents=True, exist_ok=True)
        self._samples_path.write_text(json.dumps(self._data))

    def _solve(self) -> dict:
        wrist_robots = {self.arms[c] for s in self._data["samples"] for c in s["cams"] if c in self.arms}
        reference = bc.REFERENCE_ROBOT if bc.REFERENCE_ROBOT in wrist_robots else sorted(wrist_robots)[0]
        problem = bc.Problem(self._data, self.board, reference_robot=reference)
        rms0 = problem.rms(problem.x0)
        result, rms = problem.solve()
        values = bc.calibration_values(problem, result)
        std = {k: {"deg": round(v[0], 3), "mm": round(v[1], 2)} for k, v in result["std"].items()}
        for c, r in rms.items():
            self._log(f"  {c}: reprojection {r:.2f} px ({problem.count.get(c, sum(o['cam'] == c for o in problem.obs))} views)")
        return {"values": values, "rms": {c: round(v, 3) for c, v in rms.items()},
                "rms_before": {c: round(v, 3) for c, v in rms0.items()}, "std": std, "reference": reference,
                "solved_mounts": problem.free_mount, "solved_bases": [problem.frames[c]["robot"] for c in problem.free_base],
                "captures": str(self._samples_path.relative_to(WORKSPACE)), "board": self.board.to_dict(),
                "mount_config": self._mount_config()}

    @staticmethod
    def _mount_config() -> dict:
        """The end-effector settings the corrections build on (warned about when they differ on apply)."""
        out = {}
        for side in ("right", "left"):
            path = Path(str(bc.LAUNCH_CONFIG).format(side=side))
            try:
                rc = (yaml.safe_load(path.read_text()) or {}).get("robot_config", {})
            except OSError:
                continue
            out[f"franka_{side}"] = {k: str(rc[k]) for k in ("rpy_ee", "camera_holder", "cam_holder_yaw", "cam_image_flip") if k in rc}
        return out

    # ---- saved calibrations -----------------------------------------------------------------------------------------

    def save(self, name: str) -> dict:
        if not self.result:
            return {"ok": False, "message": "No calibration result to save."}
        if not valid_name(name):
            return {"ok": False, "message": "Name: letters, digits, '.', '_' or '-'."}
        CALIB_DIR.mkdir(parents=True, exist_ok=True)
        path = CALIB_DIR / f"{name}.yaml"
        doc = {"name": name, "created": datetime.datetime.now().isoformat(timespec="seconds"), **self.result}
        path.write_text("# Camera calibration (studio: Calibration card); apply it there or with\n"
                        "# board_calibration.write_robot_values + the project's camera_poses.yaml.\n"
                        + yaml.safe_dump(doc, sort_keys=False, default_flow_style=None))
        return {"ok": True, "message": f"Saved calibration '{name}'."}

    def saved(self) -> list[dict]:
        out = []
        for path in sorted(CALIB_DIR.glob("*.yaml")):
            try:
                doc = yaml.safe_load(path.read_text()) or {}
            except (OSError, yaml.YAMLError):
                continue
            out.append({"name": path.stem, "created": doc.get("created"), "rms": doc.get("rms", {}),
                        "robots": sorted((doc.get("values") or {}).get("robots", {})),
                        "static": sorted((doc.get("values") or {}).get("static", {}))})
        return out

    def apply(self, name: str) -> dict:
        path = CALIB_DIR / f"{name}.yaml"
        if not valid_name(name) or not path.is_file():
            return {"ok": False, "message": f"No calibration '{name}'."}
        doc = yaml.safe_load(path.read_text()) or {}
        values = doc.get("values") or {}
        warnings = []
        now = self._mount_config()
        for robot, then in (doc.get("mount_config") or {}).items():
            if now.get(robot) and now[robot] != then:
                warnings.append(f"{robot}: end-effector settings changed since ({then} -> {now[robot]})")
        written = [p.name for p in bc.write_robot_values(values.get("robots"))]
        for cam, pose in (values.get("static") or {}).items():
            self.save_static(cam, pose["position"], pose["wxyz"])
        msg = f"Applied '{name}'"
        if written:
            msg += f" ({', '.join(written)}): restart the robots to load it"
        if warnings:
            msg += ". Warning: " + "; ".join(warnings)
        return {"ok": True, "message": msg + ".", "restart_robots": bool(written)}

    # ---- status ---------------------------------------------------------------------------------------------------------

    def snapshot(self) -> dict:
        return {"state": self.state, "phase": self.phase, "progress": self.progress[-14:],
                "watching": self.watching, "visibility": dict(self.visibility),
                "arms": self.arms, "static": list(self.cfg["static"]), "halted": self.halted,
                "result": ({k: self.result[k] for k in ("rms", "rms_before", "solved_mounts", "solved_bases", "reference")}
                           | {"values": self.result["values"]}) if self.result else None,
                "saved": self.saved()}
