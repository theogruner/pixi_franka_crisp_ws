"""Automatic camera calibration with the ArUco board (studio's Calibration card).

The board lies still on the table; every camera that sees it is calibrated, wrist cameras on
unknown holders included. Per arm (one after the other), starting from a pose where its wrist
camera sees the board:

1. probe: three small tilts about the grasp point; with the views so far, a hand-eye solution
   (``board_calibration.Problem``, which starts from Park & Martin and needs no prior mount)
   gives a first estimate of the camera on the flange and of the board in the world,
2. views: ``views`` camera poses on a cap above the board (same distance as at the start,
   tilted and rolled), looking at its centre, turned into grasp-point targets and driven to,
3. every still pose: one capture of all cameras (static cameras on every one) with their point
   clouds (``/ffs/<camera>/points``, if the point cloud node runs), then back to the start pose.

The final solve over all captures is the CLI's (scripts/python/board_calibration.py), fine-tuned
with the clouds (cloud_refine.py). **Add placement** runs again with the board moved elsewhere and
solves every placement together: the more of the workspace the board covered, the less a small
rotation error grows away from it. A weak run (too few / too similar wrist views, a badly pinned
down pose: ``Problem.weak``) cannot be saved.

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
import os
import re
import subprocess
import sys
import threading
import time
from pathlib import Path

import numpy as np
import yaml
from scipy.spatial.transform import Rotation

from franka_studio.motion import ARM_ID, Aborted, ArmMover
from franka_studio.paths import WORKSPACE

log = logging.getLogger(__name__)

sys.path.insert(0, str(WORKSPACE / "scripts" / "python"))
import board_calibration as bc  # noqa: E402

CALIB_DIR = WORKSPACE / "config" / "calibrations"
PROBES = ((("x", 10.0),), (("y", 10.0),), (("x", -8.0), ("y", -8.0)))   # tilts about the grasp point
VIEW_TILTS_DEG = (18.0, 30.0)
VIEW_ROLL_DEG = 15.0


def valid_name(name: str) -> bool:
    return bool(re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.-]{0,63}", name or ""))


class AutoCalibration(ArmMover):
    def __init__(self, runtime, cfg: dict, save_static, notify) -> None:
        """``save_static(name, position, wxyz)`` stores a static camera pose in the project."""
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import CameraInfo, CompressedImage, PointCloud2

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
        self._init_motion(runtime, "studio_calibration", list(self.arms.values()), cfg)
        self._lock = threading.Lock()
        self._images: dict[str, object] = {}
        self._info: dict[tuple, object] = {}
        self._img_subs: list = []
        self._qos_images = qos_profile_sensor_data
        self._msgs = (CompressedImage, CameraInfo, PointCloud2)
        self._cloud_msgs: dict[str, object] = {}
        for cam in self.cameras:     # camera_info is small: always subscribed
            for side in ("rgb", "right"):
                self.node.create_subscription(
                    CameraInfo, f"{self.prefix}/{cam}/{side}/color/rect/camera_info",
                    lambda m, k=(cam, side): self._store(self._info, k, m), qos_profile_sensor_data)

        self.watching = False
        self.visibility: dict[str, int | None] = {c: None for c in self.cameras}
        self._watch_until = 0.0
        self.state = "idle"        # idle | running | done | failed | stopped
        self.phase = ""
        self.progress: list[str] = []
        self.result: dict | None = None
        self._data: dict | None = None
        self._clouds: dict = {}                  # (sample index, camera) -> camera-frame points
        self._placement = 0
        self._wanted: set[str] = set()           # wrist cameras calibrated over all placements
        self._rebase: dict[str, np.ndarray] = {}  # flange (current TF) -> the first placement's robot bases
        self._thread: threading.Thread | None = None
        threading.Thread(target=self._watch_loop, daemon=True, name="calib-watch").start()

    # ---- subscriptions -----------------------------------------------------------------------------

    def _store(self, table: dict, key, msg) -> None:
        with self._lock:
            table[key] = msg

    def _set_images(self, on: bool) -> None:
        """Compressed images only while needed: the Jetson encodes them only while subscribed."""
        if on and not self._img_subs:
            CompressedImage, _, PointCloud2 = self._msgs
            for cam in self.cameras:
                self._img_subs.append(self.node.create_subscription(
                    CompressedImage, f"{self.prefix}/{cam}/rgb/color/rect/image/compressed",
                    lambda m, c=cam: self._store(self._images, c, m), self._qos_images))
                if self.cfg.get("cloud_topic"):
                    self._img_subs.append(self.node.create_subscription(
                        PointCloud2, self.cfg["cloud_topic"].format(camera=cam),
                        lambda m, c=cam: self._store(self._cloud_msgs, c, m), self._qos_images))
        elif not on and self._img_subs:
            from franka_studio.ros.runtime import retire_subscription

            for sub in self._img_subs:
                retire_subscription(self.node, sub)
            self._img_subs = []
            with self._lock:
                self._images.clear()
                self._cloud_msgs.clear()
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
                entry["flange"] = (self._rebase.get(cam, np.eye(4)) @ F).tolist()
            sample["cams"][cam] = entry
        sample["dropped"] = dropped
        sample["clouds"] = self._grab_clouds(list(sample["cams"]), t0)
        return sample

    def _grab_clouds(self, cams: list[str], t0: float, timeout: float = 2.0) -> dict:
        """The first point cloud of each camera from an image after ``t0`` (camera frame, thinned);
        none without the point cloud node."""
        with self._lock:
            if not self._cloud_msgs:
                return {}
        out: dict = {}
        deadline = time.time() + timeout
        while time.time() < deadline and len(out) < len(cams):
            self._check()
            with self._lock:
                msgs = {c: self._cloud_msgs.get(c) for c in cams if c not in out}
            for cam, msg in msgs.items():
                if msg is not None and bc.Capture._stamp(msg) >= t0:
                    out[cam] = bc.cloud_refine.thin(cloud_xyz(msg))
            time.sleep(0.02)
        return out

    def _add_sample(self, label: str) -> dict:
        self._settle()
        sample = self._capture()
        dropped = sample.pop("dropped", {})
        clouds = sample.pop("clouds", {})
        sample["placement"] = self._placement
        if sample["cams"]:
            k = len(self._data["samples"])
            self._data["samples"].append(sample)
            self._clouds.update({(k, cam): xyz for cam, xyz in clouds.items() if cam in sample["cams"]})
            self._save_samples()
        seen = ", ".join(f"{c} {len(e['left'])}" for c, e in sample["cams"].items()) or "nothing"
        self._log(f"{label}: board seen by {seen}" + (f"; {len(clouds)} clouds" if clouds else ""))
        for cam, why in dropped.items():
            if cam in self.arms:
                self._log(f"  {cam} not used: {why}")
        return sample

    # ---- planning ----------------------------------------------------------------------------------------------

    def _estimate(self, cam: str) -> tuple[np.ndarray, np.ndarray]:
        """(link8 <- camera, world <- board) from this arm's captures of this placement so far (small
        probe spread allowed); the board in the current TF's world, as the motions are."""
        robot = self.arms[cam]
        data = dict(self._data, samples=[{**s, "cams": {c: e for c, e in s["cams"].items() if c == cam}}
                                         for s in self._data["samples"] if s.get("placement", 0) == self._placement])
        data["samples"] = [s for s in data["samples"] if s["cams"]]
        problem = bc.Problem(data, self.board, reference_robot=robot, min_samples=3, min_spread_deg=6.0)
        if cam not in problem.free_mount:
            raise RuntimeError(f"{cam}: too few probe views with the board in sight ({problem.count.get(cam, 0)})")
        result, rms = problem.solve()
        X = result["x"][f"C:{cam}"] @ problem.frames[cam]["optical"]
        self._log(f"{cam}: first estimate, reprojection {rms[cam]:.2f} px")
        return X, np.linalg.inv(self._rebase.get(cam, np.eye(4))) @ result["x"][f"B:{self._placement}"]

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

    def start(self, arms: list | None, add: bool = False) -> dict:
        """``add``: the board was moved; capture it at its new place and solve with the earlier placements."""
        if self.state == "running":
            return {"ok": False, "message": "A calibration is running."}
        if add and not (self._data and self._data["samples"]):
            return {"ok": False, "message": "Nothing to add to: calibrate first."}
        if self.halted:
            return {"ok": False, "message": "Halted: press Resume first."}
        cams = [c for c in (arms or list(self.arms)) if c in self.arms]
        if not cams:
            return {"ok": False, "message": "Choose at least one wrist camera."}
        self._stop.clear()
        self.state, self.phase, self.progress, self.result = "running", "starting", [], None
        self._thread = threading.Thread(target=self._run, args=(cams, add), daemon=True, name="calibration")
        self._thread.start()
        if add:
            return {"ok": True, "message": f"Board placement {self._placement + 2}: calibrating {', '.join(cams)}."}
        return {"ok": True, "message": f"Calibrating {', '.join(cams)}."}

    def stop(self, reason: str = "stopped") -> bool:
        if self.state != "running":
            return False
        self._stop.set()
        self._log(f"Stop requested ({reason}).")
        return True

    def _new_placement(self, add: bool) -> None:
        frames = self._robot_frames()
        if add:
            # a calibration applied in between moved the bases in TF: keep the first placement's
            self._rebase = bc.rebase(self._data["robot_frames"], frames)
            self._placement = max(s.get("placement", 0) for s in self._data["samples"]) + 1
            intrinsics = self._intrinsics()
            if intrinsics != self._data["intrinsics"]:
                for s in self._data["samples"]:
                    s.setdefault("intrinsics", self._data["intrinsics"])
                self._data["intrinsics"] = intrinsics
        else:
            self._data = {"board": self.board.to_dict(), "intrinsics": self._intrinsics(),
                          "robot_frames": frames, "samples": []}
            self._samples_path = CALIB_DIR / time.strftime("captures_%Y%m%d_%H%M%S.json")
            self._clouds, self._placement, self._wanted, self._rebase = {}, 0, set(), {}

    def _run(self, cams: list[str], add: bool = False) -> None:
        starts: dict[str, np.ndarray] = {}
        try:
            self._set_images(True)
            time.sleep(1.0)
            self._new_placement(add)
            self._wanted |= set(cams)
            if self._placement:
                self._log(f"Board placement {self._placement + 1}")
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
            self._log("Done: weak, add a placement." if self.result.get("weak") else "Done: check the result and save it under a name.")
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
        if self._clouds:
            bc.save_clouds(bc.clouds_path(self._samples_path), self._clouds)

    def _solve(self) -> dict:
        """The CLI's solve of the saved captures, in a child process with one BLAS thread: next to the
        studio's few hundred threads the same solve takes minutes instead of seconds. Stop kills it."""
        wrist_robots = {self.arms[c] for s in self._data["samples"] for c in s["cams"] if c in self.arms}
        reference = bc.REFERENCE_ROBOT if bc.REFERENCE_ROBOT in wrist_robots else sorted(wrist_robots)[0]
        out = self._samples_path.with_suffix(".result.json")
        b = self.board
        cmd = [sys.executable, bc.__file__, "--samples", str(self._samples_path), "--json", str(out),
               "--reference", reference, "--wrists", ",".join(sorted(self._wanted)),
               "--dict", b.dict_name, "--cols", str(b.cols), "--rows", str(b.rows), "--first-id", str(b.first_id)]
        env = {**os.environ, "OPENBLAS_NUM_THREADS": "1", "OMP_NUM_THREADS": "1", "MKL_NUM_THREADS": "1"}
        out.unlink(missing_ok=True)
        proc = subprocess.Popen(cmd, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, text=True)
        try:
            while proc.poll() is None:
                self._check()
                time.sleep(0.2)
        finally:
            if proc.poll() is None:
                proc.kill()
                proc.wait()
        if proc.returncode != 0 or not out.is_file():
            err = (proc.stderr.read() or "").strip().splitlines()
            raise RuntimeError(f"solve failed: {err[-1] if err else f'exit code {proc.returncode}'}")
        s = json.loads(out.read_text())
        out.unlink()
        for c, r in s["rms"].items():
            self._log(f"  {c}: reprojection {r:.2f} px ({s['views'].get(c, 0)} views)")
        cloud = s["cloud"]
        if cloud.get("before"):
            for c, a in cloud["after"].items():
                self._log(f"  {c}: {cloud['before'][c]['mm']} -> {a['mm']} mm from the static clouds")
        self._log(f"  point clouds: {'applied' if cloud['used'] else cloud['message']}")
        for reason in s["weak"]:
            self._log(f"  weak: {reason}")
        if s["weak"]:
            self._log("Weak run: not to be saved. Add a placement or calibrate again.")
        return {**{k: s[k] for k in ("values", "weak", "cloud", "placements", "rms", "rms_before", "std", "reference",
                                     "solved_mounts", "solved_bases")},
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
        if self.result.get("weak"):
            return {"ok": False, "message": "Weak calibration, not saved: " + "; ".join(self.result["weak"])}
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
                "result": ({k: self.result.get(k) for k in ("rms", "rms_before", "solved_mounts", "solved_bases",
                                                             "reference", "weak", "placements")}
                           | {"values": self.result["values"], "cloud": _cloud_summary(self.result.get("cloud"))})
                if self.result else None,
                "can_add": bool(self._data and self._data["samples"]) and self.state != "running",
                "saved": self.saved()}


def cloud_xyz(msg) -> np.ndarray:
    """x, y, z of a PointCloud2 whose first fields are float32 x/y/z (the point cloud node's)."""
    data = np.frombuffer(bytes(msg.data), np.uint8).reshape(-1, msg.point_step)
    return np.ascontiguousarray(data[:, :12]).view(np.float32).reshape(-1, 3)


def _cloud_summary(cloud: dict | None) -> dict | None:
    if not cloud:
        return None
    if not cloud.get("used"):
        return {"used": False, "message": cloud.get("message", "")}
    return {"used": True, "scale": cloud["scale"],
            "mm": {c: [cloud["before"][c]["mm"], a["mm"]] for c, a in cloud["after"].items()}}
