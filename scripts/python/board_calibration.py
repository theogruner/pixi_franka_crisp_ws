"""Extrinsic calibration of the ZED rig from an ArUco grid board lying on the table.

    pixi run calibrate-board                    # capture (Enter per pose) + solve
    pixi run calibrate-board --once             # one capture, solve
    pixi run calibrate-board --samples calibration/board_X.json     # solve saved captures again
    ... [--marker 0.0562 --sep 0.0067] [--write [--project example]]

The board's pose is not needed: it is one more unknown. Every capture takes the next image of
every camera (rectified left, so no distortion; ZED factory intrinsics from camera_info), detects
the markers and, for wrist cameras, the flange pose (TF ``world <- <robot>_fr3_link8`` at the
image stamp; the arm has to hold still). One bundle adjustment over all captures then minimises
the reprojection error of every detected marker corner over

* ``B``  the board in the world,
* ``M``  every static camera in the world (zed_middle),
* ``C``  per wrist camera, a correction of its mount on the flange (link8 <- link8'), and
* ``D``  per robot other than the reference (``franka_right``, whose base defines the world),
         a correction of its base in the world,

with each wrist camera at ``D * world<-link8(TF) * C * link8<-optical(TF)``.

What a session can determine:

* A single capture (arms anywhere, every camera seeing the board): ``B`` and ``M`` only, with the
  robot description's wrist poses taken as they are. It also prints how far apart the two wrist
  cameras put the board -- that is the misalignment of their point clouds.
* Several captures with the wrist turned about different axes (>= 3, >= 15 deg apart, the board
  in view; hand guiding or teleop, hold still at each): ``C`` of that wrist, and for the left arm
  its base correction ``D`` as well. A wrist without enough variety keeps its description pose.

Board: ``--dict``/``--cols``/``--rows``/``--first-id`` describe the grid (default: DICT_4X4_50,
3 x 4 markers, ids 0..11 row-major, as printed by cv2.aruco.GridBoard). Measure the printed
marker side and the gap between markers with a ruler and pass ``--marker``/``--sep`` (m): every
translation scales with them. Without them the ratio is fitted to the images and the scale is
triangulated by the ZED stereo pairs (~1 %), and printed so it can be checked.

``--write`` stores the result: static cameras through the running studio (project
``camera_poses.yaml``; without the studio directly into ``projects/<--project>``), wrist mount
corrections as ``cam_pos_x .. cam_yaw`` and base corrections as ``xyz``/``rpy`` in
``src/franka_launch/config/dfki_fr3_<side>.yaml`` (restart the robots to load them). Corrections
build on what is already there: TF includes the current ones, so calibrating twice refines.
"""

from __future__ import annotations

import argparse
import itertools
import json
import re
import sys
import threading
import time
import urllib.request
from pathlib import Path

import cv2
import numpy as np
import yaml
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

WS = Path(__file__).resolve().parents[2]
PREFIX = "/zed_multi"
ARM_ID = "fr3"
WRISTS = {"zed_right_arm": "franka_right", "zed_left_arm": "franka_left"}
STATIC = ["zed_middle"]
REFERENCE_ROBOT = "franka_right"
LAUNCH_CONFIG = WS / "src/franka_launch/config/dfki_fr3_{side}.yaml"
STILL_M, STILL_DEG = 0.001, 0.2       # flange motion within 0.3 s before the image that counts as still
MIN_SAMPLES, MIN_SPREAD_DEG = 3, 15.0  # wrist rotation variety needed to solve its mount
MIN_MARKERS = 2
# The printed board's white border is narrower than OpenCV likes; it is only used as a model here.
cv2.utils.logging.setLogLevel(cv2.utils.logging.LOG_LEVEL_ERROR)


# ---- geometry ------------------------------------------------------------------------------------

def to_matrix(tf) -> np.ndarray:
    t, q = tf.transform.translation, tf.transform.rotation
    M = np.eye(4)
    M[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    M[:3, 3] = [t.x, t.y, t.z]
    return M


def from_vec(v: np.ndarray) -> np.ndarray:
    M = np.eye(4)
    M[:3, :3] = Rotation.from_rotvec(v[:3]).as_matrix()
    M[:3, 3] = v[3:]
    return M


def to_vec(M: np.ndarray) -> np.ndarray:
    return np.concatenate([Rotation.from_matrix(M[:3, :3]).as_rotvec(), M[:3, 3]])


def angle_deg(Rm: np.ndarray) -> float:
    return float(np.degrees(np.arccos(np.clip((np.trace(Rm) - 1) / 2, -1, 1))))


def pose_dict(M: np.ndarray) -> dict:
    x, y, z, w = Rotation.from_matrix(M[:3, :3]).as_quat()
    return {"position": [round(float(v), 5) for v in M[:3, 3]], "wxyz": [round(float(v), 6) for v in (w, x, y, z)]}


def xyz_rpy(M: np.ndarray) -> tuple[list, list]:
    """URDF origin: rpy are fixed-axis roll, pitch, yaw (R = Rz Ry Rx)."""
    return [float(v) for v in M[:3, 3]], [float(v) for v in Rotation.from_matrix(M[:3, :3]).as_euler("xyz")]


# ---- board ---------------------------------------------------------------------------------------

class Board:
    def __init__(self, dict_name: str, cols: int, rows: int, first_id: int, marker: float = 1.0, sep: float = 0.2):
        self.dict_name, self.cols, self.rows, self.first_id = dict_name, cols, rows, first_id
        self.dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dict_name))
        params = cv2.aruco.DetectorParameters()
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.detector = cv2.aruco.ArucoDetector(self.dictionary, params)
        self.set_size(marker, sep)

    def set_size(self, marker: float, sep: float) -> None:
        self.marker, self.sep = marker, sep
        ids = np.arange(self.first_id, self.first_id + self.cols * self.rows)
        grid = cv2.aruco.GridBoard((self.cols, self.rows), marker, sep, self.dictionary, ids)
        self.obj = {int(i): np.asarray(p, float).reshape(4, 3) for i, p in zip(grid.getIds().ravel(), grid.getObjPoints())}

    def detect(self, gray: np.ndarray) -> dict[int, np.ndarray]:
        corners, ids, _ = self.detector.detectMarkers(gray)
        if ids is None:
            return {}
        return {int(i): c.reshape(4, 2).astype(float) for i, c in zip(ids.ravel(), corners) if int(i) in self.obj}

    def points(self, det: dict) -> tuple[np.ndarray, np.ndarray]:
        ids = sorted(det)
        return np.concatenate([self.obj[i] for i in ids]), np.concatenate([np.asarray(det[i], float) for i in ids])

    def to_dict(self) -> dict:
        return {"dict": self.dict_name, "cols": self.cols, "rows": self.rows, "first_id": self.first_id,
                "marker": self.marker, "sep": self.sep}


def solve_pnp(board: Board, det: dict, K: np.ndarray) -> tuple[np.ndarray, float]:
    """camera <- board from one detection, and its RMS reprojection error (px)."""
    obj, img = board.points(det)
    ok, rvec, tvec = cv2.solvePnP(obj, img, K, None, flags=cv2.SOLVEPNP_SQPNP)
    rvec, tvec = cv2.solvePnPRefineLM(obj, img, K, None, rvec, tvec)
    proj, _ = cv2.projectPoints(obj, rvec, tvec, K, None)
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = cv2.Rodrigues(rvec)[0], tvec.ravel()
    return T, float(np.sqrt(np.mean(np.sum((proj.reshape(-1, 2) - img) ** 2, axis=1))))


def estimate_size(board: Board, views: list[dict]) -> tuple[float, float, float]:
    """Marker side and gap (m) from the captures: the gap/side ratio from the image shapes, the
    scale from the stereo pairs. views: {K, baseline, left, right}. Also returns the scale's spread."""
    def ratio_error(r: float) -> float:
        board.set_size(1.0, r)
        return sum(solve_pnp(board, v["left"], v["K"])[1] ** 2 for v in views)

    rs = np.linspace(0.02, 1.0, 99)
    r = rs[int(np.argmin([ratio_error(x) for x in rs]))]
    fine = np.linspace(max(r - 0.01, 0.005), r + 0.01, 41)
    r = float(fine[int(np.argmin([ratio_error(x) for x in fine]))])
    board.set_size(1.0, r)
    scales, weights = [], []
    for v in views:
        ids = sorted(set(v["left"]) & set(v["right"]))
        if len(ids) < MIN_MARKERS:
            continue
        K, B = v["K"], v["baseline"]
        left = np.concatenate([v["left"][i] for i in ids])
        right = np.concatenate([v["right"][i] for i in ids])
        z = K[0, 0] * B / np.maximum(left[:, 0] - right[:, 0], 1e-3)
        X = np.stack([(left[:, 0] - K[0, 2]) * z / K[0, 0], (left[:, 1] - K[1, 2]) * z / K[1, 1], z], 1)
        model = np.concatenate([board.obj[i] for i in ids])
        a, b = np.triu_indices(len(X), 1)
        dt, dm = np.linalg.norm(X[a] - X[b], axis=1), np.linalg.norm(model[a] - model[b], axis=1)
        scales.append(float(dt @ dm / (dm @ dm)))
        weights.append(float(dm @ dm))
    if not scales:
        raise RuntimeError("no camera saw the board in both images; pass --marker and --sep")
    s = float(np.average(scales, weights=weights))
    spread = float(np.std(scales) / s) if len(scales) > 1 else float("nan")
    return s, s * r, spread


# ---- capture -------------------------------------------------------------------------------------

class Capture:
    """Subscribes to the rig's images (compressed) and TF; ``grab()`` returns one sample."""

    def __init__(self, cameras: list[str], wrists: dict[str, str], stereo: bool) -> None:
        import rclpy
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import CameraInfo, CompressedImage
        from tf2_ros import Buffer, TransformListener

        rclpy.init()
        self.rclpy = rclpy
        self.node = rclpy.create_node("board_calibration")
        self.cameras, self.wrists = cameras, wrists
        self.tf = Buffer()
        TransformListener(self.tf, self.node)
        self._lock = threading.Lock()
        self.images: dict[tuple, object] = {}
        self.info: dict[tuple, object] = {}
        sides = ("rgb", "right") if stereo else ("rgb",)
        for cam, side in itertools.product(cameras, sides):
            base = f"{PREFIX}/{cam}/{side}/color/rect"
            self.node.create_subscription(CompressedImage, base + "/image/compressed",
                                          lambda m, k=(cam, side): self._store(self.images, k, m), qos_profile_sensor_data)
            self.node.create_subscription(CameraInfo, base + "/camera_info",
                                          lambda m, k=(cam, side): self._store(self.info, k, m), qos_profile_sensor_data)
        threading.Thread(target=rclpy.spin, args=(self.node,), daemon=True).start()

    def _store(self, table: dict, key: tuple, msg: object) -> None:
        with self._lock:
            table[key] = msg

    @staticmethod
    def _stamp(msg) -> float:
        return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def _lookup(self, target: str, source: str, stamp: float | None = None) -> np.ndarray | None:
        from rclpy.duration import Duration
        from rclpy.time import Time
        t = Time() if stamp is None else Time(seconds=stamp)
        try:
            return to_matrix(self.tf.lookup_transform(target, source, t, timeout=Duration(seconds=0.5)))
        except Exception:
            return None

    def intrinsics(self, timeout: float = 10.0) -> dict:
        """camera -> {K, baseline} from camera_info (rectified: P)."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            with self._lock:
                ready = all((c, "rgb") in self.info for c in self.cameras)
            if ready:
                break
            time.sleep(0.1)
        out = {}
        with self._lock:
            for cam in self.cameras:
                if (cam, "rgb") not in self.info:
                    raise RuntimeError(f"no camera_info from {cam}: are the cameras running?")
                P = np.asarray(self.info[(cam, "rgb")].p, float).reshape(3, 4)
                entry = {"K": P[:3, :3].tolist(), "baseline": None}
                if (cam, "right") in self.info:
                    Pr = np.asarray(self.info[(cam, "right")].p, float).reshape(3, 4)
                    entry["baseline"] = float(-Pr[0, 3] / Pr[0, 0]) if Pr[0, 0] else None
                out[cam] = entry
        return out

    def robot_frames(self) -> dict:
        """Per wrist camera: link8 <- optical, link8 <- mount (current correction), world <- link0."""
        out = {}
        for cam, robot in self.wrists.items():
            link8 = f"{robot}_{ARM_ID}_link8"
            optical = None
            deadline = time.time() + 15.0   # /tf_static of a big description takes a few s to arrive
            while optical is None and time.time() < deadline:
                optical = self._lookup(link8, f"{robot}_zed_left_camera_optical_frame")
            if optical is None:
                raise RuntimeError(f"no TF {link8} <- {robot}_zed_left_camera_optical_frame: is {robot} running?")
            # The stillness check looks 0.3 s back: wait until /tf has that much history
            while self._lookup("world", link8, time.time() - 0.5) is None and time.time() < deadline:
                time.sleep(0.1)
            mount = self._lookup(link8, f"{robot}_zed_camera_mount")
            base = self._lookup("world", f"{robot}_{ARM_ID}_link0")
            out[cam] = {"robot": robot, "optical": optical.tolist(),
                        "mount": None if mount is None else mount.tolist(),
                        "base": None if base is None else base.tolist()}
        return out

    def grab(self, board: Board, timeout: float = 3.0) -> dict:
        t0 = time.time()
        sides = sorted({k[1] for k in self.images} | {"rgb"})
        msgs: dict[tuple, object] = {}
        while time.time() - t0 < timeout:
            with self._lock:
                for key, msg in self.images.items():
                    if self._stamp(msg) >= t0:
                        msgs.setdefault(key, msg)
            if all((c, s) in msgs for c in self.cameras for s in sides):
                break
            time.sleep(0.02)
        sample = {"time": t0, "cams": {}}
        for cam in self.cameras:
            msg = msgs.get((cam, "rgb"))
            if msg is None:
                print(f"  {cam}: no new image")
                continue
            gray = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_GRAYSCALE)
            det = board.detect(gray)
            entry = {"stamp": self._stamp(msg), "left": {str(i): c.tolist() for i, c in det.items()}}
            if (cam, "right") in msgs:
                gr = cv2.imdecode(np.frombuffer(msgs[(cam, "right")].data, np.uint8), cv2.IMREAD_GRAYSCALE)
                entry["right"] = {str(i): c.tolist() for i, c in board.detect(gr).items()}
            note = ""
            if cam in self.wrists:
                link8 = f"{self.wrists[cam]}_{ARM_ID}_link8"
                F = self._lookup("world", link8, entry["stamp"])
                F0 = self._lookup("world", link8, entry["stamp"] - 0.3)
                if F is None or F0 is None:
                    note = "  no TF at the image stamp, skipped"
                    det = {}
                else:
                    D = np.linalg.inv(F0) @ F
                    moving = np.linalg.norm(D[:3, 3]) > STILL_M or angle_deg(D[:3, :3]) > STILL_DEG
                    if moving:
                        note = "  arm was moving, skipped"
                        det = {}
                    entry["flange"] = F.tolist()
            if len(det) >= MIN_MARKERS:
                sample["cams"][cam] = entry
            print(f"  {cam}: {len(det)} markers{note}")
        return sample

    def close(self) -> None:
        self.node.destroy_node()
        self.rclpy.try_shutdown()


# ---- bundle adjustment ---------------------------------------------------------------------------

def observations(data: dict, board: Board) -> list[dict]:
    obs = []
    for k, sample in enumerate(data["samples"]):
        for cam, e in sample["cams"].items():
            det = {int(i): np.asarray(c) for i, c in e["left"].items()}
            if len(det) < MIN_MARKERS or (cam in data["robot_frames"] and "flange" not in e):
                continue
            obj, img = board.points(det)
            obs.append({"sample": k, "cam": cam, "obj": obj, "img": img,
                        "K": np.asarray(data["intrinsics"][cam]["K"]),
                        "flange": np.asarray(e["flange"]) if "flange" in e else None})
    return obs


def rotation_spread(flanges: list[np.ndarray]) -> float:
    return max((angle_deg(a[:3, :3].T @ b[:3, :3]) for a, b in itertools.combinations(flanges, 2)), default=0.0)


class Problem:
    def __init__(self, data: dict, board: Board) -> None:
        self.board = board
        self.frames = {cam: {k: (np.asarray(v) if v is not None and k != "robot" else v) for k, v in f.items()}
                       for cam, f in data["robot_frames"].items()}
        self.obs = observations(data, board)
        if not self.obs:
            raise RuntimeError("no usable detections")
        self.cams = sorted({o["cam"] for o in self.obs})
        self.static = [c for c in self.cams if c not in self.frames]
        wrists = [c for c in self.cams if c in self.frames]
        # Which mounts / bases the captures can tell apart from the board pose
        self.spread = {c: rotation_spread([o["flange"] for o in self.obs if o["cam"] == c]) for c in wrists}
        self.count = {c: sum(o["cam"] == c for o in self.obs) for c in wrists}
        self.free_mount = [c for c in wrists if self.count[c] >= MIN_SAMPLES and self.spread[c] >= MIN_SPREAD_DEG]
        self.free_base = [c for c in self.free_mount if self.frames[c]["robot"] != REFERENCE_ROBOT
                          and any(w != c for w in wrists)]
        # A wrist of another robot only pulls on the board once its base is solved; until then its
        # detections are just evaluated (their error is the misalignment against the reference).
        for o in self.obs:
            o["used"] = (o["flange"] is None or self.frames[o["cam"]]["robot"] == REFERENCE_ROBOT
                         or o["cam"] in self.free_base)
        if not any(o["used"] and o["flange"] is not None for o in self.obs):
            raise RuntimeError(f"no camera of {REFERENCE_ROBOT} saw the board: nothing ties it to the world")
        self.blocks = (["B"] + [f"M:{c}" for c in self.static] + [f"C:{c}" for c in self.free_mount]
                       + [f"D:{c}" for c in self.free_base])
        self.x0 = self._initial()

    def wrist_pose(self, cam: str, flange: np.ndarray, x: dict | None = None) -> np.ndarray:
        x = x or {}
        C = x.get(f"C:{cam}", np.eye(4))
        D = x.get(f"D:{cam}", np.eye(4))
        return D @ flange @ C @ self.frames[cam]["optical"]

    def _initial(self) -> np.ndarray:
        boards = []
        for o in self.obs:
            if o["flange"] is not None and o["used"]:
                boards.append(self.wrist_pose(o["cam"], o["flange"]) @ self._pnp(o))
        B = boards[0].copy()
        B[:3, 3] = np.mean([b[:3, 3] for b in boards], axis=0)
        x = {"B": B}
        for c in self.static:
            o = next(o for o in self.obs if o["cam"] == c)
            x[f"M:{c}"] = B @ np.linalg.inv(self._pnp(o))
        return self.pack(x)

    def _pnp(self, o: dict) -> np.ndarray:
        ok, rvec, tvec = cv2.solvePnP(o["obj"], o["img"], o["K"], None, flags=cv2.SOLVEPNP_SQPNP)
        rvec, tvec = cv2.solvePnPRefineLM(o["obj"], o["img"], o["K"], None, rvec, tvec)
        T = np.eye(4)
        T[:3, :3], T[:3, 3] = cv2.Rodrigues(rvec)[0], tvec.ravel()
        return T

    def pack(self, x: dict) -> np.ndarray:
        return np.concatenate([to_vec(x.get(b, np.eye(4))) for b in self.blocks])

    def unpack(self, v: np.ndarray) -> dict:
        return {b: from_vec(v[6 * i:6 * i + 6]) for i, b in enumerate(self.blocks)}

    def camera_pose(self, o: dict, x: dict) -> np.ndarray:
        return x[f"M:{o['cam']}"] if o["flange"] is None else self.wrist_pose(o["cam"], o["flange"], x)

    def project(self, o: dict, x: dict) -> np.ndarray:
        T = np.linalg.inv(self.camera_pose(o, x)) @ x["B"]
        p = o["obj"] @ T[:3, :3].T + T[:3, 3]
        uv = p @ o["K"].T
        return uv[:, :2] / uv[:, 2:3]

    def residuals(self, v: np.ndarray) -> np.ndarray:
        x = self.unpack(v)
        return np.concatenate([(self.project(o, x) - o["img"]).ravel() for o in self.obs if o["used"]])

    def rms(self, v: np.ndarray) -> dict:
        x = self.unpack(v)
        out = {}
        for c in self.cams:
            r = np.concatenate([self.project(o, x) - o["img"] for o in self.obs if o["cam"] == c])
            out[c] = float(np.sqrt(np.mean(np.sum(r ** 2, axis=1))))
        return out

    def solve(self) -> tuple[dict, dict]:
        res = least_squares(self.residuals, self.x0, loss="soft_l1", f_scale=1.0, x_scale="jac", max_nfev=200)
        # 1-sigma of every block from the Gauss-Newton covariance
        J = res.jac
        dof = max(len(res.fun) - len(res.x), 1)
        sigma2 = float(np.sum(res.fun ** 2) / dof)
        cov = np.linalg.pinv(J.T @ J) * sigma2
        std = {b: (float(np.degrees(np.sqrt(np.trace(cov[6 * i:6 * i + 3, 6 * i:6 * i + 3])))),
                   float(1e3 * np.sqrt(np.trace(cov[6 * i + 3:6 * i + 6, 6 * i + 3:6 * i + 6]))))
               for i, b in enumerate(self.blocks)}
        return {"x": self.unpack(res.x), "v": res.x, "std": std}, self.rms(res.x)


# ---- writing -------------------------------------------------------------------------------------

def set_yaml_keys(path: Path, values: dict[str, str]) -> None:
    """Sets ``key: "value"`` lines under robot_config, keeping comments and order."""
    lines = path.read_text().splitlines()
    for key, value in values.items():
        line = f'  {key}: "{value}"'
        idx = next((i for i, l in enumerate(lines) if re.match(rf"\s+{key}\s*:", l)), None)
        if idx is None:
            lines.append(line)
        else:
            lines[idx] = line
    path.write_text("\n".join(lines) + "\n")


def studio_port() -> int:
    try:
        cfg = yaml.safe_load((WS / "config/studio.yaml").read_text()) or {}
        return int((cfg.get("server") or {}).get("port", 8090))
    except (OSError, ValueError, yaml.YAMLError):
        return 8090


def save_static(name: str, pose: dict, project: str | None) -> str:
    body = json.dumps({"name": name, **pose}).encode()
    req = urllib.request.Request(f"http://127.0.0.1:{studio_port()}/api/camera/save", body,
                                 {"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=5) as r:
            return f"studio: {json.loads(r.read())}"
    except Exception as e:
        if not project:
            return f"studio not reachable ({e}); pass --project to write camera_poses.yaml directly"
    path = WS / "projects" / project / "camera_poses.yaml"
    poses = (yaml.safe_load(path.read_text()) if path.is_file() else None) or {}
    poses[name] = pose
    path.write_text("# Written by the studio's viewer (Camera placement); merged into env.yaml scene.cameras.\n"
                    + yaml.safe_dump(poses, sort_keys=True, default_flow_style=None))
    return f"wrote {path.relative_to(WS)}"


# ---- report --------------------------------------------------------------------------------------

def report(problem: Problem, result: dict, rms0: dict, rms: dict, write: bool, project: str | None) -> None:
    x, std = result["x"], result["std"]
    print(f"\nboard {problem.board.marker * 1e3:.1f} mm markers, {problem.board.sep * 1e3:.1f} mm gaps; "
          f"{len(problem.obs)} detections")
    for c in problem.cams:
        fitted = "" if any(o["used"] for o in problem.obs if o["cam"] == c) else \
            f"  not fitted (its base is not solved): its error against {REFERENCE_ROBOT}"
        print(f"  {c:16s} reprojection {rms0[c]:5.2f} -> {rms[c]:5.2f} px   "
              f"({sum(o['cam'] == c for o in problem.obs)} views){fitted}")
    for c in problem.spread:
        state = "solved" if c in problem.free_mount else f"kept (needs >= {MIN_SAMPLES} views >= {MIN_SPREAD_DEG:.0f} deg apart)"
        print(f"  {c:16s} wrist rotation spread {problem.spread[c]:5.1f} deg over {problem.count[c]} views: mount {state}")

    # How differently the wrist cameras see the board with the robot description as it is
    wrist_boards = {}
    for o in problem.obs:
        if o["flange"] is not None and len(o["obj"]) >= 16:   # fewer markers: the pose can flip
            wrist_boards.setdefault(o["cam"], []).append(problem.wrist_pose(o["cam"], o["flange"]) @ problem._pnp(o))
    centre = np.append(np.mean(list(problem.board.obj.values()), axis=(0, 1)), 1.0)
    if len(wrist_boards) > 1:
        print("\nboard seen through the robot description (before):")
        (a, A), (b, Bs) = list(wrist_boards.items())[:2]
        Am, Bm = A[0], Bs[0]
        d = np.linalg.norm((Am @ centre)[:3] - (Bm @ centre)[:3])
        print(f"  {a} vs {b}: {1e3 * d:.1f} mm apart at the board centre, {angle_deg(Am[:3, :3].T @ Bm[:3, :3]):.2f} deg")

    print("\nresult (1 sigma: image noise only, not the error of a mount that was kept):")
    B = x["B"]
    print(f"  board centre in world {np.round((B @ centre)[:3], 4).tolist()}  "
          f"(+-{std['B'][0]:.2f} deg, {std['B'][1]:.1f} mm), "
          f"{np.degrees(np.arccos(abs(B[2, 2]))):.1f} deg from horizontal (lying on the table: ~0)")
    for c in problem.static:
        M = x[f"M:{c}"]
        print(f"  {c}: {pose_dict(M)}  (+-{std[f'M:{c}'][0]:.2f} deg, {std[f'M:{c}'][1]:.1f} mm)")
        if write:
            print("    " + save_static(c, pose_dict(M), project))
    for c in problem.free_mount:
        f = problem.frames[c]
        C = x[f"C:{c}"] @ (f["mount"] if f["mount"] is not None else np.eye(4))
        xyz, rpy = xyz_rpy(C)
        delta = x[f"C:{c}"]
        print(f"  {c} mount correction: {1e3 * np.linalg.norm(delta[:3, 3]):.1f} mm, {angle_deg(delta[:3, :3]):.2f} deg "
              f"(+-{std[f'C:{c}'][0]:.2f} deg, {std[f'C:{c}'][1]:.1f} mm)")
        if f["mount"] is None:
            print("    (no TF of the mount frame: the robot runs an older description; the value assumes none)")
        values = dict(zip(["cam_pos_x", "cam_pos_y", "cam_pos_z", "cam_roll", "cam_pitch", "cam_yaw"],
                          [f"{v:.6f}" for v in xyz + rpy]))
        print("    " + ", ".join(f"{k}: {v}" for k, v in values.items()))
        if write:
            path = Path(str(LAUNCH_CONFIG).format(side=f["robot"].split("_")[-1]))
            set_yaml_keys(path, values)
            print(f"    wrote {path.relative_to(WS)}")
    for c in problem.free_base:
        f = problem.frames[c]
        D = x[f"D:{c}"]
        print(f"  {f['robot']} base correction: {1e3 * np.linalg.norm(D[:3, 3]):.1f} mm, {angle_deg(D[:3, :3]):.2f} deg "
              f"(+-{std[f'D:{c}'][0]:.2f} deg, {std[f'D:{c}'][1]:.1f} mm)")
        if f["base"] is None:
            print("    (no TF world <- link0: not written)")
            continue
        xyz, rpy = xyz_rpy(D @ f["base"])
        values = {"xyz": " ".join(f"{v:.6f}" for v in xyz), "rpy": " ".join(f"{v:.6f}" for v in rpy)}
        print("    " + ", ".join(f'{k}: "{v}"' for k, v in values.items()))
        if write:
            path = Path(str(LAUNCH_CONFIG).format(side=f["robot"].split("_")[-1]))
            set_yaml_keys(path, values)
            print(f"    wrote {path.relative_to(WS)}")
    if write and (problem.free_mount or problem.free_base):
        print("  restart the robots (Robots component) to load the new description")


# ---- main ----------------------------------------------------------------------------------------

def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--samples", type=Path, help="solve these saved captures (no ROS)")
    ap.add_argument("--out", type=Path, help="where captures are saved (default calibration/board_<time>.json)")
    ap.add_argument("--once", action="store_true", help="capture once, solve, exit")
    ap.add_argument("--dict", default="DICT_4X4_50")
    ap.add_argument("--cols", type=int, default=3)
    ap.add_argument("--rows", type=int, default=4)
    ap.add_argument("--first-id", type=int, default=0)
    ap.add_argument("--marker", type=float, help="printed marker side (m), measured")
    ap.add_argument("--sep", type=float, help="printed gap between markers (m), measured")
    ap.add_argument("--write", action="store_true", help="store the result (see above)")
    ap.add_argument("--project", help="project for camera_poses.yaml when the studio is not running")
    args = ap.parse_args()
    if (args.marker is None) != (args.sep is None):
        ap.error("pass both --marker and --sep, or neither")

    board = Board(args.dict, args.cols, args.rows, args.first_id)
    if args.samples:
        data = json.loads(args.samples.read_text())
    else:
        data = capture(board, args)
    if args.marker is not None:
        board.set_size(args.marker, args.sep)
    elif data.get("board", {}).get("marker", 1.0) != 1.0:
        board.set_size(data["board"]["marker"], data["board"]["sep"])
    else:
        size_from_stereo(board, data)
    problem = Problem(data, board)
    rms0 = problem.rms(problem.x0)
    result, rms = problem.solve()
    report(problem, result, rms0, rms, args.write, args.project)


def size_from_stereo(board: Board, data: dict) -> None:
    views = []
    for sample in data["samples"]:
        for cam, e in sample["cams"].items():
            info = data["intrinsics"][cam]
            if e.get("right") and info.get("baseline"):
                views.append({"K": np.asarray(info["K"]), "baseline": info["baseline"],
                              "left": {int(i): np.asarray(c) for i, c in e["left"].items()},
                              "right": {int(i): np.asarray(c) for i, c in e["right"].items()}})
    marker, sep, spread = estimate_size(board, views)
    board.set_size(marker, sep)
    data["board"] = board.to_dict()
    print(f"board size from stereo: {marker * 1e3:.1f} mm markers, {sep * 1e3:.1f} mm gaps "
          f"(spread between views {100 * spread:.1f} %); measure them and pass --marker/--sep for accuracy")


def capture(board: Board, args) -> dict:
    cameras = STATIC + list(WRISTS)
    cap = Capture(cameras, WRISTS, stereo=args.marker is None)
    out = args.out or WS / "calibration" / time.strftime("board_%Y%m%d_%H%M%S.json")
    out.parent.mkdir(parents=True, exist_ok=True)
    try:
        data = {"board": board.to_dict() if args.marker is None else {**board.to_dict(), "marker": args.marker, "sep": args.sep},
                "intrinsics": cap.intrinsics(), "samples": []}
        time.sleep(1.0)  # TF buffer
        data["robot_frames"] = cap.robot_frames()
        if not args.once:
            print("Keep the board still. For the wrist mounts: move an arm to a pose with the board in view, "
                  "hold still, press Enter; repeat with the wrist turned about different axes.")
        while True:
            if not args.once:
                cmd = input(f"[{len(data['samples'])} captures] Enter = capture, s = solve, q = solve and quit: ").strip().lower()
                if cmd in ("s", "q"):
                    if cmd == "q":
                        break
                    try:
                        trial = Problem(data, Board(args.dict, args.cols, args.rows, args.first_id,
                                                    *(data["board"]["marker"], data["board"]["sep"])))
                        _, rms = trial.solve()
                        print("  reprojection " + ", ".join(f"{c} {v:.2f} px" for c, v in rms.items())
                              + f"; mounts solvable: {trial.free_mount or 'none'}")
                    except Exception as e:
                        print(f"  {e}")
                    continue
            sample = cap.grab(board)
            if sample["cams"]:
                data["samples"].append(sample)
                if args.marker is None and data["board"]["marker"] == 1.0:
                    size_from_stereo(board, data)
                out.write_text(json.dumps(data))
                print(f"  saved {out}")
            if args.once:
                break
    finally:
        cap.close()
    return data


if __name__ == "__main__":
    main()
