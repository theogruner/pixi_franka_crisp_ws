"""Automatic camera calibration: view planning, saving / applying (no ROS needed)."""

import json
import types

import numpy as np
import pytest
import yaml
from scipy.spatial.transform import Rotation

from franka_studio import calibration as cal
from franka_studio.config import DEFAULTS

bc = cal.bc


def _planner():
    cfg = DEFAULTS["calibration"]
    b = cfg["board"]
    obj = types.SimpleNamespace(cfg=cfg, board=bc.Board(b["dict"], b["cols"], b["rows"], b["first_id"], b["marker"], b["sep"]),
                                logs=[])
    obj._log = obj.logs.append
    return obj


def _look(eye, target):
    z = np.asarray(target, float) - eye
    z /= np.linalg.norm(z)
    x = np.cross(z, [0, 0, 1.0])
    x /= np.linalg.norm(x)
    T = np.eye(4)
    T[:3, :3] = np.stack([x, np.cross(z, x), z], 1)
    T[:3, 3] = eye
    return T


def test_plan_views_look_at_the_board_within_limits():
    p = _planner()
    B = np.eye(4)
    B[:3, :3] = Rotation.from_euler("x", 180, degrees=True).as_matrix()   # board z down, like the real one
    B[:3, 3] = [0.2, -0.05, 0.01]
    centre = (B @ np.append(np.mean(list(p.board.obj.values()), axis=(0, 1)), 1))[:3]
    X = np.eye(4)                                            # link8 <- camera: some unknown holder
    X[:3, :3] = Rotation.from_euler("xyz", [-40, 5, 135], degrees=True).as_matrix()
    X[:3, 3] = [0.09, -0.05, 0.03]
    C0 = _look(centre + [0.05, 0.25, 0.35], centre)          # camera at the start, seeing the board
    F0 = C0 @ np.linalg.inv(X)
    L8G = np.eye(4)
    L8G[:3, 3] = [0, 0, 0.103]
    G0 = F0 @ L8G
    targets = cal.AutoCalibration._plan(p, "zed_right_arm", X, B, G0, F0)
    assert len(targets) >= 6, p.logs
    cfg = p.cfg
    for G in targets:
        Tc = G @ np.linalg.inv(L8G) @ X                      # camera of that target
        to_board = centre - Tc[:3, 3]
        assert np.degrees(np.arccos(Tc[:3, 2] @ to_board / np.linalg.norm(to_board))) < 1.0   # looks at it
        assert np.linalg.norm(G[:3, 3] - G0[:3, 3]) <= cfg["max_offset"] + 1e-9
        assert bc.angle_deg(G0[:3, :3].T @ G[:3, :3]) <= cfg["max_rotation_deg"] + 1e-9
        assert G[2, 3] - centre[2] >= cfg["min_clearance"]
    # enough rotation variety for the hand-eye solve
    assert bc.rotation_spread([G0] + targets) >= bc.MIN_SPREAD_DEG


def test_save_and_apply(tmp_path, monkeypatch):
    monkeypatch.setattr(cal, "CALIB_DIR", tmp_path / "calibrations")
    side_cfg = tmp_path / "dfki_fr3_{side}.yaml"
    for side in ("right", "left"):
        (tmp_path / f"dfki_fr3_{side}.yaml").write_text(
            f'robot_config:\n  arm_prefix: "franka_{side}"\n  xyz: "0 0 0"\n  rpy_ee: "0 0 2.356194490"\n')
    monkeypatch.setattr(bc, "LAUNCH_CONFIG", side_cfg)
    saved = []
    obj = types.SimpleNamespace(save_static=lambda n, p, w: saved.append((n, p, w)))
    obj._mount_config = cal.AutoCalibration._mount_config
    obj.result = {"values": {"static": {"zed_middle": {"position": [0.1, 0.2, 0.3], "wxyz": [1, 0, 0, 0]}},
                             "robots": {"franka_left": {"cam_pos_x": "0.010000", "xyz": "0.1 0.6 0"}}, "notes": []},
                  "rms": {"zed_middle": 0.9}, "mount_config": cal.AutoCalibration._mount_config()}
    assert not cal.AutoCalibration.save(obj, "bad name")["ok"]
    assert cal.AutoCalibration.save(obj, "holders_v2")["ok"]
    obj_list = cal.AutoCalibration.saved(obj)
    assert [c["name"] for c in obj_list] == ["holders_v2"]
    r = cal.AutoCalibration.apply(obj, "holders_v2")
    assert r["ok"] and r["restart_robots"], r
    left = yaml.safe_load((tmp_path / "dfki_fr3_left.yaml").read_text())["robot_config"]
    assert left["cam_pos_x"] == "0.010000" and left["xyz"] == "0.1 0.6 0"
    assert saved == [("zed_middle", [0.1, 0.2, 0.3], [1, 0, 0, 0])]
    assert not cal.AutoCalibration.apply(obj, "missing")["ok"]


@pytest.mark.parametrize("name,ok", [("holders_v2", True), ("2026-10-29.a", True), ("", False), ("../x", False), ("a b", False)])
def test_valid_name(name, ok):
    assert cal.valid_name(name) is ok


# ---- solve: synthetic rig with a known answer ----------------------------------------------------------

K = np.array([[370.0, 0, 480], [0, 370, 300], [0, 0, 1]])
OPTICAL = np.eye(4)                                          # link8 <- optical in the description
OPTICAL[:3, :3] = Rotation.from_euler("xyz", [0, 0, 135], degrees=True).as_matrix()
OPTICAL[:3, 3] = [0.04, -0.04, 0.05]
MOUNT_ERROR = np.eye(4)                                      # what the description gets wrong
MOUNT_ERROR[:3, :3] = Rotation.from_euler("xyz", [0.6, -0.4, 0.3], degrees=True).as_matrix()
MOUNT_ERROR[:3, 3] = [0.003, -0.002, 0.004]
MIDDLE = _look(np.array([-0.15, 0.07, 0.56]), [0.35, 0.0, 0.0])


def _board_pose(p: int) -> np.ndarray:
    B = np.eye(4)
    B[:3, :3] = Rotation.from_euler("z", 20 * p, degrees=True).as_matrix()
    B[:3, 3] = [[0.25, -0.05, 0.0], [0.42, 0.12, 0.0], [0.3, -0.2, 0.0]][p]
    return B


def _scene_points() -> np.ndarray:
    """The table and two boxes, as points (m)."""
    def grid(o, u, v, step=0.006):
        a, b = np.meshgrid(np.arange(0, 1, step / np.linalg.norm(u)), np.arange(0, 1, step / np.linalg.norm(v)))
        return o + a.reshape(-1, 1) * u + b.reshape(-1, 1) * v

    pts = [grid(np.array([-0.05, -0.45, 0.0]), np.array([0.8, 0, 0]), np.array([0, 0.9, 0]))]
    for o, size in (((0.45, 0.2, 0.0), (0.1, 0.08, 0.12)), ((0.15, -0.3, 0.0), (0.06, 0.15, 0.08))):
        o, (sx, sy, sz) = np.array(o), size
        X, Y, Z = np.array([sx, 0, 0]), np.array([0, sy, 0]), np.array([0, 0, sz])
        pts += [grid(o + Z, X, Y), grid(o, X, Z), grid(o + Y, X, Z), grid(o, Y, Z), grid(o + X, Y, Z)]
    return np.concatenate(pts)


def _cloud(T: np.ndarray, scene: np.ndarray, scale: float, rng) -> np.ndarray:
    """What a camera at ``T`` with a depth reading 1/scale of the truth sees of the scene."""
    p = (scene - T[:3, 3]) @ T[:3, :3]
    uv = p @ K.T
    inside = (p[:, 2] > 0.1) & (p[:, 2] < 1.5)
    uv = uv[:, :2] / np.maximum(uv[:, 2:], 1e-9)
    inside &= (uv[:, 0] > 0) & (uv[:, 0] < 960) & (uv[:, 1] > 0) & (uv[:, 1] < 600)
    p = p[inside] / scale
    return bc.cloud_refine.thin(p + rng.normal(0, 0.0008, p.shape))


def _session(views: int, placements=(0,), base=np.eye(4), clouds=False, middle_scale=1.0, seed=0):
    """Captures of a rig with the board at ``placements``; the robot's TF base is ``base`` (the
    arm itself is where it is)."""
    rng = np.random.default_rng(seed)
    board = _planner().board
    scene = _scene_points() if clouds else None

    def detect(T_cam, B):
        out = {}
        for i, obj in board.obj.items():
            p = (np.c_[obj, np.ones(4)] @ (np.linalg.inv(T_cam) @ B).T)[:, :3] @ K.T
            out[str(i)] = (p[:, :2] / p[:, 2:] + rng.normal(0, 0.15, (4, 2))).tolist()
        return out

    samples, cloud_data = [], {}
    for p in placements:
        B = _board_pose(p)
        centre = (B @ np.append(np.mean(list(board.obj.values()), axis=(0, 1)), 1))[:3]
        for k in range(views):
            az, el = 2 * np.pi * k / views, np.radians([55, 75][k % 2])
            eye = centre + 0.4 * np.array([np.cos(az) * np.cos(el), np.sin(az) * np.cos(el), np.sin(el)])
            cam = _look(eye, centre)
            cam[:3, :3] = cam[:3, :3] @ Rotation.from_euler("z", [-20, 20][k % 2], degrees=True).as_matrix()
            flange = cam @ np.linalg.inv(MOUNT_ERROR @ OPTICAL)   # true link8 pose
            n = len(samples)
            samples.append({"placement": p, "cams": {
                "zed_middle": {"left": detect(MIDDLE, B)},
                "zed_right_arm": {"left": detect(cam, B), "flange": (base @ flange).tolist()}}})
            if clouds:
                cloud_data[(n, "zed_middle")] = _cloud(MIDDLE, scene, middle_scale, rng)
                cloud_data[(n, "zed_right_arm")] = _cloud(cam, scene, 1.0, rng)
    data = {"board": board.to_dict(), "intrinsics": {c: {"K": K.tolist()} for c in ("zed_middle", "zed_right_arm")},
            "robot_frames": {"zed_right_arm": {"robot": "franka_right", "optical": OPTICAL.tolist(), "mount": None,
                                               "base": base.tolist()}},
            "samples": samples}
    return data, cloud_data, board


def _errors(result):
    """mm / deg off the truth: the static camera, the wrist camera on its flange."""
    M = result["x"]["M:zed_middle"]
    C = result["x"]["C:zed_right_arm"]
    camera = C @ OPTICAL                                       # solved link8 <- camera
    truth = MOUNT_ERROR @ OPTICAL
    return (1000 * np.linalg.norm(M[:3, 3] - MIDDLE[:3, 3]), bc.angle_deg(M[:3, :3].T @ MIDDLE[:3, :3]),
            1000 * np.linalg.norm(camera[:3, 3] - truth[:3, 3]), bc.angle_deg(camera[:3, :3].T @ truth[:3, :3]))


def test_placements_are_solved_together():
    data, _, board = _session(12, placements=(0, 1))
    problem = bc.Problem(data, board, reference_robot="franka_right")
    result, rms = problem.solve()
    assert problem.placements == [0, 1] and {"B:0", "B:1"} <= set(result["x"])
    m_mm, m_deg, c_mm, c_deg = _errors(result)
    assert m_mm < 2 and m_deg < 0.1 and c_mm < 1 and c_deg < 0.1, (m_mm, m_deg, c_mm, c_deg)
    for p in (0, 1):
        assert np.linalg.norm(result["x"][f"B:{p}"][:3, 3] - _board_pose(p)[:3, 3]) < 0.002
    assert problem.weak(result) == []


def test_weak_run_is_flagged():
    data, _, board = _session(4)
    problem = bc.Problem(data, board, reference_robot="franka_right")
    result, _ = problem.solve()
    weak = problem.weak(result)
    assert any(w.startswith("zed_right_arm: board in 4 views") for w in weak), weak
    # an arm meant to be calibrated that never got its mount solved counts too
    assert any("zed_left_arm" in w for w in problem.weak(result, ["zed_right_arm", "zed_left_arm"]))


def test_merged_sessions_keep_the_first_robot_base():
    moved = np.eye(4)                                         # a base correction applied in between
    moved[:3, :3] = Rotation.from_euler("z", 0.5, degrees=True).as_matrix()
    moved[:3, 3] = [0.005, -0.003, 0.002]
    a = _session(10, placements=(0,), seed=1)
    b = _session(10, placements=(1,), base=moved, seed=2)     # the board elsewhere
    for s in b[0]["samples"]:
        s["placement"] = 0                                    # each session numbers its own
    data, _ = bc.merge_captures([a[:2], b[:2]])
    assert [s["placement"] for s in data["samples"]] == [0] * 10 + [1] * 10
    problem = bc.Problem(data, a[2], reference_robot="franka_right")
    result, _ = problem.solve()
    m_mm, m_deg, c_mm, c_deg = _errors(result)
    assert m_mm < 2 and m_deg < 0.1 and c_mm < 1 and c_deg < 0.1, (m_mm, m_deg, c_mm, c_deg)


def test_clouds_fine_tune_and_find_a_depth_bias(tmp_path):
    data, clouds, board = _session(10, clouds=True, middle_scale=1.02)
    path = tmp_path / "captures.json"
    bc.save_clouds(bc.clouds_path(path), clouds)
    loaded = bc.load_clouds(bc.clouds_path(path))
    assert set(loaded) == set(clouds)
    problem = bc.Problem(data, board, reference_robot="franka_right", clouds=loaded)
    result, rms = problem.solve()
    cloud = result["cloud"]
    assert cloud["used"] and not cloud["warnings"], cloud
    assert json.loads(json.dumps(bc.summary(problem, result, rms, rms)))["cloud"]["used"]
    assert abs(cloud["scale"]["zed_middle"] / cloud["scale"]["zed_right_arm"] - 1.02) < 0.004, cloud["scale"]
    assert cloud["after"]["zed_right_arm"]["mm"] < 1.5, cloud
    m_mm, m_deg, c_mm, c_deg = _errors(result)
    assert m_mm < 3 and m_deg < 0.15 and c_mm < 1.5 and c_deg < 0.15, (m_mm, m_deg, c_mm, c_deg)
    assert rms["zed_middle"] < 0.5
