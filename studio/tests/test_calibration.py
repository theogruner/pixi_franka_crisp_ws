"""Automatic camera calibration: view planning, saving / applying (no ROS needed)."""

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
