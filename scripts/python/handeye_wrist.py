"""Hand-eye calibration of a wrist camera from its own point clouds (no target needed).

    pixi run -e humble python scripts/python/handeye_wrist.py zed_left_arm [--robot franka_left]
        [--poses 6] [--timeout 240] [--apply]

While it runs, move that arm (teleop or hand guiding) through a few different poses and hold each
for ~3 s, keeping the table in the camera's view; tilt the wrist about different axes, not only
translate. At every still pose the camera's cloud (``/ffs/<camera>/points``, optical frame, from
the point cloud node) and the flange pose (TF ``world <- <robot>_fr3_link8``) are recorded.

The flange -> camera transform X is the one that makes all recorded clouds of the static scene
coincide in the world (each pose's cloud mapped with flange_i * X): a joint point-to-plane
optimisation over all poses, started from the 24 axis-aligned rotations of the robot
description's camera frame (and those turned by 180 deg about the flange axis), keeping the most
consistent result. Pairwise ICP between two views does not work here: the mostly flat table lets
two views slide, and a description that is 180 deg off gives no usable start.

``--apply`` sends it to the point cloud node as the camera's extrinsics (relative to the flange),
so the merged cloud uses it right away; the printed xacro values make it permanent.
"""

from __future__ import annotations

import argparse
import itertools
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import PointCloud2
from tf2_ros import Buffer, TransformListener

WS = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(WS / "studio"))
sys.path.insert(0, str(WS.parent / "Fast-FoundationStereo" / "jetson"))
from franka_studio.env.scene import decode_pointcloud2  # noqa: E402
from icp import icp, voxel_down  # noqa: E402

STILL_M, STILL_DEG = 0.003, 0.5     # flange motion within 0.4 s that still counts as holding still
NEW_M, NEW_DEG = 0.04, 8.0          # a pose this different from all recorded ones is a new one
MIN_PAIR_DEG = 6.0                  # pairs with less rotation say little about the rotation


def to_matrix(tf) -> np.ndarray:
    t, q = tf.transform.translation, tf.transform.rotation
    M = np.eye(4)
    M[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    M[:3, 3] = [t.x, t.y, t.z]
    return M


def angle_deg(Rm: np.ndarray) -> float:
    return float(np.degrees(np.arccos(np.clip((np.trace(Rm) - 1) / 2, -1, 1))))


def motion(A: np.ndarray, B: np.ndarray) -> tuple[float, float]:
    D = np.linalg.inv(A) @ B
    return float(np.linalg.norm(D[:3, 3])), angle_deg(D[:3, :3])


def axis_rotations() -> list[np.ndarray]:
    """The 24 rotations mapping coordinate axes onto coordinate axes."""
    out = []
    for perm in itertools.permutations(range(3)):
        for signs in itertools.product((1, -1), repeat=3):
            Rm = np.zeros((3, 3))
            for r, (c, sg) in enumerate(zip(perm, signs)):
                Rm[r, c] = sg
            if np.linalg.det(Rm) > 0:
                out.append(Rm)
    return out


def consistency(snaps: list, X: np.ndarray, voxel: float = 0.01, dist: float = 0.012) -> float:
    """Share of points of each pose that lie within ``dist`` of another pose's cloud (world)."""
    worlds = [voxel_down(c @ (P @ X)[:3, :3].T + (P @ X)[:3, 3], voxel) for P, c in snaps]
    shares = []
    for i, w in enumerate(worlds):
        others = np.vstack([o for j, o in enumerate(worlds) if j != i])
        shares.append(np.isfinite(cKDTree(others).query(w, distance_upper_bound=dist)[0]).mean())
    return float(np.mean(shares))


def refine(snaps: list, X: np.ndarray, schedule=((0.08, 0.02), (0.04, 0.012), (0.02, 0.008), (0.01, 0.005)),
           iterations: int = 12) -> np.ndarray:
    """Joint point-to-plane refinement of X over all poses (each pose against all the others)."""
    from icp import normals

    X = X.copy()
    for max_dist, voxel in schedule:
        clouds = [voxel_down(c, voxel) for _, c in snaps]
        for _ in range(iterations):
            worlds = [c @ (P @ X)[:3, :3].T + (P @ X)[:3, 3] for (P, _), c in zip(snaps, clouds)]
            rows, res = [], []
            for i, ((P, _), c, w) in enumerate(zip(snaps, clouds, worlds)):
                tgt = np.vstack([o for j, o in enumerate(worlds) if j != i])
                tree = cKDTree(tgt)
                d, idx = tree.query(w, distance_upper_bound=max_dist)
                m = np.isfinite(d)
                if m.sum() < 20:
                    continue
                q = tgt[idx[m]]
                nrm = normals(tgt, tree)[idx[m]] if len(tgt) < 60000 else normals(q, cKDTree(q))
                r = np.einsum("ij,ij->i", w[m] - q, nrm)
                a = nrm @ (P @ X)[:3, :3]                      # n expressed in the camera frame
                rows.append(np.hstack([np.cross(c[m], a), a]))
                res.append(r)
            if not rows:
                break
            J, r = np.vstack(rows), np.concatenate(res)
            delta = 0.5 * max_dist
            wgt = np.where(np.abs(r) <= delta, 1.0, delta / np.maximum(np.abs(r), 1e-12))
            Jw = J * wgt[:, None]
            x = np.linalg.lstsq(Jw.T @ J + 1e-9 * np.eye(6), -(Jw.T @ r), rcond=None)[0]
            D = np.eye(4)
            D[:3, :3] = Rotation.from_rotvec(x[:3]).as_matrix()
            D[:3, 3] = x[3:]
            X = X @ D
            if np.linalg.norm(x) < 1e-6:
                break
    return X


def calibrate(snaps: list, X0: np.ndarray) -> tuple[np.ndarray, float]:
    starts = []
    flip = np.eye(4)
    flip[:3, :3] = Rotation.from_rotvec([0, 0, np.pi]).as_matrix()   # about the flange axis
    for base in (X0, flip @ X0):
        for Rk in axis_rotations():
            S = base.copy()
            S[:3, :3] = base[:3, :3] @ Rk
            starts.append(S)
    coarse = sorted(((consistency(snaps, S, voxel=0.02, dist=0.03), k) for k, S in enumerate(starts)), reverse=True)
    best = (-1.0, None)
    for _, k in coarse[:8]:                                # refine the most promising starts
        X = refine(snaps, starts[k])
        sc = consistency(snaps, X)
        if sc > best[0]:
            best = (sc, X)
    return best[1], best[0]


def solve_ax_xb(As: list, Bs: list) -> np.ndarray:
    alphas = [Rotation.from_matrix(A[:3, :3]).as_rotvec() for A in As]
    betas = [Rotation.from_matrix(B[:3, :3]).as_rotvec() for B in Bs]
    M = sum(np.outer(b, a) for a, b in zip(alphas, betas))
    w, V = np.linalg.eigh(M.T @ M)
    Rx = V @ np.diag(1.0 / np.sqrt(np.maximum(w, 1e-12))) @ V.T @ M.T
    u, _, vt = np.linalg.svd(Rx)
    Rx = u @ np.diag([1, 1, np.linalg.det(u @ vt)]) @ vt
    C = np.vstack([A[:3, :3] - np.eye(3) for A in As])
    d = np.concatenate([Rx @ B[:3, 3] - A[:3, 3] for A, B in zip(As, Bs)])
    tx = np.linalg.lstsq(C, d, rcond=None)[0]
    X = np.eye(4)
    X[:3, :3], X[:3, 3] = Rx, tx
    return X


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("camera")
    ap.add_argument("--robot", help="default: franka_left for *left*, franka_right for *right*")
    ap.add_argument("--poses", type=int, default=6)
    ap.add_argument("--timeout", type=float, default=240.0)
    ap.add_argument("--zmax", type=float, default=1.2, help="m, camera points used for ICP")
    ap.add_argument("--apply", action="store_true", help="send the result to the point cloud node")
    args = ap.parse_args()
    robot = args.robot or ("franka_left" if "left" in args.camera else "franka_right")
    flange = f"{robot}_fr3_link8"
    optical = f"{robot}_zed_left_camera_optical_frame"

    rclpy.init()
    node = rclpy.create_node("handeye_wrist")
    tf_buffer = Buffer()
    TransformListener(tf_buffer, node)
    latest: list = []
    node.create_subscription(PointCloud2, f"/ffs/{args.camera}/points", lambda m: latest.append(m),
                             QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE))

    snaps: list[tuple[np.ndarray, np.ndarray]] = []   # (flange pose in world, camera cloud)
    print(f"Recording {args.camera} on {robot}: move the arm to {args.poses} different poses, "
          "hold each ~3 s, keep the table in view.")
    t_end = time.time() + args.timeout
    while len(snaps) < args.poses and time.time() < t_end and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.05)
        if not latest:
            continue
        msg = latest.pop()
        latest.clear()
        stamp = Time.from_msg(msg.header.stamp)
        try:
            P = to_matrix(tf_buffer.lookup_transform("world", flange, stamp))
            P0 = to_matrix(tf_buffer.lookup_transform("world", flange, stamp - rclpy.duration.Duration(seconds=0.4)))
        except Exception:
            continue
        dm, dd = motion(P0, P)
        if dm > STILL_M or dd > STILL_DEG:
            continue
        if any(motion(S, P)[0] < NEW_M and motion(S, P)[1] < NEW_DEG for S, _ in snaps):
            continue
        xyz = decode_pointcloud2(msg, 10**7)[0]
        xyz = xyz[xyz[:, 2] < args.zmax]
        snaps.append((P, xyz))
        print(f"  pose {len(snaps)}/{args.poses}: {len(xyz)} points, flange at {P[:3, 3].round(3)}")
    if len(snaps) < 3:
        sys.exit(f"Only {len(snaps)} poses recorded; need at least 3.")

    X0 = to_matrix(tf_buffer.lookup_transform(flange, optical, Time()))   # robot description
    X, score = calibrate(snaps, X0)
    print(f"\nbest start reaches {score:.0%} consistency")
    D = np.linalg.inv(X0) @ X
    rv = Rotation.from_matrix(D[:3, :3]).as_rotvec()
    print(f"\nflange -> camera (optical) X: xyz {X[:3, 3].round(4)}, "
          f"rpy {Rotation.from_matrix(X[:3, :3]).as_euler('xyz').round(4)}")
    print(f"vs robot description: {np.degrees(np.linalg.norm(rv)):.1f}° about optical axis "
          f"{(rv / (np.linalg.norm(rv) + 1e-12)).round(3)}, offset {np.linalg.norm(D[:3, 3]) * 100:.1f} cm")

    # check: all snapshots merged in the world with X vs with the description
    for name, Xc in (("description", X0), ("calibrated", X)):
        worlds = [voxel_down(c @ (P @ Xc)[:3, :3].T + (P @ Xc)[:3, 3], 0.01) for P, c in snaps]
        ov = []
        for i, j in itertools.combinations(range(len(worlds)), 2):
            d = cKDTree(worlds[i]).query(worlds[j], distance_upper_bound=0.015)[0]
            ov.append(np.isfinite(d).mean())
        table = np.mean([np.mean(np.abs(w[:, 2] - 0.02) < 0.012) for w in worlds])
        print(f"  {name:12s}: clouds of different poses overlap {np.mean(ov):.0%}, points on the table {table:.0%}")

    if args.apply:
        from std_msgs.msg import String
        import json

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        current: list = []
        node.create_subscription(String, "/ffs/extrinsics", lambda m: current.append(json.loads(m.data)), latched)
        t = time.time()
        while not current and time.time() - t < 5:
            rclpy.spin_once(node, timeout_sec=0.1)
        ext = current[-1] if current else {}
        x, y, z, w = Rotation.from_matrix(X[:3, :3]).as_quat()
        ext[args.camera] = {"frame": flange, "translate": X[:3, 3].tolist(), "rotate": [w, x, y, z]}
        pub = node.create_publisher(String, "/ffs/extrinsics", latched)
        pub.publish(String(data=json.dumps(ext)))
        t = time.time()
        while time.time() - t < 2:
            rclpy.spin_once(node, timeout_sec=0.1)
        print(f"Sent to the point cloud node: {ext[args.camera]}")


if __name__ == "__main__":
    main()
