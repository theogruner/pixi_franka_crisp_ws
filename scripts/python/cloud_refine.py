"""Fine-tuning of the board calibration with the cameras' own point clouds.

The board pins every camera down where it lies; away from it, a small rotation error of a wrist
mount, a robot base or a static camera grows with the distance (1 deg: ~9 mm at 0.5 m). Here the
point clouds captured at the same still poses (``/ffs/<camera>/points``, camera frame) add the
point-to-plane distances between each static camera's cloud and every wrist camera's cloud of the
same capture -- the whole scene the two share (table, objects, robots), not only the board -- to
the board's reprojection errors, and everything is solved again. If that moves a pose by more than
WARN_MM / WARN_DEG, board and clouds disagree (with a small board a rotation about it is what the
board pins down worst, and a noisy far cloud can pull it): the board solution is kept.

One depth scale per camera (``world = T @ (scale * p)``) absorbs a stereo depth bias, so a camera
whose depth reads 1 % short does not drag the poses; it is reported (1.010: 1 % short). Clouds of a
camera whose images are flipped (``flip_images`` of the point cloud node) are not supported.
"""

from __future__ import annotations

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

VOXEL = 0.006                  # m, grid of a stored cloud
MAX_POINTS = 4000              # per stored cloud
SCHEDULE = (0.03, 0.02, 0.012, 0.008)   # m, largest point distance of a correspondence per round
SIGMA = 0.002                  # m of point-to-plane distance weighed like 1 px of reprojection error
SCALE_PRIOR = 0.02             # 1-sigma of a depth scale around 1
FLATNESS = 0.08                # smallest / summed eigenvalue of a target point's neighbourhood
STATS_DISTANCE = 0.02          # m, correspondences the reported distances are taken over
MIN_MATCHES = 300              # in total, for a round to count
WARN_DEG, WARN_MM = 1.5, 15.0  # a pose moved more than this: clouds and board disagree


def thin(xyz: np.ndarray, voxel: float = VOXEL, max_points: int = MAX_POINTS, seed: int = 0) -> np.ndarray:
    """One point per voxel, at most ``max_points`` (random), float32."""
    xyz = np.asarray(xyz, np.float64)
    xyz = xyz[np.all(np.isfinite(xyz), axis=1) & (xyz[:, 2] > 0.05)]
    if not len(xyz):
        return np.zeros((0, 3), np.float32)
    _, idx = np.unique(np.floor(xyz / voxel).astype(np.int64), axis=0, return_index=True)
    xyz = xyz[np.sort(idx)]
    if len(xyz) > max_points:
        xyz = xyz[np.sort(np.random.default_rng(seed).choice(len(xyz), max_points, replace=False))]
    return xyz.astype(np.float32)


def normals(xyz: np.ndarray, k: int = 16) -> tuple[np.ndarray, np.ndarray]:
    """Unit normals (smallest principal axis of the k nearest neighbours) and which points lie on a
    surface flat enough for a point-to-plane distance."""
    _, idx = cKDTree(xyz).query(xyz, k=min(k, len(xyz)))
    nb = xyz[idx] - xyz[idx].mean(axis=1, keepdims=True)
    vals, vecs = np.linalg.eigh(np.einsum("nki,nkj->nij", nb, nb))
    return vecs[:, :, 0], vals[:, 0] < FLATNESS * np.maximum(vals.sum(axis=1), 1e-12)


def _transform(T: np.ndarray, p: np.ndarray) -> np.ndarray:
    return p @ T[:3, :3].T + T[:3, 3]


class CloudTerm:
    """Point-to-plane residuals between static and wrist camera clouds of the same captures.

    ``clouds``: {(sample index, camera): (N, 3) points in that camera's optical frame}."""

    def __init__(self, problem, clouds: dict) -> None:
        self.problem = problem
        self.pairs = []
        for o in problem.obs:
            if o["flange"] is None or not o["used"] or (o["sample"], o["cam"]) not in clouds:
                continue
            tgt = np.asarray(clouds[(o["sample"], o["cam"])], np.float64)
            if len(tgt) < 50:
                continue
            nrm, flat = normals(tgt)
            for static in problem.static:
                src = clouds.get((o["sample"], static))
                if src is not None and len(src) >= 50:
                    self.pairs.append({"wrist": o, "static": static, "src": np.asarray(src, np.float64),
                                       "tgt": tgt, "n": nrm, "flat": flat})
        self.cams = sorted({p["wrist"]["cam"] for p in self.pairs} | {p["static"] for p in self.pairs})

    def split(self, v: np.ndarray, n_pose: int) -> tuple[dict, dict]:
        return self.problem.unpack(v[:n_pose]), dict(zip(self.cams, np.exp(v[n_pose:])))

    def _world(self, p: dict, x: dict, scale: dict) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        o = p["wrist"]
        Tw = self.problem.wrist_pose(o["cam"], o["flange"], x)
        a = _transform(x[f"M:{p['static']}"], scale[p["static"]] * p["src"])
        b = _transform(Tw, scale[o["cam"]] * p["tgt"])
        return a, b, p["n"] @ Tw[:3, :3].T

    def match(self, x: dict, scale: dict, max_dist: float) -> list[tuple[np.ndarray, np.ndarray]]:
        """Per pair: (static point, wrist point) indices within ``max_dist``, on a flat wrist surface."""
        out = []
        for p in self.pairs:
            a, b, _ = self._world(p, x, scale)
            d, j = cKDTree(b).query(a, distance_upper_bound=max_dist)
            ok = np.isfinite(d)
            ok[ok] &= p["flat"][j[ok]]
            out.append((np.flatnonzero(ok), j[ok]))
        return out

    def residuals(self, x: dict, scale: dict, matches: list) -> list[np.ndarray]:
        """Signed point-to-plane distances (m) per pair."""
        out = []
        for p, (i, j) in zip(self.pairs, matches):
            a, b, n = self._world(p, x, scale)
            out.append(np.einsum("ij,ij->i", a[i] - b[j], n[j]))
        return out

    def distances(self, x: dict, scale: dict) -> dict:
        """Per wrist camera: median point-to-plane distance (mm) to the static clouds and the share
        of its static points within STATS_DISTANCE."""
        matches = self.match(x, scale, STATS_DISTANCE)
        res = self.residuals(x, scale, matches)
        out = {}
        for cam in sorted({p["wrist"]["cam"] for p in self.pairs}):
            k = [n for n, p in enumerate(self.pairs) if p["wrist"]["cam"] == cam]
            r = np.concatenate([res[n] for n in k])
            total = sum(len(self.pairs[n]["src"]) for n in k)
            out[cam] = {"mm": round(1000 * float(np.median(np.abs(r))), 2) if len(r) else None,
                        "matched": round(len(r) / max(total, 1), 3)}
        return out


def refine(problem, result: dict, clouds: dict) -> dict:
    """``result`` of the board solve, fine-tuned with the clouds; ``result["cloud"]`` says what
    they changed (or why they were not used)."""
    term = CloudTerm(problem, clouds)
    if not term.pairs:
        return {**result, "cloud": {"used": False, "message": "no static and wrist camera clouds of the same capture"}}
    n_pose = len(result["v"])
    v = np.concatenate([result["v"], np.zeros(len(term.cams))])
    x0, s0 = term.split(v, n_pose)
    before = term.distances(x0, s0)
    n_board = len(problem.residuals(result["v"]))
    res = None
    for max_dist in SCHEDULE:
        matches = term.match(*term.split(v, n_pose), max_dist)
        n = sum(len(i) for i, _ in matches)
        if n < MIN_MATCHES:
            break
        weight = np.sqrt(n_board / n) / SIGMA   # the clouds weigh as much as the board in total

        def fun(w: np.ndarray, matches=matches, weight=weight) -> np.ndarray:
            x, scale = term.split(w, n_pose)
            return np.concatenate([problem.residuals(w[:n_pose])]
                                  + [weight * r for r in term.residuals(x, scale, matches)]
                                  + [w[n_pose:] / SCALE_PRIOR])

        res = least_squares(fun, v, loss="soft_l1", f_scale=1.0, x_scale="jac", max_nfev=60)
        v = res.x
    if res is None:
        return {**result, "cloud": {"used": False, "message": "the static and wrist clouds hardly overlap"}}
    x, scale = term.split(v, n_pose)
    moved, warnings = {}, []
    for b in problem.blocks:
        D = np.linalg.inv(result["x"][b]) @ x[b]
        deg = float(np.degrees(Rotation.from_matrix(D[:3, :3]).magnitude()))
        mm = 1000 * float(np.linalg.norm(x[b][:3, 3] - result["x"][b][:3, 3]))
        moved[b] = {"deg": round(deg, 3), "mm": round(mm, 2)}
        if deg > WARN_DEG or mm > WARN_MM:
            warnings.append(f"{b} would move {mm:.0f} mm / {deg:.1f} deg")
    after = term.distances(x, scale)
    if warnings:
        return {**result, "cloud": {"used": False, "before": before, "after": after, "warnings": warnings,
                                    "message": "not applied, board and clouds disagree: " + "; ".join(warnings)}}
    return {**result, "x": x, "v": v[:n_pose], "std": problem.block_std(res), "std_board": result["std"],
            "cloud": {"used": True, "pairs": len(term.pairs), "before": before, "after": after,
                      "scale": {c: round(float(s), 4) for c, s in scale.items()}, "moved": moved,
                      "warnings": warnings}}
