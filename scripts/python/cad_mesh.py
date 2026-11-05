"""Replace a scanned object mesh with its CAD model, placed where the scan is.

    pixi run python scripts/python/cad_mesh.py <cad.stl|obj|ply> projects/<p>/objects/<name> [--up y]

The scan (``mesh.ply``: metres, object frame at the centre of its bounding box, world axes, posed
by ``T_world_obj`` of ``scan.json``) is the target:

1. units: the CAD extents are compared with the scan's for mm, cm, inch, dm and m (``--scale`` fixes it),
2. orientation: the object stands upright, so the CAD's up axis (``--up``; default: each of the 6
   is tried, but none that reaches BELOW under the scan) goes to world z, its top level with the scan's (``--anchor``: the cameras look from
   above, a scan has the top but often misses the bottom; ``bottom``: the CAD's bottom on the scan's
   lowest points, ``free``: the height fitted too) and only the yaw and x, y are free: every YAW_STEP_DEG, each refined by a short
   ICP of the scan points onto the CAD surface, ranked by their distances capped at CAP (thin parts
   that fix the yaw, like the arms of a stand, count as much as the bulk); the best few get a long
   one. Only scan -> CAD distances count: the scan sees part of the object (not the bottom, not the
   inside), the CAD all of it. ``--fit-scale`` also fits one uniform scale (a CAD that is not quite
   the object; a partial scan pulls it up, so check the result),
3. the CAD, in the scan's frame, is centred on its own bounding box again; ``T_world_obj`` moves
   by that offset, so the tracking starts where the scan put the object.

Keeps the scan as ``mesh_scan.ply`` / ``scan_scan.json`` (once; a second run fits against those).
"""

from __future__ import annotations

import argparse
import json
import shutil
from pathlib import Path

import numpy as np
import trimesh
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

UNITS = {"mm": 1e-3, "cm": 1e-2, "inch": 0.0254, "dm": 0.1, "m": 1.0}
UP = {"x": [0, -np.pi / 2, 0], "-x": [0, np.pi / 2, 0], "y": [np.pi / 2, 0, 0], "-y": [-np.pi / 2, 0, 0],
      "z": [0, 0, 0], "-z": [np.pi, 0, 0]}     # rotation vector taking that CAD axis to z
YAW_STEP_DEG = 10
N_CAD, N_SCAN = 20000, 3000    # surface samples
KEEP = 0.8                     # share of the scan points (the closest) an ICP step fits
CAP = 0.01                     # m, a scan point's distance counts at most this much in the ranking
SHORT, LONG, BEST = 8, 60, 5   # ICP iterations per hypothesis / of the best ones, how many best
INLIER = 0.005                 # m, scan points this close to the CAD count as matched
ANCHOR_PCT = 0.2               # %, scan heights below / above this are ignored for --anchor
BELOW = 0.03                   # m, an up axis (tried) whose bottom ends deeper below the scan's is out
MAX_SCALE = 1.25               # --fit-scale stays within 1/MAX_SCALE .. MAX_SCALE of the unit


def load_mesh(path: Path) -> trimesh.Trimesh:
    m = trimesh.load(str(path), force="mesh")
    if not len(m.faces):
        raise SystemExit(f"{path}: no triangles")
    return m


def guess_scale(cad: trimesh.Trimesh, scan_extent: np.ndarray) -> tuple[float, str]:
    """The unit whose sorted extents come closest to the scan's (log ratio of the largest two)."""
    e = np.sort(cad.extents)[::-1][:2]
    s = np.sort(scan_extent)[::-1][:2]
    err = {u: float(np.abs(np.log(f * e / s)).mean()) for u, f in UNITS.items()}
    unit = min(err, key=err.get)
    return UNITS[unit], unit


def rot_z(yaw: float) -> np.ndarray:
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def fit(yaw, t, P, tree, Q, iters: int, fit_scale: bool, anchor: tuple[float, float] | None):
    """ICP about z: p ~ s Rz(yaw) q + t for the scan points P onto the (upright) CAD samples Q,
    each step on the KEEP closest; ``anchor`` (scan height, CAD height) are kept level, unless
    None. -> yaw, t, s, ranking score (mean distance capped at CAP)."""
    s = 1.0
    if anchor is not None:
        t = np.r_[t[:2], anchor[0] - anchor[1]]
    for _ in range(iters):
        R = rot_z(yaw)
        d, i = tree.query((P - t) @ R / s)          # scan point in the CAD frame -> nearest sample
        keep = d <= np.quantile(d, KEEP)
        A, B = Q[i[keep]], P[keep]                  # B ~ s Rz A + t
        a, b = A - A.mean(0), B - B.mean(0)
        H = b[:, :2].T @ a[:, :2]
        yaw = np.arctan2(H[1, 0] - H[0, 1], H[0, 0] + H[1, 1])
        R = rot_z(yaw)
        if fit_scale:
            s = float(np.clip(np.einsum("ij,ij", b, a @ R.T) / (a ** 2).sum(), 1 / MAX_SCALE, MAX_SCALE))
        t = B.mean(0) - s * R @ A.mean(0)
        if anchor is not None:
            t[2] = anchor[0] - s * anchor[1]
    d, _ = tree.query((P - t) @ rot_z(yaw) / s)
    return yaw, t, s, float(np.minimum(d, CAP).mean())


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("cad", type=Path, help="CAD export (STL, OBJ, PLY)")
    ap.add_argument("object_dir", type=Path, help="projects/<p>/objects/<name> with mesh.ply + scan.json")
    ap.add_argument("--up", choices=UP, help="the CAD axis pointing up (default: each is tried)")
    ap.add_argument("--scale", type=float, help="CAD units -> m (default: guessed from the scan)")
    ap.add_argument("--fit-scale", action="store_true", help="also fit a uniform scale to the scan")
    ap.add_argument("--anchor", choices=["top", "bottom", "free"], default="top",
                    help="which end of the CAD is level with the scan's (default: top)")
    ap.add_argument("--dry-run", action="store_true", help="report the fit, write nothing")
    args = ap.parse_args()

    d = args.object_dir
    scan_mesh, scan_json = d / "mesh_scan.ply", d / "scan_scan.json"
    if not scan_mesh.exists():
        scan_mesh, scan_json = d / "mesh.ply", d / "scan.json"
    scan = load_mesh(scan_mesh)
    info = json.loads(scan_json.read_text())

    cad = load_mesh(args.cad)
    if args.scale:
        scale, unit = args.scale, f"x{args.scale}"
    else:
        scale, unit = guess_scale(cad, scan.extents)
    cad.apply_scale(scale)
    print(f"CAD {args.cad.name}: {len(cad.faces)} faces, unit {unit}, "
          f"{' x '.join(f'{100 * e:.1f}' for e in cad.extents)} cm "
          f"(scan {' x '.join(f'{100 * e:.1f}' for e in scan.extents)} cm)")

    rng = np.random.default_rng(0)
    Q0 = trimesh.sample.sample_surface_even(cad, N_CAD, seed=0)[0]
    P = trimesh.sample.sample_surface_even(scan, N_SCAN, seed=0)[0]
    sub = P[rng.choice(len(P), min(800, len(P)), replace=False)]

    best = []
    for up in [args.up] if args.up else UP:
        U = Rotation.from_rotvec(UP[up]).as_matrix()
        Q = Q0 @ U.T
        tree = cKDTree(Q)
        anchor = {"top": (np.percentile(P[:, 2], 100 - ANCHOR_PCT), Q[:, 2].max()),
                  "bottom": (np.percentile(P[:, 2], ANCHOR_PCT), Q[:, 2].min())}.get(args.anchor)
        if not args.up and args.anchor == "top" and anchor[0] - np.ptp(Q[:, 2]) < P[:, 2].min() - BELOW:
            continue                                 # would reach through the table
        hyps = []
        for yaw in np.deg2rad(np.arange(0, 360, YAW_STEP_DEG)):
            t = P.mean(0) - rot_z(yaw) @ Q.mean(0)
            hyps.append(fit(yaw, t, sub, tree, Q, SHORT, args.fit_scale, anchor))
        hyps.sort(key=lambda h: h[3])
        best += [(*fit(yaw, t, P, tree, Q, LONG, args.fit_scale, anchor), up, U, tree)
                 for yaw, t, s, _ in hyps[:BEST]]
    yaw, t, s, err, up, U, tree = min(best, key=lambda h: h[3])
    R = rot_z(yaw)
    dist, _ = tree.query((P - t) @ R / s)
    print(f"fit: up {up}, yaw {np.rad2deg(yaw):.1f} deg, median {1000 * np.median(dist):.1f} mm, "
          f"{100 * np.mean(dist < INLIER):.0f} % of the scan within {1000 * INLIER:.0f} mm"
          + (f", scale {s:.3f}" if args.fit_scale else ""))

    T = np.eye(4)
    T[:3, :3], T[:3, 3] = s * R @ U, t
    cad.apply_transform(T)                           # into the scan's object frame
    centre = cad.bounds.mean(0)
    cad.apply_translation(-centre)
    colors = getattr(scan.visual, "vertex_colors", None)
    if colors is not None and len(colors):
        cad.visual.vertex_colors = np.tile(np.median(colors, 0).astype(np.uint8), (len(cad.vertices), 1))

    T_world_obj = np.asarray(info["T_world_obj"], float)
    T_world_obj[:3, 3] += T_world_obj[:3, :3] @ centre
    info.update(T_world_obj=np.round(T_world_obj, 6).tolist(), extent_m=np.round(cad.extents, 4).tolist(),
                cad={"file": str(args.cad), "scale": round(scale * s, 6), "up": up, "anchor": args.anchor,
                     "median_mm": round(1000 * float(np.median(dist)), 2),
                     "inliers": round(float(np.mean(dist < INLIER)), 3)})
    if args.dry_run:
        print(f"dry run: centre moves {1000 * np.linalg.norm(centre):.1f} mm, nothing written")
        return
    if not (d / "mesh_scan.ply").exists():
        shutil.copy2(d / "mesh.ply", d / "mesh_scan.ply")
        shutil.copy2(d / "scan.json", d / "scan_scan.json")
    cad.export(str(d / "mesh.ply"))
    (d / "scan.json").write_text(json.dumps(info, indent=2))
    print(f"wrote {d / 'mesh.ply'} and scan.json (scan kept as mesh_scan.ply / scan_scan.json)")


if __name__ == "__main__":
    main()
