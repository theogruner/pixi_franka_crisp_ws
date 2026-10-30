"""Scan of unknown objects (studio's Objects card): meshes and first poses for the object list,
from views the wrist cameras take around them, so the pose tracking can start right away.

1. the Point cloud (Fast-FoundationStereo: RGB-D of every camera) and the Object poses component
   run; the node collects the newest frame of every camera (``/pose_tracking/scan``, its
   live_scan.py),
2. locate: every object is segmented by its prompt in the current views (static cameras and the
   wrist cameras where they are); the world centre of the objects found is the scan's target,
3. per arm (one after the other), from where it stands: ``views`` camera poses on a sphere around
   that centre (azimuths and elevations around the start view, at the start distance), turned into
   grasp-point targets with the camera's TF and driven to; a capture of every camera on each
   still pose, the start pose included, then back to the start,
4. finish: the node segments every capture, fuses one mesh per object (TSDF), writes
   ``objects/<name>/mesh.ply`` into the project, and starts tracking each object at its pose from
   the scan (it has not moved since).

Motions as in the camera calibration (motion.py): slow, limited to ``max_offset`` /
``max_rotation_deg`` around the start pose and ``min_clearance`` above the table; Stop aborts.
"""

from __future__ import annotations

import logging
import threading
import time
import uuid

import numpy as np

from franka_studio.motion import Aborted, ArmMover, angle_deg

log = logging.getLogger(__name__)


def look_at(eye: np.ndarray, target: np.ndarray, x_hint: np.ndarray) -> np.ndarray:
    """Optical frame (z forward) at ``eye`` looking at ``target``, x as close to ``x_hint`` as it gets."""
    z = target - eye
    z /= np.linalg.norm(z)
    x = x_hint - (x_hint @ z) * z
    if np.linalg.norm(x) < 1e-6:
        x = np.cross(z, [0.0, 0.0, 1.0])
    x /= np.linalg.norm(x)
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = np.stack([x, np.cross(z, x), z], axis=1), eye
    return T


def plan_views(C0: np.ndarray, G0: np.ndarray, centre: np.ndarray, cfg: dict) -> tuple[list[np.ndarray], list[str]]:
    """Grasp-point targets for camera poses looking at ``centre`` around the start view ``C0``
    (world <- camera optical frame; ``G0`` world <- grasp point at the same time), nearest first.
    Also the reasons of the views left out."""
    CG = np.linalg.inv(C0) @ G0                        # camera <- grasp point (fixed on the flange)
    u0 = C0[:3, 3] - centre
    d = float(np.clip(np.linalg.norm(u0), cfg["min_distance"], cfg["max_distance"]))
    az0 = np.arctan2(u0[1], u0[0])
    el0 = np.degrees(np.arcsin(np.clip(u0[2] / np.linalg.norm(u0), -1, 1)))
    targets, skipped = [], []
    for del_ in cfg["elevations_deg"]:
        el = np.radians(np.clip(el0 + del_, cfg["min_elevation_deg"], cfg["max_elevation_deg"]))
        for daz in cfg["azimuths_deg"]:
            az = az0 + np.radians(daz)
            u = np.array([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)])
            Tc = look_at(centre + d * u, centre, C0[:3, 0])
            G = Tc @ CG
            off = float(np.linalg.norm(G[:3, 3] - G0[:3, 3]))
            rot = angle_deg(G0[:3, :3].T @ G[:3, :3])
            height = float(G[2, 3] - cfg["support_z"])
            label = f"azimuth {daz:+.0f}, elevation {np.degrees(el):.0f} deg"
            if off < 0.02 and rot < 3.0:
                continue    # the start view: captured there anyway
            if off > cfg["max_offset"] or rot > cfg["max_rotation_deg"] or height < cfg["min_clearance"]:
                skipped.append(f"{label}: offset {100 * off:.0f} cm, rotation {rot:.0f} deg, "
                               f"{100 * height:.0f} cm above the table")
                continue
            targets.append(G)
    ordered, here = [], G0
    while targets and len(ordered) < int(cfg["views"]):
        i = int(np.argmin([np.linalg.norm(t[:3, 3] - here[:3, 3]) + 0.003 * angle_deg(here[:3, :3].T @ t[:3, :3])
                           for t in targets]))
        here = targets.pop(i)
        ordered.append(here)
    return ordered, skipped


class ObjectScan(ArmMover):
    def __init__(self, runtime, link, cfg: dict, arms: dict, static: list, notify) -> None:
        """``link``: the PoseTrackingLink; ``arms``: wrist camera -> robot namespace."""
        self.cfg, self.link, self.notify = cfg, link, notify
        self.arms, self.static = dict(arms), list(static)
        self._init_motion(runtime, "studio_object_scan", list(self.arms.values()), cfg)
        self.state = "idle"        # idle | running | done | failed | stopped
        self.phase = ""
        self.progress: list[str] = []
        self.objects: list[dict] = []
        self.results: dict = {}
        self._token = ""
        self._thread: threading.Thread | None = None
        self.prepare = None        # set by the studio: fn() starting the point cloud + Object poses
        self.on_done = None        # fn(results): reload the project's objects (meshes + seeded poses)

    # ---- node protocol --------------------------------------------------------------------------------

    def _node_scan(self) -> dict | None:
        scan = self.link.scan_status()
        return scan if scan and scan.get("token") == self._token else None

    def _wait(self, cond, timeout: float, what: str) -> dict:
        t_end = time.time() + timeout
        while time.time() < t_end:
            self._check()
            scan = self._node_scan()
            if scan and scan.get("state") == "failed":
                raise RuntimeError(f"pose tracking node: {scan.get('message')}")
            if scan and cond(scan):
                return scan
            time.sleep(0.1)
        raise RuntimeError(f"timeout waiting for {what}")

    def _capture(self, k: int, label: str) -> None:
        self._settle()
        self.link.send_scan({"cmd": "capture", "id": k})
        scan = self._wait(lambda s: s.get("capture_id", 0) >= k, 10.0, f"capture {k}")
        self._log(f"{label}: {scan.get('message', '')}")

    # ---- run -----------------------------------------------------------------------------------------------

    def _log(self, text: str) -> None:
        log.info(text)
        self.progress.append(text)
        del self.progress[:-60]

    def start(self, objects: list[dict], out_dir: str) -> dict:
        if self.state == "running":
            return {"ok": False, "message": "A scan is running."}
        if self.halted:
            return {"ok": False, "message": "Halted: press Resume first."}
        if not objects:
            return {"ok": False, "message": "Add the objects to scan first."}
        self._stop.clear()
        self.objects, self.results = list(objects), {}
        self.state, self.phase, self.progress = "running", "starting", []
        self._thread = threading.Thread(target=self._run, args=(out_dir,), daemon=True, name="object-scan")
        self._thread.start()
        return {"ok": True, "message": f"Scanning {', '.join(o['name'] for o in objects)}."}

    def stop(self, reason: str = "stopped") -> bool:
        if self.state != "running":
            return False
        self._stop.set()
        self._log(f"Stop requested ({reason}).")
        return True

    def _run(self, out_dir: str) -> None:
        starts: dict[str, np.ndarray] = {}
        try:
            self.phase = "starting the point cloud and the pose tracking"
            if self.prepare:
                self.prepare()
            t_end = time.time() + float(self.cfg["start_timeout_s"])
            while self.link.scan_status() is None:
                self._check()
                if time.time() > t_end:
                    raise RuntimeError("the pose tracking node does not report (Object poses component?)")
                time.sleep(0.5)
            self._token = uuid.uuid4().hex[:8]
            cameras = self.static + list(self.arms)
            self.link.send_scan({"cmd": "start", "objects": self.objects, "cameras": cameras,
                                 "out_dir": out_dir, "token": self._token})
            self._wait(lambda s: s.get("state") == "collecting", 10.0, "the node to start the scan")
            time.sleep(1.5)    # the first RGB-D of every camera (FFS computes it once subscribed)

            self.phase = "locating the objects"
            self.link.send_scan({"cmd": "locate"})
            scan = self._wait(lambda s: s.get("state") == "collecting" and s.get("message", "").startswith("located"),
                              float(self.cfg["locate_timeout_s"]), "the objects to be located")
            self._log(scan["message"])
            located = scan.get("located") or {}
            if not located:
                raise RuntimeError("none of the objects found: check the prompts and that a camera sees them")
            centre = np.mean([np.asarray(p, float) for p in located.values()], axis=0)
            self._log(f"centre {np.round(centre, 3).tolist()}")

            k = 1
            self._capture(k, "start views")
            for cam, robot in self.arms.items():
                self.phase = f"{cam}: planning"
                G0 = self._grasp(robot)
                C0 = self._lookup("world", f"{robot}_zed_left_camera_optical_frame")
                if C0 is None:
                    self._log(f"{cam}: no TF of its camera, skipped")
                    continue
                targets, skipped = plan_views(C0, G0, centre, self.cfg)
                self._log(f"{cam}: {len(targets)} views planned" + (f", {len(skipped)} out of reach" if skipped else ""))
                if not targets:
                    continue
                starts[robot] = G0
                for i, T in enumerate(targets, 1):
                    self.phase = f"{cam}: view {i}/{len(targets)}"
                    if not self._move(robot, T):
                        self._log(f"  view {i}: target not reached, capturing anyway")
                    k += 1
                    self._capture(k, f"{cam} view {i}")
                self.phase = f"{cam}: back to the start"
                self._move(robot, G0)
                starts.pop(robot, None)

            self.phase = "segmenting and fusing (takes a while)"
            self.link.send_scan({"cmd": "finish"})
            scan = self._wait(lambda s: s.get("state") in ("done", "failed"), float(self.cfg["fuse_timeout_s"]),
                              "the meshes")
            self.results = scan.get("results") or {}
            for name, r in self.results.items():
                self._log(f"{name}: " + (r["error"] if "error" in r else
                                         f"{r['views']} views, {' x '.join(f'{100 * e:.1f}' for e in r['extent_m'])} cm"))
            if self.on_done:
                self.on_done(self.results)
            ok = sum("mesh" in r for r in self.results.values())
            self.state, self.phase = ("done" if ok else "failed"), "done"
            self._log(f"Done: {ok}/{len(self.results)} objects scanned; tracking starts at their scan poses.")
        except Aborted:
            self.state, self.phase = "stopped", "stopped"
            self.link.send_scan({"cmd": "cancel"})
            self._log("Stopped. The arms hold where they are.")
        except Exception as e:
            log.exception("object scan failed")
            self.state, self.phase = "failed", "failed"
            self.link.send_scan({"cmd": "cancel"})
            self._log(f"Failed: {e}")
            for robot, G0 in starts.items():    # bring a scanning arm back where it started
                try:
                    if not self.halted:
                        self._move(robot, G0)
                except Exception:
                    pass
        finally:
            self.notify(f"Object scan {self.state}.", "success" if self.state == "done" else "warn")

    # ---- status ---------------------------------------------------------------------------------------------

    def snapshot(self) -> dict:
        node = self._node_scan() if self.state == "running" else None
        return {"state": self.state, "phase": self.phase, "progress": self.progress[-14:],
                "node": node, "results": self.results, "halted": self.halted}
