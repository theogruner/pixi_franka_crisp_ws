"""3D view of the active SceneEnv, served by viser and embedded in the studio page.

What is drawn comes from the env alone, so the view shows exactly what the policy (or the
recording) gets:

* the observation of the latest ``StepRecord``: every image feature as a panel image and, when
  ``scene.cameras`` gives its pose, as a camera frustum in the scene; the state vectors as text;
* ``env.render()``: robot models (URDF, posed by the joint states), URDF links as frames, poses
  from topics (optionally with a mesh) and point clouds;
* point cloud features of the observation (``(N, 3)`` / ``(N, 6)`` arrays, e.g. the sampled
  ``observation.pointcloud``) as their own cloud, so the policy's input can be checked against
  the full merged cloud.

Two modes (``set_mode``, the page's Scene / Scene graph switch): ``scene`` draws all of the above;
``graph`` only the detected objects (scene_graph.py): their points in their colours, boxes, labels,
lines between related objects and, where the pose tracking has one, their frame.

Rendering runs in its own thread at ``viewer.rate_hz``; the env's listener only stores the newest
record, so the robot's control loop never waits for the viewer.
"""

from __future__ import annotations

import io
import logging
import threading
import time
from pathlib import Path

import numpy as np

from franka_studio.env import SceneEnv, StepRecord

log = logging.getLogger(__name__)

BRAND = (10, 132, 255)       # system blue, matches the studio accent
UNCALIBRATED = (251, 191, 36)  # amber: camera drawn at a guessed / hand-placed pose
# Where an uncalibrated static camera is drawn until it is placed: in front of the table,
# looking at its centre (world frame: x forward from the robot bases, z up).
IMAGE_PERIOD_S = 0.2   # images (panel + frustums) are re-sent at most this often: they dominate bandwidth
CLOUD_PERIOD_S = 0.2   # same for point clouds (~1 MB per update)
OBS_CLOUD_COLOR = (250, 204, 21)   # observation point clouds without colour
DEFAULT_CAMERA_POSITION = (0.95, 0.0, 0.75)
DEFAULT_CAMERA_TARGET = (0.35, 0.0, 0.05)
POSE_COLORS = [(244, 114, 182), (167, 139, 250), (52, 211, 153), (251, 191, 36)]
RELATION_COLOR = (142, 142, 147)   # lines between related objects (graph mode)


def _mtime(file: str) -> float | None:
    """When a (project) mesh file was last written; None for package:// URLs or a missing file."""
    if "://" in file:
        return None
    try:
        from franka_studio.paths import resolve

        return resolve(file).stat().st_mtime
    except (OSError, ValueError):
        return None


def _package_handler(fname: str) -> str:
    """Resolves ``package://pkg/path`` (and ``file://``) for yourdfpy."""
    if fname.startswith("package://"):
        pkg, _, rel = fname[len("package://"):].partition("/")
        try:
            from ament_index_python.packages import get_package_share_directory

            return str(Path(get_package_share_directory(pkg)) / rel)
        except Exception:
            return fname
    if fname.startswith("file://"):
        return fname[len("file://"):]
    return fname


def look_at_wxyz(position: tuple, target: tuple) -> np.ndarray:
    """Orientation of an optical frame (z forward, y down) at ``position`` looking at ``target``."""
    z = np.asarray(target, float) - np.asarray(position, float)
    z /= np.linalg.norm(z)
    x = np.cross(z, (0.0, 0.0, 1.0))
    x = x / np.linalg.norm(x) if np.linalg.norm(x) > 1e-6 else np.array([0.0, -1.0, 0.0])
    y = np.cross(z, x)
    return _quat_from_matrix(np.column_stack([x, y, z]))


def _quat_from_matrix(m: np.ndarray) -> np.ndarray:
    from scipy.spatial.transform import Rotation

    x, y, z, w = Rotation.from_matrix(m[:3, :3]).as_quat()
    return np.array([w, x, y, z])


class _Robot:
    def __init__(self, server: object, name: str, urdf_xml: str) -> None:
        import yourdfpy
        from viser.extras import ViserUrdf

        self.xml = urdf_xml
        self.urdf = yourdfpy.URDF.load(io.StringIO(urdf_xml), filename_handler=_package_handler,
                                       build_scene_graph=True, load_meshes=True)
        self.viser = ViserUrdf(server, self.urdf, root_node_name=f"/robots/{name}")
        self.joint_names = list(self.viser.get_actuated_joint_names())
        self.cfg = np.zeros(len(self.joint_names))
        self.shown = True

    def update(self, joints: dict) -> None:
        cfg = np.array([joints.get(n, c) for n, c in zip(self.joint_names, self.cfg)])
        if not np.allclose(cfg, self.cfg, atol=1e-5):
            self.cfg = cfg
            self.viser.update_cfg(cfg)

    def link_pose(self, link: str) -> np.ndarray | None:
        if link not in self.urdf.link_map:
            return None
        return self.urdf.get_transform(frame_to=link, frame_from=self.urdf.base_link)

    def remove(self) -> None:
        try:
            self.viser.remove()
        except Exception:
            pass


class SceneViewer:
    """viser server + render thread."""

    def __init__(self, host: str, port: int, rate_hz: float = 15.0) -> None:
        import viser
        # Imported here, before ROS traffic starts: importing them lazily from the render thread
        # holds the import lock for seconds under load, which stalls viser's client handler.
        import trimesh  # noqa: F401
        import yourdfpy  # noqa: F401
        from scipy.spatial.transform import Rotation  # noqa: F401
        from viser.extras import ViserUrdf  # noqa: F401

        self.server = viser.ViserServer(host=host, port=port, label="Franka Studio · Scene", verbose=False)
        self.server.gui.configure_theme(control_width="medium", dark_mode=True,
                                        show_logo=False, show_share_button=False, brand_color=BRAND)
        # Control panel docked right and collapsed to its rail: the scene gets the whole view, one click
        # opens the panel. Both are replayed to every browser that connects later.
        self.server.gui.main_panel.dock_right()
        self.server.gui.main_panel.minimize()
        self.server.scene.set_up_direction("+z")
        cam = self.server.initial_camera
        cam.position, cam.look_at, cam.up = (1.9, -1.6, 1.3), (0.25, 0.0, 0.25), (0.0, 0.0, 1.0)
        self.server.scene.add_grid("/grid", width=3.0, height=3.0, cell_size=0.1, section_size=0.5,
                                   cell_color=(51, 65, 85), section_color=(71, 85, 105), plane="xy",
                                   shadow_opacity=0.2)
        self.server.scene.add_frame("/world", axes_length=0.15, axes_radius=0.005)
        self.rate_hz = rate_hz
        self.port = port
        self._lock = threading.Lock()
        self._env: SceneEnv | None = None
        self._record: StepRecord | None = None
        self._robots: dict[str, _Robot] = {}
        self._robot_errors: dict[str, str] = {}
        self._nodes: dict[str, object] = {}      # scene path -> handle
        self._gui_images: dict[str, object] = {}
        self._meshes: dict[str, object] = {}
        self._fps: list[float] = []
        self._image_sent: dict[str, float] = {}
        gui = self.server.gui
        with gui.add_folder("Display"):
            self.show_robots = gui.add_checkbox("Robots", True)
            self.show_frames = gui.add_checkbox("Frames & poses", True)
            self.show_clouds = gui.add_checkbox("Point clouds", True)
            self.show_cams = gui.add_checkbox("Camera frustums", True)
            self.show_obs_cloud = gui.add_checkbox("Observation point cloud", True,
                                                   hint="The sampled point cloud the policy gets")
            self.point_size = gui.add_slider("Point size", 0.001, 0.02, 0.001, 0.002)
            self.frustum_size = gui.add_slider("Camera image size", 0.02, 0.3, 0.01, 0.06)
        # Static cameras without extrinsics are drawn at a guessed pose; place them here.
        self.on_save_camera = None     # set by the studio: fn(name, position, wxyz) -> str
        self.on_preview_camera = None  # fn(name, position, wxyz): unsaved pose (moves the merged cloud)
        self.on_align_camera = None    # fn(name, position, wxyz) -> {ok, message, position, wxyz}
        self.hidden_poses = None       # fn() -> names of poses not to draw (e.g. objects not tracked)
        self.scene_graph = None        # fn() -> (graph {objects, relations}, cloud (xyz, rgb, stamp) | None)
        self.mode = "scene"            # scene | graph
        self._graph: dict[str, object] = {}       # graph-mode scene nodes
        self._graph_keys: dict[str, object] = {}  # what each graph node was drawn from (redraw on change)
        self._graph_poses: set[str] = set()       # tracked poses of detected objects (drawn in graph mode)
        self._placed: dict[str, tuple] = {}   # name -> (position, wxyz) for unposed cameras
        self._unsaved: set[str] = set()       # placed (gizmo / alignment) but not saved yet
        self._placeable: list[str] = []
        self._gizmo = None
        with gui.add_folder("Camera placement", expand_by_default=False):
            self.place_dd = gui.add_dropdown("Camera", ["—"], initial_value="—")
            self.place_cb = gui.add_checkbox("Move with gizmo", False,
                                             hint="Drag the gizmo until the frustum's image matches the scene")
            self.place_align = gui.add_button(
                "Align to point clouds", icon=viser.Icon.ARROWS_MAXIMIZE,
                hint="ICP of this camera's point cloud against the other cameras' (needs the point cloud "
                     "running); place it roughly first")
            self.place_save = gui.add_button("Save pose to project", icon=viser.Icon.DEVICE_FLOPPY)
            self.place_md = gui.add_markdown("")
        self.place_cb.on_update(lambda _: self._update_gizmo())
        self.place_dd.on_update(lambda _: self._update_gizmo())
        self.place_save.on_click(lambda _: self._save_placement())
        self.place_align.on_click(lambda _: threading.Thread(target=self._align_placement, daemon=True).start())
        self.info_md = gui.add_markdown("*No env yet*")
        self._cameras_folder = gui.add_folder("Cameras")
        self._images_folder = gui.add_folder("Observation images", expand_by_default=False)
        self.state_md = gui.add_markdown("")
        self.show_robots.on_update(lambda _: self._set_robot_visibility())
        self._running = True
        threading.Thread(target=self._loop, daemon=True, name="viewer").start()

    # ---- env binding ----------------------------------------------------------------------------

    def set_env(self, env: SceneEnv | None) -> None:
        with self._lock:
            self._env = env
            self._record = None
        if env is not None and self._on_record not in env._listeners:
            env.add_listener(self._on_record)
        self._robot_errors.clear()
        self._placed.clear()
        self._unsaved.clear()
        self.place_cb.value = False
        self._clear_dynamic()

    def _on_record(self, record: StepRecord) -> None:
        self._record = record

    def latest_record(self) -> StepRecord | None:
        return self._record

    # ---- rendering ---------------------------------------------------------------------------------

    def _clear_dynamic(self) -> None:
        nodes = [self._nodes[k] for k in sorted(self._nodes, key=len, reverse=True)]  # children first
        for handle in nodes + list(self._gui_images.values()):
            try:
                handle.remove()
            except Exception:
                pass
        self._nodes.clear()
        self._gui_images.clear()

    def _robots_visible(self) -> bool:
        return self.show_robots.value and self.mode == "scene"

    def _set_robot_visibility(self) -> None:
        for robot in self._robots.values():
            try:
                robot.viser.show_visual = robot.shown = self._robots_visible()
            except Exception:
                pass

    def _loop(self) -> None:
        period = 1.0 / max(self.rate_hz, 1.0)
        while self._running:
            t0 = time.perf_counter()
            try:
                self._render_once()
            except Exception as e:
                log.warning("viewer render failed: %s", e)
            time.sleep(max(0.0, period - (time.perf_counter() - t0)))

    def _node(self, path: str, factory: object) -> object:
        handle = self._nodes.get(path)
        if handle is None:
            handle = factory()
            self._nodes[path] = handle
        return handle

    def _render_once(self) -> None:
        env, record = self._env, self._record
        self._render_graph()
        if env is None:
            return
        scene = env.render()
        scfg = env.config.get("scene") or {}
        self._render_robots(scene.robots, scfg.get("frames") or [])
        self._render_poses(scene.poses, env)
        self._render_clouds(scene.point_clouds, env)
        self._render_cameras(scene.cameras, scfg.get("cameras") or {})
        if record is not None:
            self._render_record(record, scfg.get("cameras") or {})
            now = time.time()
            self._fps = [t for t in self._fps if now - t < 2.0] + [now]
        robots = ", ".join(f"{n} {'✓' if n in self._robots else '…'}" for n in scene.robots) or "none"
        err = "".join(f"\n\n⚠️ {n}: {e}" for n, e in self._robot_errors.items())
        mode = record.mode if record else "waiting"
        view = f"\n\n**Scene graph view** · {self._graph_count} objects" if self.mode == "graph" else ""
        self.info_md.content = (f"**{env.name}** · {mode} · step {record.step if record else 0}\n\n"
                                f"robots: {robots}{err}{view}")

    def _render_robots(self, robots: dict, frames: list) -> None:
        for name in [n for n in self._robots if n not in robots]:  # not part of this env
            self._robots.pop(name).remove()
        for name, r in robots.items():
            robot = self._robots.get(name)
            if r["urdf"] and (robot is None or robot.xml != r["urdf"]) and name not in self._robot_errors:
                if robot:
                    robot.remove()
                try:
                    self._robots[name] = _Robot(self.server, name, r["urdf"])
                except Exception as e:
                    self._robot_errors[name] = str(e)[:200]
                    log.warning("robot %s: %s", name, e)
                    continue
            if name in self._robots:
                self._robots[name].update(r["joints"])
                if self._robots[name].shown != self._robots_visible():
                    self._set_robot_visibility()
        for link in frames:
            for robot in self._robots.values():
                T = robot.link_pose(link)
                if T is None:
                    continue
                h = self._node(f"/frames/{link}", lambda link=link: self.server.scene.add_frame(
                    f"/frames/{link}", axes_length=0.08, axes_radius=0.004))
                h.position, h.wxyz = T[:3, 3], _quat_from_matrix(T)
                h.visible = self.show_frames.value and self.mode == "scene"
                break

    def _render_poses(self, poses: dict, env: SceneEnv) -> None:
        cfgs = env.scene.pose_cfg
        hidden = self.hidden_poses() if self.hidden_poses else set()
        for i, (name, pose) in enumerate(poses.items()):
            cfg = cfgs.get(name, {})
            color = tuple(cfg.get("color", POSE_COLORS[i % len(POSE_COLORS)]))
            h = self._node(f"/poses/{name}", lambda name=name, color=color, cfg=cfg: self.server.scene.add_frame(
                f"/poses/{name}", axes_length=float(cfg.get("axes_length", 0.06)),
                axes_radius=float(cfg.get("axes_radius", 0.003)),
                origin_radius=0.008, origin_color=color))
            scene_mode = self.mode == "scene"
            shown = (self.show_frames.value and name not in hidden) if scene_mode else name in self._graph_poses
            h.position, h.wxyz, h.visible = pose.position, pose.wxyz, shown
            if cfg.get("label", True):
                label = self._node(f"/poses/{name}/label", lambda name=name: self.server.scene.add_label(
                    f"/poses/{name}/label", name, font_screen_scale=0.7))
                label.visible = scene_mode   # graph mode: the detection's label
            if cfg.get("mesh"):
                path = f"/poses/{name}/mesh"
                key = (cfg["mesh"], _mtime(cfg["mesh"]), cfg.get("mesh_scale", 1.0))
                if self._nodes.get(path + ":key") not in (None, key):   # another file, or rewritten (a scan)
                    try:
                        self._nodes.pop(path).remove()
                    except Exception:
                        self._nodes.pop(path, None)
                self._nodes[path + ":key"] = key
                mesh = self._node(path, lambda path=path, cfg=cfg: self._add_mesh(
                    path, cfg["mesh"], cfg.get("mesh_scale", 1.0)))
                mesh.visible = scene_mode    # graph mode: the detected points, not the model

    def _add_mesh(self, path: str, file: str, scale: float) -> object:
        import trimesh

        from franka_studio.paths import resolve

        mesh = trimesh.load(_package_handler(str(resolve(file)) if "://" not in file else file), force="mesh")
        return self.server.scene.add_mesh_trimesh(path, mesh, scale=scale)

    def _render_clouds(self, clouds: dict, env: SceneEnv) -> None:
        for name, (xyz, rgb, stamp) in clouds.items():
            path = f"/clouds/{name}"
            if path in self._nodes:
                self._nodes[path].visible = self.show_clouds.value and self.mode == "scene"
            if self.mode != "scene" or self._nodes.get(path + ":stamp") == stamp or \
                    not self._image_due(path, CLOUD_PERIOD_S):
                continue  # hidden, unchanged, or sent less than CLOUD_PERIOD_S ago
            self._nodes[path + ":stamp"] = stamp
            cfg = env.scene.cloud_cfg.get(name, {})
            position, wxyz = np.zeros(3), np.array([1.0, 0, 0, 0])
            if cfg.get("attach"):
                robot = self._robots.get(cfg["attach"].get("robot"))
                T = robot.link_pose(cfg["attach"].get("link")) if robot else None
                if T is not None:
                    position, wxyz = T[:3, 3], _quat_from_matrix(T)
            elif cfg.get("pose"):
                position = np.asarray(cfg["pose"].get("position", [0, 0, 0]), float)
                wxyz = np.asarray(cfg["pose"].get("wxyz", [1, 0, 0, 0]), float)
            # adding under the same name replaces the previous cloud
            self._nodes[path] = self.server.scene.add_point_cloud(
                path, xyz, rgb, point_size=self.point_size.value, point_shape="circle",
                position=position, wxyz=wxyz, visible=self.show_clouds.value)

    def _camera_pose(self, cam: dict) -> tuple | None:
        """World pose of a camera: a URDF link (optical frame, z forward) or a fixed pose."""
        if cam.get("link"):
            robots = ([self._robots[cam["robot"]]] if cam.get("robot") in self._robots
                      else [] if cam.get("robot") else list(self._robots.values()))
            for robot in robots:
                T = robot.link_pose(cam["link"])
                if T is None and cam.get("robot") and cam["link"].startswith(cam["robot"] + "_"):
                    # robot description from before the camera frames got the arm prefix
                    T = robot.link_pose(cam["link"][len(cam["robot"]) + 1:])
                if T is not None:
                    return T[:3, 3], _quat_from_matrix(T)
            return None
        if "position" not in cam:
            return None
        return np.asarray(cam["position"], float), np.asarray(cam.get("wxyz", (1, 0, 0, 0)), float)

    def _frustum(self, name: str, img: np.ndarray, cam: dict, fov: float | None, pose: tuple | None = None,
                 color: tuple = BRAND, label: str | None = None) -> None:
        pose = pose or self._camera_pose(cam)
        if pose is None:
            return
        h, w = img.shape[:2]
        fov = np.deg2rad(float(cam["fov_deg"])) if cam.get("fov_deg") else (fov or np.deg2rad(70))
        path = f"/cameras/{name}"
        shape = (round(float(fov), 3), round(w / h, 3), float(cam.get("scale", self.frustum_size.value)), color)
        handle = self._nodes.get(path)
        if handle is None or self._nodes.get(path + ":shape") != shape:
            # adding under the same name replaces the previous frustum
            self._nodes[path] = self.server.scene.add_camera_frustum(
                path, fov=shape[0], aspect=shape[1], scale=shape[2], image=img, format="jpeg", jpeg_quality=70,
                position=tuple(pose[0]), wxyz=tuple(pose[1]), color=color,
                visible=self.show_cams.value and self.mode == "scene")
            self._nodes[path + ":shape"] = shape
            self._image_sent[path] = time.time()
        else:  # cheap per-frame update; the image only every IMAGE_PERIOD_S
            handle.position, handle.wxyz = tuple(pose[0]), tuple(pose[1])
            handle.visible = self.show_cams.value and self.mode == "scene"
            if self._image_due(path):
                handle.image = img
        text = label or name
        key = f"{path}/label"
        if self._nodes.get(key + ":text") != text:  # re-add only when the text changes
            self._nodes[key] = self.server.scene.add_label(key, text, font_screen_scale=0.7)
            self._nodes[key + ":text"] = text

    def _image_due(self, key: str, period: float = IMAGE_PERIOD_S) -> bool:
        now = time.time()
        if now - self._image_sent.get(key, 0.0) < period:
            return False
        self._image_sent[key] = now
        return True

    # ---- placing cameras without extrinsics -----------------------------------------------------

    def _placed_pose(self, name: str, cam: dict) -> tuple:
        # A saved pose wins over the cached one unless the camera is being moved (unsaved): it can
        # also change without the gizmo (board_calibration.py --write saves through the studio).
        if "position" in cam and name not in self._unsaved:
            self._placed[name] = (np.asarray(cam["position"], float),
                                  np.asarray(cam.get("wxyz", (1, 0, 0, 0)), float))
        elif name not in self._placed:   # no saved pose: a guess beside the scene to start from
            offset = np.array([0.0, 0.25 * len(self._placed), 0.0])
            pos = np.asarray(DEFAULT_CAMERA_POSITION) + offset
            self._placed[name] = (pos, look_at_wxyz(pos, np.asarray(DEFAULT_CAMERA_TARGET) + offset))
        return self._placed[name]

    def _update_gizmo(self) -> None:
        name = self.place_dd.value
        if self._gizmo is not None:
            self._gizmo.remove()
            self._gizmo = None
        if not self.place_cb.value or name not in self._placeable:
            return
        pos, wxyz = self._placed_pose(name, {})
        self._gizmo = self.server.scene.add_transform_controls(
            "/placement/gizmo", scale=0.25, position=tuple(pos), wxyz=tuple(wxyz), depth_test=False)

        @self._gizmo.on_update
        def _(_event: object, name: str = name) -> None:
            self._placed[name] = (np.asarray(self._gizmo.position), np.asarray(self._gizmo.wxyz))
            self._unsaved.add(name)
            if self.on_preview_camera is not None:
                self.on_preview_camera(name, list(self._gizmo.position), list(self._gizmo.wxyz))

    def _save_placement(self) -> None:
        name = self.place_dd.value
        if name not in self._placed or self.on_save_camera is None:
            self.place_md.content = "Choose a camera first."
            return
        pos, wxyz = self._placed[name]
        self.place_md.content = self.on_save_camera(name, [round(float(v), 4) for v in pos],
                                                    [round(float(v), 5) for v in wxyz])
        self._unsaved.discard(name)

    def _align_placement(self) -> None:
        name = self.place_dd.value
        if name not in self._placeable or self.on_align_camera is None:
            self.place_md.content = "Choose a camera first."
            return
        pos, wxyz = self._placed_pose(name, {})
        self.place_align.disabled = True
        self.place_md.content = f"Aligning **{name}** …"
        try:
            result = self.on_align_camera(name, [float(v) for v in pos], [float(v) for v in wxyz])
        finally:
            self.place_align.disabled = False
        if result.get("ok"):
            self._placed[name] = (np.asarray(result["position"], float), np.asarray(result["wxyz"], float))
            self._unsaved.add(name)
            if self._gizmo is not None and self.place_dd.value == name:
                self._gizmo.position, self._gizmo.wxyz = tuple(result["position"]), tuple(result["wxyz"])
            self.place_md.content = (f"✓ {result['message']}\n\nCheck the merged cloud, run it again to refine, "
                                     "then **Save pose to project**.")
        else:
            self.place_md.content = f"⚠️ {result.get('message', 'failed')}"

    def _sync_placeable(self, names: list[str]) -> None:
        if names == self._placeable:
            return
        self._placeable = names
        self.place_dd.options = names or ["—"]
        if self.place_dd.value not in names:
            self.place_dd.value = names[0] if names else "—"
        self.place_md.content = ("Amber frustums are guesses or unsaved: move them with the gizmo (the "
                                 "merged point cloud follows), *Align to point clouds*, then save."
                                 if names else "Every camera has a pose.")

    def _render_cameras(self, cameras: dict, cfg: dict) -> None:
        """Every camera the scene receives (rig snapshot / image topics): panel image + frustum."""
        for name, (img, fov, _stamp) in sorted(cameras.items()):
            key = f"camera:{name}"
            handle = self._gui_images.get(key)
            if handle is None:
                with self._cameras_folder:
                    handle = self.server.gui.add_image(img, label=name, format="jpeg", jpeg_quality=75)
                self._gui_images[key] = handle
            elif self._image_due(key):
                handle.image = img
        placeable = []
        for name, (img, fov, _stamp) in sorted(cameras.items()):
            cam = cfg.get(name, {})
            if cam.get("link"):
                self._frustum(name, img, cam, fov)
                continue
            # a fixed pose from the config, or a guessed / hand-placed one
            placeable.append(name)
            pose = self._placed_pose(name, cam)
            calibrated = "position" in cam and name not in self._dirty_placements()
            self._frustum(name, img, cam, fov, pose=pose, color=BRAND if calibrated else UNCALIBRATED,
                          label=name if "position" in cam else f"{name} (not calibrated)")
        self._sync_placeable(placeable)

    def _dirty_placements(self) -> set:
        """Cameras moved with the gizmo / aligned since their pose was last saved."""
        return set(self._unsaved)

    def _render_record(self, record: StepRecord, cameras: dict) -> None:
        obs = record.observation or {}
        lines = []
        for key, value in obs.items():
            if not isinstance(value, np.ndarray):
                if key == "task" and value:
                    lines.append(f"**prompt** · {value}")
                continue
            if value.ndim == 3:
                img = np.ascontiguousarray(value.transpose(1, 2, 0) if value.shape[0] in (1, 3, 4) else value)
                if img.shape[-1] == 1:
                    img = np.repeat(img, 3, axis=-1)
                img = img[..., :3]
                if img.dtype != np.uint8:
                    img = np.clip(img * (255 if img.max() <= 1.0 else 1), 0, 255).astype(np.uint8)
                handle = self._gui_images.get(key)
                if handle is None:
                    with self._images_folder:
                        handle = self.server.gui.add_image(img, label=key.split(".")[-1], format="jpeg",
                                                           jpeg_quality=80)
                    self._gui_images[key] = handle
                elif self._image_due(key):
                    handle.image = img
                if key in cameras:
                    self._frustum(key.split(".")[-1], img, cameras[key], None)
            elif value.ndim == 2 and value.shape[1] in (3, 6) and value.shape[0] > 8:
                self._render_obs_cloud(key, value)
                lines.append(f"**{key.split('.')[-1]}** `{value.shape[0]} × {value.shape[1]}` points")
            else:
                vec = np.asarray(value).ravel()
                lines.append(f"**{key.split('.')[-1]}** `{np.array2string(vec, precision=3, max_line_width=60)}`")
        if record.action is not None:
            act = np.array2string(np.asarray(record.action).ravel(), precision=3, max_line_width=60)
            lines.append(f"**action** `{act}`")
        if record.info.get("missing"):
            lines.append("⏳ waiting for: " + ", ".join(record.info["missing"][:4]))
        self.state_md.content = "\n\n".join(lines)

    def _render_obs_cloud(self, key: str, value: np.ndarray) -> None:
        path = f"/observation/{key}"
        if path in self._nodes:
            self._nodes[path].visible = self.show_obs_cloud.value and self.mode == "scene"
        if self.mode != "scene" or not self._image_due(path, CLOUD_PERIOD_S):
            return
        xyz = np.ascontiguousarray(value[:, :3], dtype=np.float32)
        if value.shape[1] == 6:
            rgb = np.clip(value[:, 3:] * (255 if value[:, 3:].max() <= 1.0 else 1), 0, 255).astype(np.uint8)
        else:
            rgb = np.tile(np.array(OBS_CLOUD_COLOR, np.uint8), (len(xyz), 1))
        self._nodes[path] = self.server.scene.add_point_cloud(
            path, xyz, rgb, point_size=self.point_size.value * 1.6, point_shape="circle",
            visible=self.show_obs_cloud.value)

    # ---- scene graph view --------------------------------------------------------------------------

    def set_mode(self, mode: str) -> None:
        """``scene``: everything; ``graph``: only the detected objects (the rest is hidden, not removed)."""
        self.mode = mode if mode in ("scene", "graph") else "scene"
        self._set_robot_visibility()
        for path, handle in list(self._nodes.items()):
            if ":" in path or not hasattr(handle, "visible"):
                continue
            if path.startswith(("/clouds/", "/cameras/", "/observation/", "/frames/")):
                try:
                    handle.visible = self.mode == "scene" and self._shown_in_scene(path)
                except Exception:
                    pass
        if self.mode == "scene":
            self._clear_graph()

    def _shown_in_scene(self, path: str) -> bool:
        if path.startswith("/clouds/"):
            return self.show_clouds.value
        if path.startswith("/cameras/"):
            return self.show_cams.value
        if path.startswith("/observation/"):
            return self.show_obs_cloud.value
        return self.show_frames.value

    @property
    def _graph_count(self) -> int:
        return sum(1 for k in self._graph if k.endswith("/box"))

    def _clear_graph(self, keep: set | None = None) -> None:
        for path in sorted(self._graph, key=len, reverse=True):
            if keep is not None and path in keep:
                continue
            try:
                self._graph.pop(path).remove()
            except Exception:
                pass
            self._graph_keys.pop(path, None)
        if keep is None:
            self._graph_poses = set()

    def _graph_node(self, path: str, key: object, factory) -> None:
        """(Re-)draws ``path`` when what it is drawn from (``key``) changed."""
        if self._graph_keys.get(path) != key or path not in self._graph:
            self._graph[path] = factory()   # adding under the same name replaces it
            self._graph_keys[path] = key

    def _render_graph(self) -> None:
        if self.mode != "graph" or self.scene_graph is None:
            return
        try:
            graph, cloud = self.scene_graph()
        except Exception as e:
            log.debug("scene graph: %s", e)
            return
        objects = graph.get("objects") or []
        self._graph_poses = {o["pose"] for o in objects if o.get("pose")}
        drawn: set[str] = set()
        if cloud is not None and len(cloud[0]):
            xyz, rgb, stamp = cloud
            self._graph_node("/graph/points", (stamp, self.point_size.value), lambda: self.server.scene.add_point_cloud(
                "/graph/points", xyz, rgb, point_size=self.point_size.value * 1.5, point_shape="circle"))
            drawn.add("/graph/points")
        centres = {}
        for o in objects:
            box = o.get("box") or {}
            if "center" not in box:
                continue
            base = f"/graph/objects/{o['id']}"
            centre, size, yaw = np.asarray(box["center"]), np.asarray(box["size"]), float(box.get("yaw", 0.0))
            centres[o["id"]] = centre
            color = tuple(int(c) for c in o.get("color", BRAND))
            wxyz = (np.cos(yaw / 2), 0.0, 0.0, np.sin(yaw / 2))
            key = (tuple(np.round(centre, 3)), tuple(np.round(size, 3)), round(yaw, 2), color)
            self._graph_node(base + "/box", key, lambda: self.server.scene.add_box(
                base + "/box", color=color, dimensions=tuple(size), wireframe=True, position=tuple(centre), wxyz=wxyz,
                cast_shadow=False, receive_shadow=False))
            text = o.get("label") or o.get("prompt") or "?"
            top = (float(centre[0]), float(centre[1]), float(centre[2] + size[2] / 2 + 0.02))
            self._graph_node(base + "/label", (text, tuple(np.round(top, 3))), lambda: self.server.scene.add_label(
                base + "/label", text, position=top, font_screen_scale=0.8, anchor="bottom-center"))
            drawn |= {base + "/box", base + "/label"}
        segments = [(centres[r["a"]], centres[r["b"]]) for r in graph.get("relations") or []
                    if r.get("a") in centres and r.get("b") in centres]
        if segments:
            pts = np.array(segments, dtype=np.float32)
            key = tuple(np.round(pts, 2).ravel())
            self._graph_node("/graph/relations", key, lambda: self.server.scene.add_line_segments(
                "/graph/relations", pts, RELATION_COLOR, thickness=2.0, thickness_units="screen"))
            drawn.add("/graph/relations")
        self._clear_graph(keep=drawn)

    def stop(self) -> None:
        self._running = False
        try:
            self.server.stop()
        except Exception:
            pass
