"""The studio: owns the components, the active project, env, policy, recorder and cluster jobs.

Holds all logic; the web server only calls these methods and renders ``status()``.

Env switching: the project's env (``env.yaml``) is active while no policy is loaded, so the viewer
shows what is being recorded. Loading a policy builds a new env from the policy's config and makes
it active, so the viewer shows exactly what the policy is fed (its crops, resizes, state layout).
While nothing steps the env, a monitor thread ticks it so the view stays live.

Observation space: the project env only carries the modalities switched on (``modalities`` in
env.yaml, see modalities.py); switching one rebuilds it, starts/stops the component producing it
(e.g. the point cloud on the Jetson) and changes what the data collector records next to its
fixed topics.
"""

from __future__ import annotations

import collections
import logging
import threading
import time
from pathlib import Path

from franka_studio.cluster.cluster import Cluster
from franka_studio.collection.collector import DataCollection, list_datasets, valid_name
from franka_studio import modalities as mods
from franka_studio.env import SceneEnv
from franka_studio.paths import add_import_paths, resolve
from franka_studio.pointcloud import PointCloudLink
from franka_studio.policy import remote
from franka_studio.policy.runner import ERROR, LOADING, PolicyRunner
from franka_studio.processes import ProcessManager
from franka_studio.projects import Project, list_projects, load_project, save_camera_pose
from franka_studio.ros import registry
from franka_studio.state import UiState

log = logging.getLogger(__name__)


def _start_ros2_daemon() -> None:
    """Starts the ros2 CLI daemon in a session of its own. Otherwise the first ``ros2`` command of a
    component spawns it inside that component's process group, where it outlives the component
    and keeps it looking alive after it has exited."""
    import subprocess

    try:
        subprocess.Popen(["ros2", "daemon", "start"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                         stdin=subprocess.DEVNULL, start_new_session=True)
    except OSError:
        pass


class Studio:
    """Application state and actions."""

    def __init__(self, cfg: dict, with_viewer: bool = True) -> None:
        self.cfg = cfg
        self.state = UiState()
        add_import_paths(cfg["client_paths"])
        registry.load(cfg.get("plugin_modules"))
        wire = cfg["wire"]
        remote.configure_wire(wire["observation_module"], wire["policy_config_module"])

        from franka_studio.ros.runtime import RosRuntime

        self.ros = RosRuntime()
        self.processes = ProcessManager()
        self.processes.set_probe(self._probe)
        self.pointcloud = PointCloudLink(self.ros, cfg.get("pointcloud_topics"))
        self.cluster = Cluster()
        dc = cfg["data_collector"]
        self.collection = DataCollection(lambda: self.ros.call_trigger(dc["trigger_service"]))
        self.viewer = None
        if with_viewer:
            from franka_studio.viz.scene_viewer import SceneViewer

            v = cfg["viewer"]
            self.viewer = SceneViewer(v["host"], int(v["port"]), float(v["rate_hz"]))
            self.viewer.on_save_camera = self.save_camera_pose
            self.viewer.on_preview_camera = self.pointcloud.preview
            self.viewer.on_align_camera = self.align_camera

        self._lock = threading.RLock()
        self._events: collections.deque = collections.deque(maxlen=50)
        self._event_id = 0
        self.project: Project | None = None
        self.modalities: dict[str, mods.Modality] = {}
        self.enabled_modalities: set[str] = set()
        self.project_env: SceneEnv | None = None
        self.runner: PolicyRunner | None = None
        self._loading: str | None = None
        self._datasets_cache: tuple[float, dict] = (0.0, {})
        self._last_record = None
        self._datasets_saved = 0
        self._alive = True

        _start_ros2_daemon()
        projects = self.projects()
        initial = self.state.get("project") or cfg.get("default_project") or (projects[0] if projects else None)
        if initial not in projects:
            initial = projects[0] if projects else None
        self.processes.configure(cfg["components"])
        if initial:
            self.select_project(initial)
        threading.Thread(target=self._monitor_loop, daemon=True, name="monitor").start()
        threading.Thread(target=self._probe_loop, daemon=True, name="probes").start()

    # ---- events -----------------------------------------------------------------------------------

    def notify(self, text: str, level: str = "info") -> None:
        with self._lock:
            self._event_id += 1
            self._events.append({"id": self._event_id, "level": level, "text": text, "t": time.time()})
        getattr(log, "warning" if level in ("warn", "error") else "info")(text)

    def _result(self, ok: bool, msg: str) -> dict:
        self.notify(msg, "success" if ok else "error")
        return {"ok": ok, "message": msg}

    # ---- components ------------------------------------------------------------------------------

    def _probe(self, spec: object) -> bool:
        if getattr(spec, "probe_service", None):
            return self.ros.service_exists(spec.probe_service)
        if getattr(spec, "probe_topic", None):
            return self.ros.topic_has_publishers(spec.probe_topic)
        return False

    def _probe_loop(self) -> None:
        while self._alive:
            self.processes.refresh_probes()
            time.sleep(2.0)

    def start_component(self, name: str) -> dict:
        return self._result(*self.processes.start(name))

    def stop_component(self, name: str) -> dict:
        if name == self.cfg["data_collector"]["component"] and self.collection.recording:
            self.collection.stop()
        return self._result(*self.processes.stop(name))

    def component_logs(self, name: str, since: int = 0) -> list:
        proc = self.processes.get(name)
        return proc.lines_since(since) if proc else []

    # ---- projects ------------------------------------------------------------------------------------

    @property
    def projects_dir(self) -> Path:
        return resolve(self.cfg["projects_dir"])

    def projects(self) -> list[str]:
        return list_projects(self.projects_dir)

    def select_project(self, project_id: str) -> dict:
        if self.runner and self.runner.running:
            return self._result(False, "Stop the policy before switching projects.")
        if self.collection.recording:
            return self._result(False, "Stop the recording before switching projects.")
        root = self.projects_dir / project_id
        if not (root / "project.yaml").is_file():
            return self._result(False, f"No project {project_id!r} in {self.projects_dir}.")
        project = load_project(root)
        with self._lock:
            self._unload_policy()
            if self.project_env:
                self.project_env.close()
                self.project_env = None
            self.project = project
            self.state.set("project", project_id)
            self.processes.configure({**self.cfg["components"], **(project.cfg.get("components") or {})})
            self.modalities = mods.load_modalities(project.env_config)
            self.enabled_modalities = mods.resolve_enabled(
                self.modalities, self.state.project(project_id).get("modalities"))
            self.project_env = self._make_project_env()
            self._set_view(self.project_env)
            self.pointcloud.set_cameras((project.env_config.get("scene") or {}).get("cameras") or {})
        if project.error:
            self.notify(f"{project.name}: {project.error}", "warn")
        return self._result(True, f"Project {project.name} — {len(project.policies)} policies.")

    def _make_project_env(self) -> SceneEnv | None:
        project = self.project
        if not project or not project.env_config:
            return None
        project.env_config.setdefault("_language", False)
        config = mods.apply(project.env_config, self.modalities, self.enabled_modalities)
        return self._make_env(config, f"{project.name} · scene")

    def _make_env(self, config: dict, name: str) -> SceneEnv:
        env = SceneEnv(self.ros, config, name=name)
        env.add_listener(lambda record, env=env: self._on_record(env, record))
        return env

    def _on_record(self, env: SceneEnv, record: object) -> None:
        if env is self.active_env:
            self._last_record = record

    def _set_view(self, env: SceneEnv | None) -> None:
        self._last_record = None
        if self.viewer:
            self.viewer.set_env(env)

    @property
    def active_env(self) -> SceneEnv | None:
        runner = self.runner
        if runner and runner.state not in (LOADING, ERROR):
            return runner.env
        return self.project_env

    def save_camera_pose(self, name: str, position: list, wxyz: list) -> str:
        """Stores a camera pose placed in the viewer for the project (and the loaded envs)."""
        if not self.project:
            return "No project selected."
        save_camera_pose(self.project, name, position, wxyz)
        for env in {self.project_env, self.runner.env if self.runner else None} - {None}:
            cams = env.config.setdefault("scene", {}).setdefault("cameras", {})
            if not (cams.get(name) or {}).get("link"):
                cams[name] = {**(cams.get(name) or {}), "position": position, "wxyz": wxyz}
        self.pointcloud.set_cameras((self.project.env_config.get("scene") or {}).get("cameras") or {})
        msg = f"Saved the pose of {name} to {self.project.id}/camera_poses.yaml."
        self.notify(msg, "success")
        return msg

    def align_camera(self, name: str, position: list, wxyz: list) -> dict:
        """ICP of ``name``'s point cloud against the other cameras (on the Jetson), from a rough pose."""
        result = self.pointcloud.align(name, position, wxyz)
        if result.get("ok"):
            self.pointcloud.preview(name, result["position"], result["wxyz"])  # merged cloud shows it
            self.notify(f"Aligned {name}: {result['message']}. Save the pose to keep it.", "success")
        else:
            self.notify(f"Aligning {name} failed: {result.get('message')}", "warn")
        return result

    # ---- observation space ---------------------------------------------------------------------------

    def set_modality(self, modality: str, on: object) -> dict:
        on = on in (True, "true", "1", 1, "on")
        m = self.modalities.get(modality)
        if m is None:
            return self._result(False, f"Unknown modality {modality!r}.")
        if m.required and not on:
            return self._result(False, f"{m.label} is always part of the observation.")
        if self.collection.recording:
            return self._result(False, "Stop the recording before changing the observation space.")
        if (modality in self.enabled_modalities) == on:
            return {"ok": True, "message": "unchanged"}
        with self._lock:
            self.enabled_modalities = (self.enabled_modalities | {modality}) if on else (self.enabled_modalities - {modality})
            self.state.set_project(self.project.id, modalities=sorted(self.enabled_modalities))
            if self.project_env is not None:  # suspended while a policy is loaded: rebuilt on unload
                self.project_env.close()
                self.project_env = self._make_project_env()
                if self.active_env is self.project_env:
                    self._set_view(self.project_env)
        note = ""
        if m.component and self.processes.get(m.component):
            if on and not self.processes.is_up(m.component):
                ok, msg = self.processes.start(m.component)
                note = f" Starting {m.component}." if ok else f" ({msg})"
            elif not on and self.processes.get(m.component).snapshot().get("owned"):
                self.processes.stop(m.component)
                note = f" Stopping {m.component}."
        return self._result(True, f"{m.label} {'on' if on else 'off'}.{note}")

    def _modality_status(self) -> list[dict]:
        comps = {c["name"]: c for c in self.processes.snapshot()}
        out = []
        for mid, m in self.modalities.items():
            comp = comps.get(m.component) if m.component else None
            up = bool(comp and (comp.get("detected") or (comp.get("owned") and comp.get("state") == "running")))
            out.append({**m.to_dict(), "enabled": mid in self.enabled_modalities,
                        "component_up": up if m.component else None})
        return out

    # ---- policies ------------------------------------------------------------------------------------

    def refresh_policies(self) -> dict:
        if not self.project:
            return self._result(False, "No project selected.")
        self.project.refresh_policies()
        broken = sum(1 for p in self.project.policies.values() if p.error)
        note = f" ({broken} with problems)" if broken else ""
        return self._result(True, f"Found {len(self.project.policies)} policies{note}.")

    def load_policy(self, policy_id: str) -> dict:
        if not self.project or policy_id not in self.project.policies:
            return self._result(False, "Unknown policy; refresh the list.")
        spec = self.project.policies[policy_id]
        if spec.error:
            return self._result(False, f"{spec.name}: {spec.error}")
        if self.runner and self.runner.running:
            return self._result(False, "Stop the running policy first.")
        if self.collection.recording:
            return self._result(False, "Stop the recording first.")
        if self._loading:
            return self._result(False, "A policy is already loading.")
        self._loading = policy_id
        threading.Thread(target=self._load_worker, args=(spec,), daemon=True).start()
        self.state.set_project(self.project.id, policy=policy_id)
        return {"ok": True, "message": f"Loading {spec.name}…"}

    def _load_worker(self, spec: object) -> None:
        try:
            with self._lock:
                self._unload_policy()
            env = self._make_env(spec.config, spec.name)
            runner = PolicyRunner(spec, env)
            with self._lock:
                self.runner = runner
            runner.load()
            if runner.state == ERROR:
                self.notify(runner.error, "error")
                with self._lock:
                    if self.runner is runner:
                        self._unload_policy()
                return
            for w in env.warnings:
                self.notify(f"{spec.name}: {w}", "warn")
            self._set_view(env)
            self._suspend_project_env()
            self.notify(f"{spec.name} loaded.", "success")
        except Exception as e:
            log.exception("loading policy failed")
            self.notify(f"Loading {spec.name} failed: {e}", "error")
            with self._lock:
                self._unload_policy()
        finally:
            self._loading = None

    def _suspend_project_env(self) -> None:
        """While a policy env is active the project env only costs CPU (same camera topics)."""
        with self._lock:
            if self.project_env is not None:
                self.project_env.close()
                self.project_env = None

    def _resume_project_env(self) -> None:
        if self.project_env is None:
            self.project_env = self._make_project_env()

    def _unload_policy(self) -> None:
        runner, self.runner = self.runner, None
        if runner:
            runner.unload()
            runner.env.close()
            self._resume_project_env()
            self._set_view(self.project_env)

    def unload_policy(self) -> dict:
        if not self.runner:
            return self._result(False, "No policy loaded.")
        name = self.runner.spec.name
        with self._lock:
            self._unload_policy()
        return self._result(True, f"Unloaded {name}.")

    def run_policy(self, prompt: str) -> dict:
        if not self.runner:
            return self._result(False, "Load a policy first.")
        if self.collection.recording:
            return self._result(False, "Stop the recording before running the policy.")
        if self.active_env is not None:
            self.active_env.stop_home()   # both would drive the joints
        ok, msg = self.runner.run(prompt)
        if ok and self.project:
            self.state.set_project(self.project.id, prompt=prompt)
        return self._result(ok, msg)

    def stop_policy(self) -> dict:
        env = self.active_env
        homing = env is not None and env.stop_home()   # Stop also aborts a joint homing
        if not self.runner:
            return self._result(True, "Homing stopped.") if homing else self._result(False, "No policy loaded.")
        return self._result(*self.runner.stop())

    def home(self) -> dict:
        if self.runner and self.runner.running:
            self.runner.stop()
        env = self.active_env
        if env is None or not env.home():
            return self._result(False, "No home pose configured (home_pose in the policy/env config).")
        return self._result(True, env.home_message() or "Homing.")

    # ---- data collection -----------------------------------------------------------------------------

    def datasets(self, force: bool = False) -> dict:
        if not self.project:
            return {}
        t, cached = self._datasets_cache
        saved = sum(1 for e in self.collection.history if e.status in ("saved", "discarded"))
        if force or time.time() - t > 3.0 or saved != self._datasets_saved:
            self._datasets_saved = saved
            cached = list_datasets(self.project.data_root)
            self._datasets_cache = (time.time(), cached)
        return cached

    def select_dataset(self, dataset: str, task: str, instruction: str = "") -> dict:
        if self.project:
            self.state.set_project(self.project.id, dataset=dataset, task=task, instruction=instruction)
        return {"ok": True}

    def start_recording(self, dataset: str, task: str, instruction: str = "", operator: str = "") -> dict:
        if not self.project:
            return self._result(False, "No project selected.")
        if self.runner and self.runner.running:
            return self._result(False, "Stop the policy first: data is recorded while you teleoperate.")
        dc = self.cfg["data_collector"]["component"]
        if not self.processes.is_up(dc):
            return self._result(False, "Start the data collector first.")
        dataset, task = dataset.strip(), task.strip()
        if not (valid_name(dataset) and valid_name(task)):
            return self._result(False, "Choose a dataset and a task (letters, digits, - _ . and spaces).")
        meta = {"project": self.project.id, "instruction": instruction or self.project.tasks.get(task, ""),
                "operator": operator, "kind": "demo", "studio_version": 1}
        if self.runner:  # DAgger: a correction recorded around a loaded policy
            s = self.runner.spec
            meta.update(kind="dagger", policy={"name": s.name, "file": str(s.file), "type": s.policy_type,
                                               "checkpoint": s.checkpoint, "prompt": self.runner.prompt})
        topics = mods.record_topics(self.modalities, self.enabled_modalities)
        ok, msg = self.ros.set_parameters(self.cfg["data_collector"]["node"], {"extra_topics": topics})
        if not ok and topics:
            return self._result(False, f"The data collector cannot record {', '.join(topics)} ({msg}); "
                                       "restart it so it has the extra_topics parameter.")
        meta["modalities"] = sorted(self.enabled_modalities)
        meta["extra_topics"] = topics
        self.state.set_project(self.project.id, dataset=dataset, task=task, instruction=instruction,
                               operator=operator)
        result = self._result(*self.collection.start(self.project.data_root, dataset, task, meta))
        self._datasets_cache = (0.0, {})
        return result

    def stop_recording(self) -> dict:
        result = self._result(*self.collection.stop())
        self._datasets_cache = (0.0, {})
        return result

    def discard_last(self) -> dict:
        result = self._result(*self.collection.discard_last())
        self._datasets_cache = (0.0, {})
        return result

    # ---- cluster -------------------------------------------------------------------------------------------

    def _project_file(self, sub: str, name: str | None) -> Path | None:
        if not name or not self.project:
            return None
        path = (self.project.root / sub / name).resolve()
        if self.project.root.resolve() not in path.parents or not path.is_file():
            return None
        return path

    def sync_to_cluster(self, dataset: str, task: str, convert_config: str | None, convert: bool) -> dict:
        if not self.project:
            return self._result(False, "No project selected.")
        cfg_file = self._project_file("conversion", convert_config)
        if convert and convert_config and cfg_file is None:
            return self._result(False, f"Conversion config {convert_config} not found.")
        local = self.project.data_root / dataset / task
        return self._result(*self.cluster.sync_and_convert(
            self.project.id, self.project.cluster, local, dataset, task, cfg_file, convert))

    def train_on_cluster(self, config: str, dataset: str = "", task: str = "") -> dict:
        if not self.project:
            return self._result(False, "No project selected.")
        cfg_file = self._project_file("train_configs", config)
        if cfg_file is None:
            return self._result(False, "Choose a training config.")
        return self._result(*self.cluster.train(self.project.id, self.project.cluster, cfg_file, dataset, task))

    def refresh_jobs(self) -> dict:
        if self.project:
            self.cluster.refresh_slurm(self.project.cluster)
        return {"ok": True}

    # ---- monitor -----------------------------------------------------------------------------------------

    def _monitor_loop(self) -> None:
        period = 1.0 / float(self.cfg.get("monitor_rate_hz", 10.0))
        while self._alive:
            env = self.active_env
            runner = self.runner
            if env is not None and not (runner and runner.running and runner.env is env):
                try:
                    env.tick(runner.prompt if runner and runner.env is env and runner.prompt else None)
                except Exception as e:
                    log.debug("monitor tick failed: %s", e)
            time.sleep(period)

    @staticmethod
    def _camera_status(env: SceneEnv) -> list[dict]:
        now = time.time()
        return [{"name": n, "shape": list(img.shape), "age": now - stamp, "ok": now - stamp < 1.0}
                for n, (img, _fov, stamp) in sorted(env.render().cameras.items())]

    def camera_image(self, name: str) -> object:
        """Newest (downscaled) RGB image of a scene camera of the active env, or None."""
        env = self.active_env
        cam = env.render().cameras.get(name) if env else None
        return cam[0] if cam else None

    def latest_record(self) -> object:
        """The newest StepRecord of the active env (for the page's camera tiles)."""
        return self._last_record

    # ---- status ----------------------------------------------------------------------------------------------

    def status(self, since_event: int = 0) -> dict:
        project = self.project
        env = self.active_env
        remembered = self.state.project(project.id) if project else {}
        with self._lock:
            events = [e for e in self._events if e["id"] > since_event]
        runner = self.runner
        return {
            "time": time.time(),
            "projects": self.projects(),
            "project": project.summary() if project else None,
            "remembered": remembered,
            "components": self.processes.snapshot(),
            "data_collector": self.cfg["data_collector"]["component"],
            "policies": [p.to_dict(project.policies_dir) for p in project.policies.values()] if project else [],
            "policy": runner.snapshot() if runner else None,
            "loading": self._loading,
            "env": ({**env.describe(), "features": env.feature_status(), "cameras": self._camera_status(env)}
                    if env else None),
            "collection": self.collection.snapshot(),
            "modalities": self._modality_status() if project else [],
            "record_topics": mods.record_topics(self.modalities, self.enabled_modalities) if project else [],
            "pointcloud": self.pointcloud.snapshot(),
            "datasets": self.datasets(),
            "tasks": project.tasks if project else {},
            "conversion_configs": [str(p.relative_to(project.root / "conversion")) for p in project.files("conversion")] if project else [],
            "train_configs": [str(p.relative_to(project.root / "train_configs")) for p in project.files("train_configs")] if project else [],
            "cluster": {"host": project.cluster.get("host"), "configured": bool(project.cluster.get("host"))} if project else {},
            "jobs": self.cluster.snapshot(project.id) if project else [],
            "viewer": {"port": self.cfg["viewer"]["port"], "public_url": self.cfg["viewer"].get("public_url"),
                       "enabled": self.viewer is not None},
            "events": events,
        }

    def shutdown(self, stop_components: bool = True) -> None:
        self._alive = False
        if self.collection.recording:
            self.collection.stop()
        with self._lock:
            self._unload_policy()
        if stop_components:
            self.processes.stop_all(wait=True)
        if self.viewer:
            self.viewer.stop()
        self.ros.shutdown()
