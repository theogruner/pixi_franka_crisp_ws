"""Projects: one folder per project below ``projects_dir``::

    projects/<name>/
        project.yaml        name, data_root, env, tasks, cluster, components
        env.yaml            the scene env shown when no policy is loaded (mappings + scene)
        policies/           one yaml per policy (see policy/registry.py), any sub-folders
        conversion/         dataset conversion configs uploaded for "copy + convert"
        train_configs/      training configs uploaded for "train on cluster"
"""  # noqa: D208

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import yaml

from franka_studio.paths import resolve
from franka_studio.policy.registry import PolicySpec, discover


@dataclass
class Project:
    """A loaded project."""

    id: str
    root: Path
    cfg: dict
    env_config: dict = field(default_factory=dict)
    policies: dict[str, PolicySpec] = field(default_factory=dict)
    error: str = ""

    @property
    def name(self) -> str:
        return self.cfg.get("name", self.id)

    @property
    def data_root(self) -> Path:
        return resolve(str(self.cfg.get("data_root", f"~/data/{self.id}")).format(project=self.id), self.root)

    @property
    def policies_dir(self) -> Path:
        return resolve(self.cfg.get("policies_dir", "policies"), self.root)

    @property
    def tasks(self) -> dict[str, str]:
        """Task folder -> default instruction."""
        tasks = self.cfg.get("tasks") or {}
        if isinstance(tasks, list):
            return {t: "" for t in tasks}
        return {k: v or "" for k, v in tasks.items()}

    @property
    def cluster(self) -> dict:
        return self.cfg.get("cluster") or {}

    def files(self, sub: str) -> list[Path]:
        d = self.root / sub
        return sorted(p for p in d.rglob("*") if p.is_file() and not p.name.startswith(".")) if d.is_dir() else []

    def refresh_policies(self) -> None:
        self.policies = discover(self.policies_dir, self.env_config.get("scene"))

    def summary(self) -> dict:
        return {"id": self.id, "name": self.name, "description": self.cfg.get("description", ""),
                "root": str(self.root), "data_root": str(self.data_root), "error": self.error}


def load_project(root: Path) -> Project:
    pid = root.name
    try:
        with open(root / "project.yaml") as f:
            cfg = yaml.safe_load(f) or {}
    except Exception as e:
        return Project(pid, root, {}, error=f"project.yaml: {e}")
    project = Project(pid, root, cfg)
    env_file = cfg.get("env", "env.yaml")
    try:
        with open(resolve(env_file, root)) as f:
            project.env_config = yaml.safe_load(f) or {}
    except FileNotFoundError:
        project.env_config = {}
    except Exception as e:
        project.error = f"{env_file}: {e}"
    merge_camera_poses(project)
    project.refresh_policies()
    return project


CAMERA_POSES = "camera_poses.yaml"


def merge_camera_poses(project: Project) -> None:
    """Poses placed in the viewer (camera_poses.yaml) fill in cameras that have no URDF link."""
    try:
        poses = yaml.safe_load((project.root / CAMERA_POSES).read_text()) or {}
    except (OSError, yaml.YAMLError):
        return
    cams = project.env_config.setdefault("scene", {}).setdefault("cameras", {})
    for name, pose in poses.items():
        if not (cams.get(name) or {}).get("link"):
            cams[name] = {**(cams.get(name) or {}), **pose}


def save_camera_pose(project: Project, name: str, position: list, wxyz: list) -> None:
    path = project.root / CAMERA_POSES
    try:
        poses = yaml.safe_load(path.read_text()) or {}
    except (OSError, yaml.YAMLError):
        poses = {}
    poses[name] = {"position": position, "wxyz": wxyz}
    header = "# Written by the studio's viewer (Camera placement); merged into env.yaml scene.cameras.\n"
    path.write_text(header + yaml.safe_dump(poses, sort_keys=True, default_flow_style=None))
    cams = project.env_config.setdefault("scene", {}).setdefault("cameras", {})
    cams[name] = {**(cams.get(name) or {}), "position": position, "wxyz": wxyz}


def list_projects(projects_dir: Path) -> list[str]:
    if not projects_dir.is_dir():
        return []
    return sorted(p.name for p in projects_dir.iterdir() if (p / "project.yaml").is_file())
