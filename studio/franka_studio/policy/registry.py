"""Discovers the policies of a project (``projects/<name>/policies/**/*.yaml``).

A policy file is either

* a policy client config itself (``mappings``, ``actions``, ``policy``, ``server_address``, ...),
  optionally with a ``studio:`` block, or
* a small entry that points at an unchanged client config::

      studio:
        name: X-VLA · place cup · EE abs
        kind: vision_language          # vision | vision_language
        prompts: ["Pick up the cup and place it under the coffee machine."]
        description: trained on 120 demos
      client_config: ../../client_configs/robi_client_place_cup.yaml   # relative to this file

``kind`` decides whether the UI asks for a prompt. When it is missing it is inferred: a config with
a ``policy.task`` or a VLA policy type is vision+language, anything else is vision only.
Scene settings (``scene``) missing from a policy are taken from the project's env config.
"""

from __future__ import annotations

import datetime
import hashlib
import re
from dataclasses import dataclass, field
from pathlib import Path

import yaml

VISION, VISION_LANGUAGE = "vision", "vision_language"
VLA_TYPES = {"xvla", "pi0", "pi05", "pi0fast", "smolvla", "molmoact", "molmoact2", "openvla", "groot", "beast_refiner"}
RUN_TIMESTAMP = re.compile(r"(\d{4}-\d{2}-\d{2})_(\d{2})-(\d{2})-(\d{2})")


@dataclass
class PolicySpec:
    """One selectable policy."""

    id: str
    name: str
    kind: str
    file: Path
    config: dict
    prompts: list[str] = field(default_factory=list)
    description: str = ""
    policy_type: str = ""
    checkpoint: str = ""
    step: str = ""
    trained: datetime.datetime | None = None
    server_address: str = ""
    error: str = ""

    @property
    def language(self) -> bool:
        return self.kind == VISION_LANGUAGE

    def to_dict(self, root: Path) -> dict:
        try:
            rel = str(self.file.relative_to(root))
        except ValueError:
            rel = str(self.file)
        return {
            "id": self.id, "name": self.name, "kind": self.kind, "file": rel,
            "prompts": self.prompts, "description": self.description,
            "policy_type": self.policy_type, "checkpoint": self.checkpoint, "step": self.step,
            "trained": self.trained.isoformat(timespec="minutes") if self.trained else None,
            "server_address": self.server_address, "error": self.error,
            "frequency": self.config.get("frequency"),
            "actions_per_chunk": (self.config.get("policy") or {}).get("actions_per_chunk"),
            "features": list((self.config.get("mappings") or {}).keys()),
        }


def _checkpoint_info(ckpt: str) -> tuple[str, datetime.datetime | None]:
    parts = Path(ckpt).parts
    step = ""
    if "checkpoints" in parts and parts.index("checkpoints") + 1 < len(parts):
        step = parts[parts.index("checkpoints") + 1]
    stamps = RUN_TIMESTAMP.findall(ckpt)
    trained = None
    if stamps:
        day, h, m, s = stamps[-1]
        trained = datetime.datetime.fromisoformat(f"{day}T{h}:{m}:{s}")
    return step, trained


def load_policy(file: Path, scene_defaults: dict | None = None) -> PolicySpec:
    """Parses one policy file; problems end up in ``spec.error`` instead of raising."""
    pid = hashlib.sha1(str(file).encode()).hexdigest()[:10]
    try:
        with open(file) as f:
            raw = yaml.safe_load(f) or {}
    except Exception as e:
        return PolicySpec(pid, file.stem, VISION, file, {}, error=f"unreadable: {e}")
    meta = raw.get("studio") or {}
    config = raw
    error = ""
    if raw.get("client_config"):
        target = (file.parent / raw["client_config"]).expanduser().resolve()
        try:
            with open(target) as f:
                config = yaml.safe_load(f) or {}
        except Exception as e:
            config, error = {}, f"client_config {target}: {e}"
    config = dict(config)
    if scene_defaults and "scene" not in config:
        config["scene"] = scene_defaults
    policy = config.get("policy") or {}
    policy_type = str(policy.get("policy_type", ""))
    kind = meta.get("kind")
    if kind not in (VISION, VISION_LANGUAGE):
        kind = VISION_LANGUAGE if (policy.get("task") or policy_type.lower() in VLA_TYPES) else VISION
    prompts = list(meta.get("prompts") or ([policy["task"]] if policy.get("task") else []))
    config["_language"] = kind == VISION_LANGUAGE
    ckpt = str(policy.get("pretrained_name_or_path", ""))
    step, trained = _checkpoint_info(ckpt)
    if not error and not config.get("mappings"):
        error = "no observation mappings"
    if not error and not config.get("server_address"):
        error = "no server_address"
    return PolicySpec(
        id=pid, name=meta.get("name") or file.stem, kind=kind, file=file, config=config,
        prompts=prompts, description=meta.get("description", ""), policy_type=policy_type,
        checkpoint=ckpt, step=step, trained=trained,
        server_address=str(config.get("server_address", "")), error=error,
    )


def discover(policies_dir: Path, scene_defaults: dict | None = None) -> dict[str, PolicySpec]:
    """All policies below ``policies_dir``, newest trained first."""
    if not policies_dir.is_dir():
        return {}
    specs = [load_policy(p, scene_defaults) for p in sorted(policies_dir.rglob("*.y*ml"))
             if p.suffix in (".yaml", ".yml")]
    specs.sort(key=lambda s: (s.trained is not None, s.trained or datetime.datetime.min), reverse=True)
    return {s.id: s for s in specs}
