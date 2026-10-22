"""Cluster operations over ssh, configured per project (``cluster:`` in project.yaml).

Commands are templates filled with ``{name}`` placeholders, so the cluster side (environment,
SLURM options, script paths) is configuration, not code::

    cluster:
      host: hessian                       # ssh host (from ~/.ssh/config)
      remote_raw_root: /mnt/.../custom_datasets/{project}/raw
      remote_dataset_root: /mnt/.../custom_datasets/{project}/lerobot
      remote_config_dir: /mnt/.../studio_configs/{project}
      convert_cmd: >-
        cd /mnt/.../data_postprocessing && sbatch ... --wrap "python scripts/convert_batched.py
        --raw-dir '{remote_raw_dir}' --config '{config}' --root '{remote_dataset_dir}'"
      train_cmd: "cd /mnt/.../training && sbatch train.sbatch '{config}'"

Placeholders: project, dataset, task, host, remote_raw_dir (``<raw_root>/<dataset>/<task>``),
remote_dataset_dir (``<dataset_root>/<dataset>_<task>``), config (remote path of the uploaded
config, or ``default_convert_config`` when none is chosen), config_name, timestamp.
A ``Submitted batch job <id>`` in the output is tracked as a SLURM job.
"""

from __future__ import annotations

import collections
import datetime
import json
import re
import shlex
import subprocess
import threading
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path

from franka_studio.paths import STATE_DIR

SLURM_RE = re.compile(r"Submitted batch job (\d+)")
SSH_OPTS = ["-o", "BatchMode=yes", "-o", "ConnectTimeout=15"]
JOBS_FILE = STATE_DIR / "cluster_jobs.json"


class _SafeDict(dict):
    def __missing__(self, key: str) -> str:
        return "{" + key + "}"


def fill(template: str, values: dict) -> str:
    return str(template).format_map(_SafeDict(values))


@dataclass
class ClusterJob:
    """A studio-side operation (sync, convert, train) and the SLURM job it may have started."""

    id: str
    kind: str
    title: str
    project: str
    created: float
    status: str = "running"         # running | done | failed | submitted
    slurm_id: str | None = None
    slurm_state: str | None = None
    lines: list = field(default_factory=list)

    def to_dict(self) -> dict:
        d = asdict(self)
        d["lines"] = self.lines[-200:]
        return d


class Cluster:
    """Runs and tracks the cluster operations of all projects."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self.jobs: collections.OrderedDict[str, ClusterJob] = collections.OrderedDict()
        self._load()

    # ---- persistence --------------------------------------------------------------------------

    def _load(self) -> None:
        try:
            for d in json.loads(JOBS_FILE.read_text()):
                job = ClusterJob(**d)
                if job.status == "running":  # the studio was restarted mid-operation
                    job.status = "failed"
                    job.lines.append("[studio] interrupted by a restart")
                self.jobs[job.id] = job
        except (OSError, ValueError, TypeError):
            pass

    def _save(self) -> None:
        try:
            STATE_DIR.mkdir(parents=True, exist_ok=True)
            data = [j.to_dict() for j in list(self.jobs.values())[-50:]]
            JOBS_FILE.write_text(json.dumps(data, indent=1))
        except OSError:
            pass

    # ---- running ------------------------------------------------------------------------------

    def _new_job(self, kind: str, title: str, project: str) -> ClusterJob:
        job = ClusterJob(id=f"{kind}-{int(time.time() * 1000)}", kind=kind, title=title,
                         project=project, created=time.time())
        with self._lock:
            self.jobs[job.id] = job
            while len(self.jobs) > 50:
                self.jobs.popitem(last=False)
        return job

    @staticmethod
    def _run(job: ClusterJob, argv: list[str]) -> int:
        job.lines.append("$ " + " ".join(shlex.quote(a) for a in argv))
        try:
            proc = subprocess.Popen(argv, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                    stdin=subprocess.DEVNULL, text=True, bufsize=1, errors="replace")
        except OSError as e:
            job.lines.append(f"[studio] {e}")
            return 127
        assert proc.stdout is not None
        for line in proc.stdout:
            line = line.rstrip("\n")
            # rsync --info=progress2 rewrites one line with \r: keep only the newest state
            if "\r" in line:
                line = line.split("\r")[-1]
            if line:
                job.lines.append(line)
                del job.lines[:-500]
            m = SLURM_RE.search(line)
            if m:
                job.slurm_id = m.group(1)
        return proc.wait()

    def _ssh(self, job: ClusterJob, host: str, command: str) -> int:
        return self._run(job, ["ssh", *SSH_OPTS, host, command])

    def _finish(self, job: ClusterJob, ok: bool) -> None:
        job.status = ("submitted" if job.slurm_id else "done") if ok else "failed"
        self._save()

    @staticmethod
    def _values(project: str, cfg: dict, dataset: str = "", task: str = "", **extra: str) -> dict:
        base = {"project": project, "dataset": dataset, "task": task, "host": cfg.get("host", ""),
                "timestamp": datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")}
        raw_root = fill(cfg.get("remote_raw_root", ""), base)
        ds_root = fill(cfg.get("remote_dataset_root", ""), base)
        base.update(
            remote_raw_root=raw_root, remote_dataset_root=ds_root,
            remote_raw_dir=f"{raw_root}/{dataset}/{task}" if dataset else raw_root,
            remote_dataset_dir=f"{ds_root}/{dataset}_{task}" if dataset else ds_root,
            remote_config_dir=fill(cfg.get("remote_config_dir", f"{raw_root}/../studio_configs"), base),
        )
        base.update(extra)
        return base

    def _upload_config(self, job: ClusterJob, host: str, local: Path, values: dict) -> str | None:
        remote_dir = values["remote_config_dir"]
        remote = f"{remote_dir}/{values['timestamp']}_{local.name}"
        if self._ssh(job, host, f"mkdir -p {shlex.quote(remote_dir)}") != 0:
            return None
        if self._run(job, ["scp", *SSH_OPTS, str(local), f"{host}:{remote}"]) != 0:
            return None
        return remote

    def sync_and_convert(self, project: str, cfg: dict, local_dir: Path, dataset: str, task: str,
                         convert_config: Path | None, convert: bool = True) -> tuple[bool, str]:
        """Rsync ``local_dir`` to the raw folder on the cluster, then run ``convert_cmd``."""
        host = cfg.get("host")
        if not host or not cfg.get("remote_raw_root"):
            return False, "Set cluster.host and cluster.remote_raw_root in project.yaml."
        if not local_dir.is_dir():
            return False, f"{local_dir} does not exist."
        if convert and not cfg.get("convert_cmd"):
            return False, "Set cluster.convert_cmd in project.yaml (or copy only)."
        title = f"{'Copy + convert' if convert else 'Copy'} {dataset}/{task}"
        job = self._new_job("convert" if convert else "sync", title, project)
        values = self._values(project, cfg, dataset, task)

        def work() -> None:
            dst = values["remote_raw_dir"]
            ok = self._ssh(job, host, f"mkdir -p {shlex.quote(dst)}") == 0
            ok = ok and self._run(job, ["rsync", "-a", "--partial", "--info=progress2,stats1",
                                        "-e", "ssh " + " ".join(SSH_OPTS),
                                        f"{local_dir}/", f"{host}:{dst}/"]) == 0
            if ok and convert:
                remote_cfg = ""
                if convert_config is not None:
                    remote_cfg = self._upload_config(job, host, convert_config, values) or ""
                    ok = bool(remote_cfg)
                else:
                    remote_cfg = fill(cfg.get("default_convert_config", ""), values)
                if ok:
                    cmd = fill(cfg["convert_cmd"], {**values, "config": remote_cfg,
                                                    "config_name": convert_config.name if convert_config else ""})
                    ok = self._ssh(job, host, cmd) == 0
            self._finish(job, ok)

        threading.Thread(target=work, daemon=True).start()
        return True, f"{title}: started."

    def train(self, project: str, cfg: dict, config: Path, dataset: str = "", task: str = "") -> tuple[bool, str]:
        host = cfg.get("host")
        if not host or not cfg.get("train_cmd"):
            return False, "Set cluster.host and cluster.train_cmd in project.yaml."
        job = self._new_job("train", f"Train {config.stem}", project)
        values = self._values(project, cfg, dataset, task)

        def work() -> None:
            remote_cfg = self._upload_config(job, host, config, values)
            ok = remote_cfg is not None
            if ok:
                cmd = fill(cfg["train_cmd"], {**values, "config": remote_cfg, "config_name": config.name})
                ok = self._ssh(job, host, cmd) == 0
            self._finish(job, ok)

        threading.Thread(target=work, daemon=True).start()
        return True, f"Training {config.stem}: submitting…"

    def refresh_slurm(self, cfg: dict) -> None:
        """Updates the SLURM state of submitted jobs (one ``squeue``/``sacct`` call)."""
        host = cfg.get("host")
        ids = [j.slurm_id for j in self.jobs.values() if j.slurm_id and j.slurm_state not in
               ("COMPLETED", "FAILED", "CANCELLED", "TIMEOUT", "OUT_OF_MEMORY")]
        if not host or not ids:
            return
        cmd = f"sacct -n -X -P -o JobID,State -j {','.join(ids)}"
        try:
            out = subprocess.run(["ssh", *SSH_OPTS, host, cmd], capture_output=True, text=True, timeout=30).stdout
        except (OSError, subprocess.TimeoutExpired):
            return
        states = dict(line.split("|", 1) for line in out.splitlines() if "|" in line)
        for job in self.jobs.values():
            if job.slurm_id in states:
                job.slurm_state = states[job.slurm_id].split()[0]
        self._save()

    def snapshot(self, project: str) -> list[dict]:
        jobs = [j for j in self.jobs.values() if j.project == project]
        return [j.to_dict() for j in reversed(jobs[-15:])]
