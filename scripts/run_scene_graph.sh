#!/bin/bash
# Start the object detection + scene graph node (config/scene_graph.yaml) in the env of its own repo
# (../pose_tracking: torch, SAM, open3d). What to detect comes from the studio (/scene_graph/request).
#   bash scripts/run_scene_graph.sh
# Needs the point cloud running (its RGB-D). Ctrl-C / SIGTERM stops it.

ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
CONFIG="$ROOT_DIR/config/scene_graph.yaml"

eval "$(python3 - "$CONFIG" <<'PY'
import base64, json, shlex, sys
import yaml
cfg = yaml.safe_load(open(sys.argv[1])) or {}
repo = cfg.pop("repo", "../pose_tracking")
print(f"repo={shlex.quote(repo)}")
print(f"threads={int(cfg.pop('cpu_threads', 8))}")
print(f"config_b64={base64.b64encode(json.dumps(cfg).encode()).decode()}")
PY
)" || exit 1
repo="$(cd "$ROOT_DIR" && cd "${repo/#\~/$HOME}" && pwd)" || exit 1
echo "Object detection ($repo)"

# env -i: not the humble env of this workspace (its PYTHONPATH / ROS overlay); the PC's Fast DDS
# profile, so the Jetson's and this PC's topics both reach the node. OMP_NUM_THREADS: Open3D and
# torch would take every core otherwise (registration froze the PC). PYTHONFAULTHANDLER: a
# segfault in native code (Open3D, torch) prints the Python stack of every thread.
env -i HOME="$HOME" PATH=/usr/bin:/bin PYTHONFAULTHANDLER=1 POSE_FASTDDS_PROFILE="$ROOT_DIR/config/fastdds_pc.xml" \
    OMP_NUM_THREADS="$threads" MKL_NUM_THREADS="$threads" OPENBLAS_NUM_THREADS="$threads" \
    "$(command -v pixi)" run --manifest-path "$repo/pixi.toml" detect --config-b64 "$config_b64" &
pid=$!
trap 'kill -TERM "$pid" 2>/dev/null' INT TERM
wait "$pid"
