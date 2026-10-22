#!/bin/bash
# Start the Fast-FoundationStereo point cloud node on the Jetson (over ssh): every camera of the
# rig -> one merged cloud on /ffs/points (config/cameras.yaml `pointcloud`).
#   bash scripts/run_pointcloud.sh [rig]
# Needs the cameras running with `stereo: true` and the Jetson set up once with
# ../Fast-FoundationStereo/jetson/setup_from_pc.sh. Ctrl-C / SIGTERM / closing ssh stops it.

ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
CONFIG="$ROOT_DIR/config/cameras.yaml"

RIG_VARS="$(python3 "$ROOT_DIR/scripts/python/camera_rig.py" "$CONFIG" "${1:-}" --pointcloud)" || exit 1
eval "$RIG_VARS"
echo "Point cloud for rig '$rig' ($cameras) on $ssh_target"

# -tt: the remote node gets a pty, so it is stopped (SIGHUP) when this ssh exits
ssh -tt -o BatchMode=yes -o ConnectTimeout=10 "$ssh_target" \
    "cd $pc_repo && exec ~/.pixi/bin/pixi run -e jetson rig-cloud --config-b64 $pc_config_b64" &
pid=$!
trap 'kill -TERM "$pid" 2>/dev/null' INT TERM
wait "$pid"
