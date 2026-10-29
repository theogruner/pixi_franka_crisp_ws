#!/bin/bash
# Start the Fast-FoundationStereo point cloud node: every camera of the rig -> one merged cloud on
# /ffs/points (config/cameras.yaml `pointcloud`). `pointcloud.host` says where it runs: on the
# Jetson (over ssh, set up once with ../Fast-FoundationStereo/jetson/setup_from_pc.sh) or on this
# PC (the `gsplat-policy` env of ../Fast-FoundationStereo, engine from its pc-build-engine task).
#   bash scripts/run_pointcloud.sh [rig]
# Needs the cameras running with `stereo: true`. Ctrl-C / SIGTERM / closing ssh stops it.

ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
CONFIG="$ROOT_DIR/config/cameras.yaml"

RIG_VARS="$(python3 "$ROOT_DIR/scripts/python/camera_rig.py" "$CONFIG" "${1:-}" --pointcloud)" || exit 1
eval "$RIG_VARS"

if [ "$pc_host" = pc ]; then
    repo="$(cd "$ROOT_DIR" && cd "${pc_repo/#\~/$HOME}" && pwd)" || exit 1
    echo "Point cloud for rig '$rig' ($cameras) on this PC ($repo)"
    # env -i: not the humble env of this workspace (its PYTHONPATH / ROS overlay); the PC's Fast
    # DDS profile, so the Jetson's images reach the node over the camera link
    env -i HOME="$HOME" PATH=/usr/bin:/bin FFS_FASTDDS_PROFILE="$ROOT_DIR/config/fastdds_pc.xml" \
        "$(command -v pixi)" run --manifest-path "$repo/pixi.toml" -e gsplat-policy \
        pc-rig-cloud --config-b64 "$pc_config_b64" &
else
    echo "Point cloud for rig '$rig' ($cameras) on $ssh_target"
    # -tt: the remote node gets a pty, so it is stopped (SIGHUP) when this ssh exits
    ssh -tt -o BatchMode=yes -o ConnectTimeout=10 "$ssh_target" \
        "cd $pc_repo && exec ~/.pixi/bin/pixi run -e jetson rig-cloud --config-b64 $pc_config_b64" &
fi
pid=$!
trap 'kill -TERM "$pid" 2>/dev/null' INT TERM
wait "$pid"
