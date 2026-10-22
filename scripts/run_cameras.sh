#!/bin/bash
# Start the ZED camera rig: the ZED wrappers on the Jetson (over ssh) and the rig aggregator here.
#   bash scripts/run_cameras.sh [rig]      (rig defaults to `rig` in config/cameras.yaml)
# Ctrl-C / SIGTERM stops both; closing the ssh session stops the remote launch.

ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
CONFIG="$ROOT_DIR/config/cameras.yaml"
PARAMS="$(mktemp --suffix=.yaml /tmp/zed_rig_aggregator_XXXX)"

RIG_VARS="$(python3 "$ROOT_DIR/scripts/python/camera_rig.py" "$CONFIG" "${1:-}" --params "$PARAMS")" || exit 1
eval "$RIG_VARS"
echo "Camera rig '$rig': $cameras (ZED on $ssh_target${depth:+, with depth}${zed_mode:+, overrides: $zed_mode})"

pids=()
# -tt: the remote launch gets a pty, so it is stopped (SIGHUP) when this ssh exits
ssh -tt -o BatchMode=yes -o ConnectTimeout=10 "$ssh_target" "source $remote_setup && bash $launch_script $rig $zed_mode" &
pids+=($!)
# The binary itself, not `ros2 run`: that wrapper does not pass SIGTERM on, which left the
# aggregator running after the Jetson side had exited.
AGGREGATOR="$(ros2 pkg prefix zed_rig_aggregator_node)/lib/zed_rig_aggregator_node/zed_rig_aggregator_node"
"$AGGREGATOR" --ros-args -r __node:=zed_rig_aggregator --params-file "$PARAMS" &
pids+=($!)

# Background jobs of a script ignore SIGINT, so forward a stop as SIGTERM. If either side
# exits on its own (e.g. a camera fails on the Jetson), stop the other one too.
trap 'kill -TERM "${pids[@]}" 2>/dev/null' INT TERM
wait -n
kill -TERM "${pids[@]}" 2>/dev/null
wait
rm -f "$PARAMS"
