#!/bin/bash

ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
OVERRIDES_FILE="$ROOT_DIR/config/robot_overrides.yaml"

# Read defaults from config/robot_overrides.yaml (single source of truth,
# also read by scripts/run_robot.sh and scripts/setup.sh).
eval "$(pixi run -e humble python3 "$ROOT_DIR/scripts/python/read_overrides.py" "$OVERRIDES_FILE")"

# Start the ADB daemon once up front. Otherwise each per-arm teleop node finds no daemon and
# starts its own at the same moment; the loser gets "Address already in use" and exits, so
# only one arm is teleoperable.
if command -v adb &> /dev/null; then
    adb start-server
fi

pixi run -e humble \
ros2 launch franka_meta_quest start.launch.py spawn_franka_left:=$spawn_franka_left spawn_franka_right:=$spawn_franka_right overrides_file:=$OVERRIDES_FILE
