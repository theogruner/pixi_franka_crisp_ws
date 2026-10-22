#!/bin/bash

# USE_RVIZ=true pixi run robot  -> also start rviz2 (off by default; Franka Studio has a 3D view)
ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
OVERRIDES_FILE="$ROOT_DIR/config/robot_overrides.yaml"

# Read defaults from config/robot_overrides.yaml (single source of truth,
# also read by scripts/setup.sh and by the ROS2 launch files).
eval "$(pixi run -e humble python3 "$ROOT_DIR/scripts/python/read_overrides.py" "$OVERRIDES_FILE")"
echo "config/robot_overrides.yaml: spawn_franka_left=$spawn_franka_left spawn_franka_right=$spawn_franka_right use_fake_hardware=$use_fake_hardware bypass_safety=$bypass_safety gripper_type_left=$gripper_type_left gripper_type_right=$gripper_type_right"

check_gripper() {
    local side="$1"
    local dev="/dev/dynamixel_${side}"
    if [ ! -e "$dev" ]; then
        echo "Error: spawn_franka_${side} is true but gripper device $dev was not found."
        exit 1
    fi
    if ! pixi run -e humble python3 "$(dirname "$0")/python/check_gripper.py" "$dev"; then
        echo "Error: spawn_franka_${side} is true but the gripper on $dev is not responding. Is it powered on?"
        exit 1
    fi
}

if [ "$use_fake_hardware" = false ]; then
    [ "$spawn_franka_left" = true ] && [ "$gripper_type_left" = rh_p12_rn_a ] && check_gripper left
    [ "$spawn_franka_right" = true ] && [ "$gripper_type_right" = rh_p12_rn_a ] && check_gripper right
fi

# ROBOT_NO_TMUX=1 (set by Franka Studio) keeps both launches attached to this process, so
# whoever started the script can stream their output and stop them with one signal.
if [ -n "${ROBOT_NO_TMUX:-}" ] || ! command -v tmux &> /dev/null
then
    [ -z "${ROBOT_NO_TMUX:-}" ] && echo "tmux is not installed. Running commands in background instead."
    pids=()
    ros2 launch franka_launch example.launch.py spawn_franka_left:=$spawn_franka_left spawn_franka_right:=$spawn_franka_right use_fake_hardware:=$use_fake_hardware use_rviz:=${USE_RVIZ:-false} overrides_file:=$OVERRIDES_FILE &
    pids+=($!)
    ros2 launch robot_ik_layer start_ijk.launch.py spawn_franka_left:=$spawn_franka_left spawn_franka_right:=$spawn_franka_right bypass_safety:=$bypass_safety overrides_file:=$OVERRIDES_FILE &
    pids+=($!)
    # Background jobs of a script ignore SIGINT, so forward a stop as SIGTERM (ros2 launch
    # shuts its nodes down cleanly on it).
    trap 'kill -TERM "${pids[@]}" 2>/dev/null' INT TERM
    wait
    wait
    exit 0
fi

# Use tmux to open multiple terminals side-by-side
SESSION="robot_run"
tmux new-session -d -s $SESSION "pixi run -e humble ros2 launch franka_launch example.launch.py spawn_franka_left:=$spawn_franka_left spawn_franka_right:=$spawn_franka_right use_fake_hardware:=$use_fake_hardware use_rviz:=${USE_RVIZ:-false} overrides_file:=$OVERRIDES_FILE ; exec bash"
tmux split-window -v -t $SESSION "pixi run -e humble ros2 launch robot_ik_layer start_ijk.launch.py spawn_franka_left:=$spawn_franka_left spawn_franka_right:=$spawn_franka_right bypass_safety:=$bypass_safety overrides_file:=$OVERRIDES_FILE ; exec bash"

echo "Started robot launch nodes in a tmux session."
tmux attach-session -t $SESSION
