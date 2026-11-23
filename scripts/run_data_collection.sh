#!/bin/bash
# Extra arguments go to the launch file, e.g. `pixi run data-collection label_dialog:=false`
pixi run -e humble \
ros2 launch franka_data_collection start.launch.py "$@"
