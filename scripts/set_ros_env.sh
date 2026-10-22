#/usr/bin/env sh

# Ensure conda libs take precedence over system libs
export LD_LIBRARY_PATH=$CONDA_PREFIX/lib:$LD_LIBRARY_PATH

ROS_ENV_FILE="scripts/personal_ros_env.sh"

if [ -f "$ROS_ENV_FILE" ]; then
    echo "$ROS_ENV_FILE already exists. Sourcing it..."
    . "$ROS_ENV_FILE"
else
    if [ "$ROS_DISTRO" = "jazzy" ] || [ "$ROS_DISTRO" = "humble" ]; then
        echo "$ROS_ENV_FILE not found. Using default environment variables..."
        export ROS_DOMAIN_ID=5
        export ROS_LOCALHOST_ONLY=0
        export RMW_IMPLEMENTATION=rmw_fastrtps_cpp #rmw_cyclonedds_cpp
    fi
fi

# Fast DDS profile: whitelist loopback + the camera link so the multi-homed PC does not
# announce unreachable locators (docker0 etc.) that black-hole Jetson->PC image data
# (topics visible, no messages). See the comment in config/fastdds_pc.xml.
FASTDDS_PC_PROFILE="${PIXI_PROJECT_ROOT:-$(pwd)}/config/fastdds_pc.xml"
if [ -f "$FASTDDS_PC_PROFILE" ]; then
    export FASTRTPS_DEFAULT_PROFILES_FILE="$FASTDDS_PC_PROFILE"
fi

WORKSPACE_SETUP="${PIXI_PROJECT_ROOT:-$(pwd)}/install/setup.bash"
if [ -f "$WORKSPACE_SETUP" ]; then
    . "$WORKSPACE_SETUP"
fi

ros2 daemon stop
ros2 daemon start
