#/usr/bin/env sh

# Ensure conda libs take precedence over system libs
export LD_LIBRARY_PATH=$CONDA_PREFIX/lib:$LD_LIBRARY_PATH

ROS_ENV_FILE="scripts/personal_ros_env.sh"

if [ -f "$ROS_ENV_FILE" ]; then
    echo "$ROS_ENV_FILE already exists. Sourcing it..."
    . "$ROS_ENV_FILE"
fi

# Defaults for whatever the personal env file did not set. Deliberately NOT gated on
# $ROS_DISTRO: in the default pixi environment ROS_DISTRO is exported by a conda
# activation script that runs after this one, so such a guard silently skips this
# block. `pixi run robot` then sends `export ROS_DOMAIN_ID=` over ssh and the jetson
# cameras come up on domain 0 while every PC node runs on domain 5.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-5}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}" #rmw_cyclonedds_cpp

# Fast DDS profile: whitelist loopback + camera link so this multi-homed PC does not
# announce unreachable locators (docker0 etc.) that blackhole Jetson->PC image data.
FASTDDS_PC_PROFILE="${PIXI_PROJECT_ROOT:-$(pwd)}/config/fastdds_config.xml"
if [ -f "$FASTDDS_PC_PROFILE" ] && [ "$RMW_IMPLEMENTATION" = "rmw_fastrtps_cpp" ]; then
    export FASTRTPS_DEFAULT_PROFILES_FILE="$FASTDDS_PC_PROFILE"
    # The profile's unicast discovery ports are derived from the domain id. On a
    # mismatch the later participants island silently, so say so loudly.
    if [ "$ROS_DOMAIN_ID" != "5" ]; then
        echo "WARNING: ROS_DOMAIN_ID=$ROS_DOMAIN_ID but config/fastdds_config.xml lists" >&2
        echo "         discovery ports for domain 5. Regenerate them or discovery will" >&2
        echo "         silently island nodes (see the comment in that file)." >&2
    fi
fi

WORKSPACE_SETUP="${PIXI_PROJECT_ROOT:-$(pwd)}/install/setup.bash"
if [ -f "$WORKSPACE_SETUP" ]; then
    . "$WORKSPACE_SETUP"
fi

ros2 daemon stop
ros2 daemon start
