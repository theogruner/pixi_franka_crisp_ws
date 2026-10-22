#!/bin/bash
set -uo pipefail

# Clone/update the repositories that make up the workspace.
#
# What gets cloned is decided by an install profile from config/workspace.yaml:
#   bash scripts/setup.sh [profile]
# The profile is remembered in `git config --local workspace.profile`, so later runs
# need no argument. List the options with:
#   python3 scripts/python/workspace.py profiles

ROS_DISTRO=${ROS_DISTRO:-humble}

ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
SRC_DIR="$ROOT_DIR/src"
WORKSPACE_PY="$ROOT_DIR/scripts/python/workspace.py"
OVERRIDES_FILE="$ROOT_DIR/config/robot_overrides.yaml"

# --- profile selection -------------------------------------------------------
PROFILE="${1:-}"
if [ -z "$PROFILE" ]; then
    PROFILE="$(git -C "$ROOT_DIR" config --local --get workspace.profile || true)"
fi
if [ -z "$PROFILE" ]; then
    PROFILE="dfki"
    echo "No profile given and none remembered, defaulting to '$PROFILE'."
fi

if ! pixi run -e humble python3 "$WORKSPACE_PY" repos --profile "$PROFILE" >/dev/null; then
    exit 1
fi
git -C "$ROOT_DIR" config --local workspace.profile "$PROFILE"

echo "Install profile: $PROFILE  (ROS_DISTRO: $ROS_DISTRO)"
echo "Working in: $SRC_DIR"

# Fail early if the profile cannot serve the hardware in robot_overrides.yaml.
pixi run -e humble python3 "$WORKSPACE_PY" check --profile "$PROFILE" --overrides "$OVERRIDES_FILE" || exit 1

SPECS="$(pixi run -e humble python3 "$WORKSPACE_PY" specs --profile "$PROFILE" --ros-distro "$ROS_DISTRO")"

# --- preflight: report every unreachable repo at once, not one at a time ------
# BatchMode/no terminal prompt so a missing key fails instead of hanging on a password.
export GIT_TERMINAL_PROMPT=0
export GIT_SSH_COMMAND="${GIT_SSH_COMMAND:-ssh -oBatchMode=yes}"

unreachable=""
while IFS='|' read -r name url branch rev flags; do
    [ -z "$name" ] && continue
    # A fresh clone leaves an empty dir per gitlink, so test the checkout, not the dir.
    [ -e "$SRC_DIR/$name/.git" ] && continue      # already checked out, no need to probe
    [ -e "$SRC_DIR/franka_ros2/$name/.git" ] && continue   # nested in the franka_ros2 bundle
    if ! git ls-remote --exit-code "$url" HEAD >/dev/null 2>&1; then
        case ",$flags," in
            *,optional,*) echo "Note: optional repo '$name' is unreachable, skipping it." ;;
            *) unreachable="$unreachable\n  $name -> $url" ;;
        esac
    fi
done <<< "$SPECS"

if [ -n "$unreachable" ]; then
    echo "Error: profile '$PROFILE' needs repositories you cannot currently access:" >&2
    echo -e "$unreachable" >&2
    echo "Check your SSH key / VPN, or pick a smaller profile (workspace.py profiles)." >&2
    exit 1
fi

mkdir -p "$SRC_DIR"
cd "$SRC_DIR"

# Look a repository's url up in the manifest instead of hardcoding it twice.
spec_url() {
    echo "$SPECS" | awk -F'|' -v want="$1" '$1 == want { print $2 }'
}

# --- franka_ros2 bundle ------------------------------------------------------
# Bespoke because upstream is trimmed in place and two internal repos are cloned
# inside its tree. Repos flagged `special` in the manifest are handled here, not by
# the generic loop below. This block disappears once the DFKI franka_ros2 fork lands.
if echo "$SPECS" | grep -q '^franka_ros2|'; then
    if [ ! -d "franka_ros2" ]; then
        echo "Cloning franka_ros2 ($ROS_DISTRO)..."
        git clone -b "$ROS_DISTRO" https://github.com/frankarobotics/franka_ros2.git ./franka_ros2
        git -C ./franka_ros2 checkout 7ed0458

        # Remove nodes not used in this pipeline
        rm -rf ./franka_ros2/franka_gazebo ./franka_ros2/franka_fr3_moveit_config ./franka_ros2/franka_mobile_example_controllers ./franka_ros2/franka_mobile_sensors ./franka_ros2/franka_gazebo_bringup
        rm -rf ./franka_ros2/libfranka ./franka_ros2/franka_robot_state_broadcaster

        cd ./franka_ros2/
        git clone --recurse-submodules "$(spec_url libfranka)" libfranka
        git clone "$(spec_url franka_robot_state_broadcaster)" franka_robot_state_broadcaster
        cd ..

        vcs import ./franka_ros2 < ./franka_ros2/dependency.repos --recursive --skip-existing
        rosdep install --from-paths ./franka_ros2 --ignore-src --rosdistro "$ROS_DISTRO" --skip-keys "ignition-plugin franka_ign_ros2_control" -y
    else
        echo "franka_ros2 already exists, pulling changes for sub-repos..."
        git -C franka_ros2 pull
        for nested in libfranka franka_robot_state_broadcaster; do
            want="$(spec_url "$nested")"
            have="$(git -C "franka_ros2/$nested" remote get-url origin 2>/dev/null || true)"
            if [ -n "$have" ] && [ "$have" != "$want" ]; then
                echo "  franka_ros2/$nested: remote moved, repointing to $want"
                git -C "franka_ros2/$nested" remote set-url origin "$want"
            fi
            git -C "franka_ros2/$nested" pull
        done
        git -C franka_ros2/libfranka submodule update --init --recursive
    fi
fi

cd "$SRC_DIR"

# --- everything else ---------------------------------------------------------
# src/ is git submodules: .gitmodules is committed, so a clone already carries every
# gitlink and this only has to activate the profile and check the tree out. The
# franka_ros2 bundle above is the exception (special: true in the manifest).
if [ ! -f "$ROOT_DIR/.gitmodules" ]; then
    echo "Error: .gitmodules is missing - src/ cannot be populated." >&2
    echo "This file is committed; restore it with 'git checkout .gitmodules'." >&2
    exit 1
fi

echo "Activating profile '$PROFILE'"
PROFILE_PATHS="$(pixi run -e humble python3 "$WORKSPACE_PY" paths --profile "$PROFILE")"
git -C "$ROOT_DIR" config --local --unset-all submodule.active 2>/dev/null || true
while read -r subpath; do
    [ -n "$subpath" ] && git -C "$ROOT_DIR" config --local --add submodule.active "$subpath"
done <<< "$PROFILE_PATHS"

# Contributor ergonomics. push.recurseSubmodules=on-demand is the important one: it
# refuses to push a superproject commit pointing at a submodule commit nobody can fetch.
git -C "$ROOT_DIR" config --local submodule.recurse true
git -C "$ROOT_DIR" config --local push.recurseSubmodules on-demand
git -C "$ROOT_DIR" config --local status.submoduleSummary true
git -C "$ROOT_DIR" config --local diff.submodule log

# Propagate any url change in .gitmodules to already-checked-out submodules.
git -C "$ROOT_DIR" submodule sync --recursive >/dev/null

# Only submodules matching submodule.active are touched, so this installs exactly
# the profile and leaves other groups uncheckouted.
git -C "$ROOT_DIR" submodule update --init --recursive

pixi run -e humble python3 "$WORKSPACE_PY" verify-submodules || \
    echo "warning: .gitmodules has drifted from config/workspace.yaml (see above)." >&2

# `git clone --recursive` checks out every submodule, because submodule.active is local
# config that does not exist until this script runs. Report the extras instead of deleting
# them: colcon will build them, which is slow but harmless, and the trees may hold work.
extra=""
while read -r subpath; do
    [ -z "$subpath" ] && continue
    [ -e "$ROOT_DIR/$subpath/.git" ] || continue
    grep -qx "$subpath" <<< "$PROFILE_PATHS" || extra="$extra\n  $subpath"
done <<< "$(git -C "$ROOT_DIR" config -f "$ROOT_DIR/.gitmodules" --get-regexp path | awk '{print $2}')"

if [ -n "$extra" ]; then
    echo
    echo "Note: these submodules are checked out but not part of profile '$PROFILE':" >&2
    echo -e "$extra" >&2
    echo "colcon will still build them. Remove one with 'git submodule deinit <path>'" >&2
    echo "(it refuses if the tree has uncommitted work)." >&2
fi

# --- system tools ------------------------------------------------------------
if ! command -v scrcpy &> /dev/null || [ "$(scrcpy --version | head -n 1 | grep -o '1\.')" = "1." ]; then
    echo "Installing scrcpy from snap (v3.x required for audio)..."
    sudo apt remove -y scrcpy
    sudo snap install scrcpy
else
    echo "Modern scrcpy is already installed, skipping."
fi

echo "Setup and sync complete (profile: $PROFILE)."
