> [!NOTE]
> This workspace is based on the **CRISP** ecosystem. For the full, original documentation, see <https://learnsyslab.github.io/crisp_controllers/>.

- [1. CRISP Workspace](#1-crisp-workspace)
  - [1.1. Prerequisites](#11-prerequisites)
    - [1.1.1. SSH Key](#111-ssh-key)
    - [1.1.2. Install Pixi](#112-install-pixi)
    - [1.1.3. Stable Gripper Device Names](#113-stable-gripper-device-names)
  - [1.2. Getting Started](#12-getting-started)
    - [1.2.1. Clone the Workspace](#121-clone-the-workspace)
    - [1.2.2. Enter the ROS Environment](#122-enter-the-ros-environment)
    - [1.2.3. Unified Setup \& Environment Initialization](#123-unified-setup--environment-initialization)
    - [1.2.4. Build the Workspace](#124-build-the-workspace)
    - [1.2.5 Libfranka](#125-libfranka)
  - [1.3. Running the Robot](#13-running-the-robot)
    - [1.3.1. Base Launch (with Safety Layer)](#131-base-launch-with-safety-layer)
    - [1.3.2. Teleoperation (Meta Quest VR)](#132-teleoperation-meta-quest-vr)
    - [1.3.3. Data Collection](#133-data-collection)
    - [1.3.4. Running a Policy](#134-running-a-policy)
    - [1.3.5. Running in Simulation](#135-running-in-simulation)
    - [1.3.6. Franka Studio (Web UI)](#136-franka-studio-web-ui)
  - [1.4. Advanced Usage](#14-advanced-usage)
    - [1.4.1. Environments](#141-environments)
    - [1.4.2. Additional Tasks](#142-additional-tasks)
  - [1.5. Troubleshooting](#15-troubleshooting)

# 1. CRISP Workspace

This repository provides a unified, **Pixi-managed** environment for the Franka Emika Panda robots, integrating the **CRISP** (C++ Real-time Impedance and Space Programming) ecosystem for high-perform[...]

## 1.1. Prerequisites

### 1.1.1. SSH Key
Several private repositories are cloned via SSH during setup. Make sure your SSH key is added to your GitHub account before running `pixi run setup`:
```bash
# Generate a key if you don't have one
ssh-keygen -t ed25519 -C "your_email@example.com"
# Print the public key to copy into GitHub
cat ~/.ssh/id_ed25519.pub
```

### 1.1.2. Install Pixi
If you haven't installed Pixi yet, run the following command:
```bash
curl -fsSL https://pixi.sh/install.sh | bash
source ~/.bashrc
```

### 1.1.3. Stable Gripper Device Names
The RH-P12-RN-A grippers enumerate as `/dev/ttyUSB<N>` in random order. Install a udev rule to pin each adapter to a fixed symlink (`/dev/dynamixel_left`, `/dev/dynamixel_right`):

```bash
sudo cp 99-dynamixel.rules /etc/udev/rules.d/ # located in franka_launch/udev
sudo udevadm control --reload-rules && sudo udevadm trigger
ls -l /dev/dynamixel_*
```

Furthermore, add your user to the `dialout` group
```
sudo usermod -aG dialout <user>
```

## 1.2. Getting Started

### 1.2.1. Clone the Workspace
```bash
git clone https://github.com/DFKI-SAIROL/pixi_franka_crisp_ws.git
cd pixi_franka_crisp_ws
```

> [!IMPORTANT]
> Clone **without** `--recursive`. `src/` holds ~14 submodules and a recursive clone fetches all
> of them, which defeats the install profile: the profile is applied by `pixi run setup` (it sets
> `submodule.active`, which is local config and cannot exist before the clone). A plain clone
> leaves every submodule as an empty directory until you pick a profile in step 1.2.3.

### 1.2.2. Enter the ROS Environment
Enter the Pixi shell first — `pixi run setup` calls `rosdep` which requires ROS to be available in PATH:
```bash
pixi shell -e humble
```

> [!NOTE]
> This is a temporary requirement until `rosdep` is replaced with a pure Pixi-managed dependency resolution.

### 1.2.3. Unified Setup & Environment Initialization
Inside the shell, clone all dependencies (CRISP, Franka ROS 2, Dynamixel, etc.) and install system deps:
```bash
pixi run setup            # uses the profile you last selected, or 'dfki'
pixi run setup sim-only   # or pick one explicitly
```

Which repositories get cloned is decided by an **install profile**. List them with:
```bash
pixi run profiles
```

| Profile | What it installs |
| --- | --- |
| `dfki` | Full lab platform: both arms, Dynamixel grippers, VR teleop, data collection |
| `sim-only` | Fake hardware only - no gripper drivers, no teleop |
| `minimal` | Smallest set that builds and drives an arm |

The profile is remembered in `git config --local workspace.profile`, so later `pixi run setup`
calls need no argument. Profiles, groups, and repository URLs are all declared in
[`config/workspace.yaml`](config/workspace.yaml) - add a repository there rather than editing
`scripts/setup.sh`.

Most of `src/` is tracked as **git submodules** pinned to specific commits, so a fresh clone
reproduces a known-good checkout. `pixi run setup` therefore installs the *pinned* versions rather
than the latest. To move to the tip of each tracked branch:

```bash
pixi run update-src   # git submodule update --remote --merge
```

That leaves a working-tree change you either commit (a deliberate version bump, reviewable in the
superproject) or discard. `src/franka_ros2/` is the exception - it stays a plain clone because the
setup trims tracked files out of the upstream tree.

> [!NOTE]
> This clones repositories into `src/`, applies custom patches, and installs dependencies via `rosdep` and `snap` (for `scrcpy`).

Then refresh the Pixi environment to ensure it is fully in sync with `pixi.lock`:
```bash
pixi install -e humble
```

### 1.2.4. Build the Workspace
Compile all C++ and Python packages:
```bash
pixi run build
```

> [!NOTE]
> `config/robot_overrides.yaml` is the single source of truth for per-arm robot/gripper settings (IPs, controllers, etc.). Edit it to overwrite the defaults in `franka_launch/config`.

> [!WARNING]
> The build may fail partway through due to some internal problems with RAM. If this happens, simply rerun `pixi run build` — colcon will pick up where it left off. See backlog for details. This may[...]

### 1.2.5 Libfranka
Make sure to run a realtime kernel and to add your user to the `realtime` group to successfully run libfranka.
```
sudo usermod -aG realtime <user>
```


---

## 1.3. Running the Robot

### 1.3.1. Base Launch (with Safety Layer)
This command opens a tmux session with the Franka driver and the safety layer:
```bash
pixi run robot
```

### 1.3.2. Teleoperation (Meta Quest VR)
To start the VR teleoperation stack:
```bash
pixi run teleop
```

### 1.3.3. Data Collection
To start a data collection run:
```bash
pixi run data-collection
```

### 1.3.4. Running a Policy

To run a trained policy, follow these steps:

1.  **Launch Pixi Shell**:
    ```bash
    pixi shell -e humble
    ```
2.  **Start Robot & Cameras**:
    ```bash
    pixi run robot
    ```
3.  **Run Teleoperation**:
    ```bash
    pixi run teleop
    ```
4.  **Start Policy Server**:
    Run this from the `lerobot` environment:
    ```bash
    micromamba activate lerobot
    python policy_server/policy_server.py
    ```
5.  **Start Policy Client**:
    ```bash
    pixi run client
    ```

> [!IMPORTANT]
> **Policy Client Controls:**
> - `h`: Drive to home position.
> - `r`: Start the policy.
> - `s`: Stop the policy.

> [!CAUTION]
> Keyboard keys will trigger actions as soon as the client is running. Be careful, as starting the policy by accident can be dangerous. Ensure you specify the correct policy checkpoint and type in the[...]

### 1.3.5. Running in Simulation
To run against fake/mock hardware instead of the physical arms (no robot connection required), set `use_fake_hardware: true` in `config/robot_overrides.yaml`:
```yaml
use_fake_hardware: true
```
Then launch as usual with `pixi run robot`.

> [!NOTE]
> Teleoperation still requires the physical Quest setup — `pixi run teleop` does not run in simulation.

### 1.3.6. Franka Studio (Web UI)
One browser page for the tasks above: start/stop robot, cameras, teleop and the data collector, load and run a
project's policies (vision or vision+language with a prompt), record demos or DAgger corrections into a dataset/task,
copy + convert data and start trainings on the cluster, and a 3D view of exactly what the policy sees.
```bash
pixi run studio   # http://localhost:8090
```
Projects live in [`projects/`](projects/) (start from `projects/example/`), settings in
[`config/studio.yaml`](config/studio.yaml). See [`studio/README.md`](studio/README.md).

**Point clouds.** Fast-FoundationStereo runs on the Jetson for every rig camera and publishes one
merged cloud (`/ffs/points`); switch it on in the studio's *Observation space* card or run
`pixi run pointcloud` (cameras with `jetson.stereo: true`, see [`config/cameras.yaml`](config/cameras.yaml)).
One-time setup of the Jetson from the PC: `bash ../Fast-FoundationStereo/jetson/setup_from_pc.sh`
(see `jetson/README.md` there).
Camera extrinsics (both wrist mounts, the left robot's base, the static `zed_middle`) come from an
ArUco grid board on the table: `pixi run calibrate-board` (see
[`scripts/python/board_calibration.py`](scripts/python/board_calibration.py)).

---

## 1.4. Advanced Usage

### 1.4.1. Environments
- **`humble`**: Default ROS 2 Humble environment.

### 1.4.2. Additional Tasks
- **`pixi run clean`**: Removes `build`, `install`, and `log` directories.

---
## 1.5. Troubleshooting

