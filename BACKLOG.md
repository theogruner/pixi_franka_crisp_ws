# Backlog

Open tasks at the top, dated log of done/notable changes below. Newest first.

## Backlog / TODO
- [ ] **Check the RAM** — `pixi run build` intermittently crashes `cc1plus` (suspected faulty RAM). Run `memtest86+` (several passes), reseat/replace DIMMs. Workaround for now: re-run the build a few times until it passes.
- [ ] **Bind the `build` task to an environment** — `build` sits in the top-level `[tasks]` with no
      `environment=`, so a bare `pixi run build` silently uses the `default` env and poisons the
      CMake caches with `envs/default` paths. `test` already uses the
      `[{task=..., environment="humble"}]` form. Binding `build` the same way would fix the footgun
      but pin the workspace to `humble`; decide how `jazzy` should be built first.
- [ ] **Make `libfranka` + `franka_robot_state_broadcaster` actually public** — moved to
      `DFKI-SAIROL` on GitHub 2026-08-27, but anonymous https is still denied, so they remain the
      only private deps and still gate *every* profile
      (`workspace.py repos --profile minimal --visibility private`). Flip visibility, then set
      `visibility: public` and switch their urls to https in `config/workspace.yaml`.
- [ ] **Switch public repos to `https://` in `config/workspace.yaml`** — 12 DFKI-SAIROL repos are
      already public but cloned over SSH, so outside contributors are blocked by transport, not
      permissions.
- [ ] **Remove `zed_rig_aggregator_node` from `franka_py`** — the launch references are gone from
      this repo, but the package itself still lives in `src/franka_py/zed_rig_aggregator_node`.
      That is the "embedded somewhere" case; deleting it is a change in the `franka_py` repo.
- [ ] **Check for duplicate package names** — the IAS `franka_robot_state_broadcaster` repo also
      ships `franka_ros2` and `franka_semantic_components` packages, which `franka_ros2` provides
      too. Verify colcon is picking the intended copy.
- [ ] Widen `rosdep install` from `src/franka_ros2` to all of `src/` (deliberately left alone
      during the manifest refactor to keep behaviour unchanged).
- [x] Add Kay's config file for some tresholds: https://github.com/pompetzki/pixi_franka_ros2/blob/6d56274990ca0692217adebcd117da310c9b7381/scripts/launch/main.launch.py 
- [x] joint trajectory controller interpolators (move Puze's code for linear and bspline interpolation)
- [x] Move robot related configs to franka_launch
- [x] Option for installing the grippers in src/ 
- [x] move msgs to the correct repo
- [x] custom to aggregator from franka_py
- [x] data_collection is a standalone package
- [x] remove franka_ui
- [x] place the link to the crisp documnetation in the top of our


## Log

### 2026-10-23

#### Joint-space home
- **Problem:** All homes were Cartesian `grasp_point` targets, which turning the end effectors by
  pi made wrong; the old stack's joint homing (`home_joints`, safety node `policy_target_joint`)
  was lost, and the studio's Home published a `PoseStamped` on a path that is a service.
- **Solution:** `franka_client/joint_home.py`: minimum-jerk joint trajectory through the current
  safety node's `policy_target_dq` (all joints together, peak `home_joint_max_speed`), settle on
  the measured joints, then re-target the IK at the reached pose (else it pulls back). Home
  joints per arm in `teleop_<side>.yaml` (values of the old setup, 2026-10-06; capture new ones
  with `python -m franka_client.joint_home`). Used by teleop B (held), the studio's Home (project
  `home_pose` with `ns`; Stop aborts) and the clients' H. Cartesian home stays as fallback.

#### Point cloud: steadier, faster, cleaner
- **Problem:** The right view vanished every other refresh, ~1 Hz, speckle noise.
- **Solution / findings (`../Fast-FoundationStereo/jetson/ffs_rig_node.py`):**
  - `max_age` 0.35 -> 2 s: the cameras are processed one after the other (~0.3 s each), so the
    right cloud was always just older than the limit; now each camera's last cloud stays.
  - Raw image subscriptions (stamp read from the CDR header, only the used pair deserialized):
    6 x 30 Hz images were all deserialized in Python. Per-stage timings in `/ffs/status`.
  - GPU/CPU pipeline: a GPU thread runs the network camera after camera, a CPU thread projects /
    filters meanwhile. All three views refresh every ~0.65 s (was ~1.15 s); a merged cloud goes
    out after every camera (~4.5 Hz). Now GPU-bound: `disparity` ~220 ms per camera (engine alone
    ~120 ms in the bench; the rest is preparation and sharing the GPU with the ZED wrapper).
  - Density filter `support` (1 cm cells, >= 4 points of the same camera): removes speckle.
- **Open:** faster engine (4 iterations and/or batch 3 and/or 320x224); tighter `crop` / `zfar`.

### 2026-10-22

#### Point cloud observation (Fast-FoundationStereo on the Jetson)
- **Problem:** Point clouds as an observation / in the recordings, without loading the PC.
- **Solution:** `../Fast-FoundationStereo/jetson/ffs_rig_node.py` runs one TensorRT engine for
  all rig cameras on the Jetson and publishes one merged, world-frame cloud (`/ffs/points`); set up
  from the PC with `jetson/setup_from_pc.sh` (the Jetson has no internet: pixi binary copied,
  packages through an `ssh -R` SOCKS proxy). Started by `pixi run pointcloud` or the studio's
  Point cloud modality (new *Observation space* card: modalities in `env.yaml`, recorded topics via
  the collector's new `extra_topics` parameter). ZED depth is off now (`jetson.depth: false`,
  aggregator RGB-only), which freed the Jetson GPU and halved the camera link (475 -> 207 MB/s).
- **Findings:**
  - Both wrist cameras' frames were unprefixed (`zed_left_camera_optical_frame`) and collided on
    `/tf_static`; now `<arm_prefix>_zed_*`.
  - The wrist camera mount: both holders are at the original yaw (-pi/4; the round-1 change by pi
    was wrong), with both optical frames rolled by pi (`cam_image_flip`, default true): in both
    wrist clouds the table normal points to the image bottom, i.e. the images are upside down
    w.r.t. the original optical frames. The whole left end effector (gripper + camera) is mounted
    on its flange turned by 180 deg relative to the right one (`rpy_ee` and `cam_holder_yaw`
    3pi/4 in `dfki_fr3_left.yaml`); this turns `franka_left_grasp_point` by pi about z as well. Found by fitting the board plane in each camera's cloud (level to ~3-8°)
    and matching the board seen by both wrist cameras. `cam_holder_yaw` stays as a per-arm arg.
  - Without depth the ZED publishes at 60 Hz; capped at 30 Hz (`general.pub_frame_rate`) because
    the collector encodes 30 fps videos.
  - The Jetson's `fastdds_jetson.xml` names the PC without ports, so new Jetson nodes never
    discover most PC participants (no TF). The FFS node uses `jetson/fastdds_ffs.xml` with the
    ports enumerated (a 160-entry list broke discovery; 80 works).
  - `ros2 run` does not forward SIGTERM (orphaned aggregator); the ros2 daemon spawned inside a
    component's process group kept it "running" after it exited. Both fixed.
- **Open:** 480x320 / 8 iterations takes ~120 ms per camera on the Orin (~1 Hz merged for three
  cameras). Faster engines (4 iterations, 320x224) still have to be built and compared.

#### Camera extrinsics from an ArUco board
- **Problem:** The two wrist clouds do not line up and `zed_middle` has no pose.
- **Solution:** `scripts/python/board_calibration.py` (`pixi run calibrate-board`): bundle
  adjustment of the reprojected marker corners over the board pose, static camera poses, per-wrist
  mount corrections (new `<cam>_camera_mount` link; the `cam_pos_*`/`cam_roll..yaw` args of the
  ZED macro had no effect before: its joint carried a second `<origin>`) and the left base
  correction (`franka_right` defines the world). `--write` stores them in `dfki_fr3_*.yaml` and the
  project's `camera_poses.yaml`. Synthetic test with 8 poses per arm: ~0.1 deg / < 1 mm.
- **Findings:** First single capture: the wrists put the board 59 mm / 7.8 deg apart, and both see
  the flat board 7-10 deg from horizontal, so both mounts are off (not only the left base). Board:
  DICT_4X4_50, 3x4, 56.5 mm markers / 5.5 mm gaps (measured; the stereo estimate was 56.2 / 6.7).
  - `gripper.launch.py` built its own description without the camera args, and its state
    publisher sends the whole arm + camera to `/tf_static` too: two versions of
    `franka_left_zed_*` (holder yaw 3pi/4 from the yaml, -pi/4 default) raced there, so each
    listener got either. That made the earlier left-camera orientation tests unreliable. Now it
    gets the same camera keys (`CAMERA_KEYS`). The board confirms the 3pi/4 mount of the whole
    left end effector (gripper + camera): with it the left camera sees the board 58 mm / ~8 deg
    from the right arm's view; -pi/4 or a flipped image put it 50-85 cm off. (A first reading
    said -pi/4: it compared TF captured during the race, with the two versions mislabeled.)
  - 2026-10-23: the right end effector had been mounted the other way round by someone before
    (that is what the old -pi/4 described). It is mounted like the left one again, so both
    arms use `rpy_ee` and `cam_holder_yaw` 3pi/4; board captures from before are obsolete.
  - `config/fastdds_pc.xml` named the Jetson without ports (= its participants 0..3 only): PC
    processes started after the ZED container came back with a higher id saw no camera topics.
    Its ports 8660..8738 are listed now, like the local ones.

### 2026-10-21

#### Franka Studio: web UI for stack, policies, data collection and cluster
- **Problem:** Running an experiment meant several terminals (robot, cameras, teleop, data
  collection, client) plus rviz/image windows to check what the policy sees; the Oopsie client's
  UI did this for one policy family and one dataset format only.
- **Solution:** `pixi run studio` (`studio/`, config in `config/studio.yaml`, projects in
  `projects/`). Built around a gym-style `SceneEnv` created from the policy client config, so the
  3D viewer (viser) shows exactly the observation a policy gets. Policies, datasets/tasks,
  conversion and training configs are per project. The data collector node is unchanged; the
  studio files its episodes into `<data_root>/<dataset>/<task>/`.
- **Finding:** with Fast DDS, a BEST_EFFORT subscriber received 0 of 50 `RigSnapshot` messages
  (~5 MB) that a RELIABLE subscriber got. franka_client's RobiClient subscribes BEST_EFFORT, so its
  image features may never arrive on this RMW. The studio adapts the reliability to the
  publishers; RobiClient was not changed.

### 2026-09-01

#### pinocchio 4.0.0 broke the build (env drifted from the lock)
- **Problem:** `pixi run build` failed with `fatal error: pinocchio/spatial/se3.hpp: No such file or
  directory` in `franka_control_module`. pinocchio 4.0.0 reorganised the headers: `pinocchio/spatial/`
  now ships only `fwd.hpp`, and the old layout moved under `pinocchio/deprecated/pinocchio/`, which is
  not on the default include path. `crisp_controllers` includes `spatial/se3.hpp` and
  `spatial/explog.hpp` too, so this breaks the whole workspace, not one package.
- **Root cause:** *not* a bad re-solve. `pixi.lock` correctly pinned pinocchio **3.9.0** for the
  `humble` env all along; the on-disk `.pixi/envs/humble` had **drifted** to 4.0.0. The environment
  was out of sync with the lock file.
- **Solution:** `pixi install -e humble` re-syncs the env to the lock and restores 3.9.0 — that is
  what actually fixed the build. If a build was attempted against the drifted env, `pixi run clean`
  too, since the stale CMake caches are sticky. Separately, `pinocchio` was declared `"*"` in
  `pixi.toml`; it is now `">=3,<4"` so that a future `pixi update` cannot pull 4.x back in. That pin
  left `pixi.lock` byte-identical — it is a guard, not the fix. Drop it once the sources are ported
  to the 4.x header layout.

### 2026-08-27

#### `libfranka` + `franka_robot_state_broadcaster` moved to GitHub
- **Problem:** Both lived on IAS GitLab, and their urls were duplicated in two places -
  `config/workspace.yaml` *and* hardcoded in the bespoke `franka_ros2` block of `setup.sh`.
- **Solution:** Repointed to `git@github.com:DFKI-SAIROL/...`. The `franka_ros2` block now looks
  urls up from the manifest via a `spec_url` helper instead of hardcoding them, so the manifest is
  genuinely the only place a url appears. `sync_repo` and the nested-repo loop now compare each
  checkout's `origin` against the manifest and `git remote set-url` when they differ, and submodule
  mode runs `git submodule sync --recursive` - so a future move applies itself on the next
  `pixi run setup` rather than needing a manual fix on every machine.
- **Still private:** anonymous https is denied for both (verified against `crisp_py` as a public
  control), so `visibility` stays `private` in the manifest and no profile is credential-free yet.
- Note for the eventual submodule migration: `libfranka` tracks `main`, `franka_robot_state_broadcaster`
  tracks `master`.

### 2026-08-25

#### `franka_py` is a multi-package repo (corrects an earlier note)
- `franka_client`, `franka_meta_quest`, `zed_rig_aggregator_node`, `franka_control_module` and
  `franka_visualization` are **packages inside `franka_py`**, not separate repositories. So
  `pixi run teleop` and `pixi run client` were never broken, and the VR teleop is already public
  (`franka_py` is a public repo). `franka_meta_quest` was removed from the manifest accordingly.
- Consequence for the manifest: **groups cannot be finer-grained than repositories.** Teleop is not
  separately selectable, it arrives with `franka_py` in the `dfki-core` group.

#### Phase 2: src/ migrated to git submodules
- **Problem:** `src/` was gitignored plain clones, so every `pixi run setup` silently moved everyone
  to whatever was on the default branch, and there was no way to reproduce a known-good checkout.
- **Solution:** the 14 migratable repos are now submodules pinned at committed SHAs. Profile
  selection drives `submodule.active`, so a profile installs exactly its groups.
  `workspace.py verify-submodules` checks `.gitmodules` against the manifest for drift.
- **No migration script.** The conversion was one-shot and is materialised in the committed
  `.gitmodules` plus the `src/` gitlinks, so a clone is already migrated and `setup.sh` only ever
  runs the submodule path. The throwaway `scripts/migrate_to_submodules.sh` and the clone-mode
  fallback were removed rather than kept as dead code with a destructive entry point.
- **Not migrated:** the `franka_ros2` bundle (`franka_ros2`, `libfranka`,
  `franka_robot_state_broadcaster`). The setup deletes 49 tracked files from the upstream tree,
  which a submodule cannot represent; it stays a plain clone under `/src/franka_ros2/` (still
  gitignored) until the DFKI fork lands.
- **Behaviour change:** `pixi run setup` no longer pulls the latest commits - it checks out the
  pinned ones. Use `pixi run update-src` to move submodules to the tip of their tracked branch,
  then either commit that (a deliberate bump) or discard it.
- **Known gap:** narrowing a profile (e.g. `dfki` -> `minimal`) stops tracking the extra
  submodules but does not remove their working trees, so colcon still builds them. Needs
  `git submodule deinit` for the now-inactive paths if profile switching becomes a real workflow.


#### numpy conda/pypi pin conflict blocks `pixi shell -e humble`
- **Problem:** `numpy` was declared as a *pypi* dependency (`feature.extra.pypi-dependencies`), but
  conda packages (ros-humble-desktop, pinocchio, ...) pull numpy in too and the conda pin wins.
  The conda solve pinned `numpy==2.5.2`, which cannot satisfy `crisp-python`'s `numpy<=2.3`, so the
  environment failed to solve at all.
- **Solution:** Moved numpy to the conda side as `[feature.extra.dependencies] numpy = "<=2.3"`,
  matching crisp_py's bound so both solvers agree. `humble`/`jazzy` now resolve numpy 2.3.0;
  `humble-clean` keeps 2.5.2 (it has no crisp-python, so no conflict). Re-pin if crisp_py's numpy
  bound moves.

#### Install profiles: groups + profiles manifest
- **Problem:** `scripts/setup.sh` hardcoded ~16 `sync_repo` calls, so there was one install
  configuration for everyone. Which packages a gripper needed was encoded in a third place
  (`read_overrides.py:DYNAMIXEL_GRIPPER_TYPES`), separate from both the clone list and the runtime
  config.
- **Solution:** Added `config/workspace.yaml` (repos / groups / profiles / gripper_requirements)
  and `scripts/python/workspace.py` to resolve it. `setup.sh` now drives its clone loop from the
  manifest, remembers the profile in `git config --local workspace.profile`, validates the profile
  against `robot_overrides.yaml`, and preflights all URLs with `git ls-remote` before cloning
  anything. Dropped `needs_dynamixel` from `read_overrides.py` — that mapping now lives only in the
  manifest. Backend is still `git clone`; submodules are a later, separate step.

#### Visibility audit
- All DFKI-SAIROL package repos are already **public** except `bspline_controller`. `libfranka` and
  `franka_robot_state_broadcaster` (IAS GitLab) are the only private dependencies in the build path.

#### Dropped ZED aggregator and B-spline controller
- **Problem:** `zed_rig_aggregator_node` was launched from `run_robot.sh` (already commented out)
  but never cloned; `bspline_controller` was cloned but is too specific to support right now.
- **Solution:** Both removed from the workspace. `zed_rig_aggregator_node` stays private and should
  be migrated away from wherever it is still embedded. `bspline_controller` stays private and
  unsupported; `bspline_joint_controller` removed from the `arm_controller` options in
  `robot_overrides.yaml`.

### 2026-09-07
 - **Problem:** remove mock_components/GenericSystem which doesn't work with effort (crisp_controllers/src/cartesian_controller.cpp uses effort command interface to communicate with the robot)
 - **Solution:** add fake_effort_hardware to handle the effort command interface and simulate the bimanual platform together with the real meta quest: https://github.com/DFKI-SAIROL/fake_effort_hardware

### 2026-07-07

#### cleaning/repos
 - add franka_data_collection, franka_custom_msgs
 - clean franka_py

#### Add a condiition to see if the gripper is turned on
  - **Problem:** The gripper is turned on separately, therefore an user might not notice that it's not turned on while launching the whole setup.
  - **Solution** Before launching the whole setup, ping the grippers (if specified).

### 2026-06-23

#### Intermittent build failures (suspected faulty RAM)
- **Problem:** `pixi run build` randomly crashes compiling `crisp_controllers` with `cc1plus: internal compiler error: Segmentation fault`. It is non-deterministic (different files/passes on identical source), so it points to a hardware fault — likely bad RAM, not a code bug.
- **Solution:** No code fix — just re-run `pixi run build` a few times until it succeeds. The RAM should be checked (`memtest86+`, reseat/replace DIMMs) in the future. See backlog above.

#### Stable gripper device names
- **Problem:** The Robotis RH-P12-RN-A grippers enumerate as `/dev/ttyUSB<N>` in kernel order, so the number is not tied to a physical gripper — power-cycling or replugging can swap left/right and break the launch.
- **Solution:** Added udev rules in `src/franka_launch/udev/99-dynamixel.rules` that pin each FTDI adapter's serial to a fixed symlink (`/dev/dynamixel_left`, `/dev/dynamixel_right`). Documented the install steps in the README under Prerequisites (§1.1.3).
