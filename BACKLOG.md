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
- [ ] **Joint limits in the IK** — `robot_ik_layer` (DLS, no limit handling) keeps pushing a joint
      past its FR3 limit when the teleop target asks for it; the safety node then holds the
      command 0.2 rad (`max_spring_stretch`) into the limit indefinitely. Seen on the left wrist
      (joint7 2.94 measured, 3.14 commanded) on 2026-10-24.
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

### 2026-10-31

#### Right teleop node died at every start ("Device not found")
- **Problem:** only the left arm followed the Quest. The right `oculus_action_main` exited at
  startup with `Device not found. Make sure that device is running and is connected over USB`;
  the left one, started at the same moment, found the Quest.
- **Root cause:** both nodes look up the Quest at once and the first to need it starts the adb
  server. The other connects while that server is still enumerating USB, gets an empty device
  list and `get_usb_device()` exited on the first try. It used to be hidden by an adb server
  left running from earlier; since teleop is a studio systemd unit, the server it starts lives in
  `franka-studio-teleop.service`'s cgroup and dies with it, so every start raced.
- **Solution:** `get_usb_device()` (`franka_meta_quest/oculus_action/oculus_reader.py`) retries
  for up to 10 s before giving up.
- **Also that day:** the right gripper closed on left commands because the two U2D2 adapters
  had been re-plugged to the opposite grippers (2026-10-30); `99-dynamixel.rules` names them by
  FTDI serial, so the cabling must match it. Check with `ls -l /dev/dynamixel_*` while one
  adapter is unplugged.

### 2026-10-24

#### Left teleop Y (home) aborted at once: wrist against its limit
- **Problem:** Y on the left controller started a joint homing that aborted within 2–20 ms
  (`aborted: the arm does not follow (moved by something else?)`); B on the right worked.
- **Root cause:** the left wrist sat at joint7 = 2.94 rad (FR3 limit 3.016) and the IK target
  asked for more. The IK has no joint limits, so the safety node's spring clamp held the command
  at measured + `max_spring_stretch` (0.2) = 3.14. `JointHome` plans from the commanded joints and
  aborts when measured is more than `MAX_DEVIATION` (also 0.2) off the plan, so a joint pinned at
  the clamp fails on the first tick, i.e. exactly when the arm is stuck and homing is wanted.
- **Solution:** `franka_meta_quest/joint_home.py` allows each joint its commanded-vs-measured gap
  at the start (+0.05 rad) when that exceeds `MAX_DEVIATION`; other joints keep 0.2, and a frozen
  arm is still caught. The abort message now names the joint and the gap. The IK winding a joint
  into its limit is not fixed (see the TODO above).

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
