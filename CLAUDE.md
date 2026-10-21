# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this repository is

A **Pixi-managed ROS 2 meta-workspace** for bimanual Franka FR3 arms built on the CRISP ecosystem
(upstream docs: <https://learnsyslab.github.io/crisp_controllers/>). It contains almost no source
code of its own — it is the orchestration layer that:

1. clones a profile-selected subset of ~18 separate repositories into `src/`
   (`config/workspace.yaml` + `scripts/setup.sh`),
2. builds them with colcon inside a Pixi/RoboStack ROS environment,
3. launches them via thin wrapper scripts in `scripts/`.

Most of `src/` is tracked as **git submodules** pinned to specific commits. The exception is
`/src/franka_ros2/`, which stays gitignored and is cloned by a bespoke block (see below). The real
implementation — controllers, launch files, safety layer, teleop, data collection — lives in those
repos, each with its own git history.

`franka_meta_quest` (VR teleop) is its own repository, so teleop is separately
selectable as the `teleop` group. The old `franka_py` multi-package repo was dropped from
the workspace: `franka_client` is superseded by the policy client, `franka_control_module`
and `zed_rig_aggregator_node` are unused, and `franka_visualization` was not carried over
(recover it from that repo's history if the rviz pose trails are wanted again).

## Commands

Everything must run inside the Pixi ROS environment. Enter it first:

```bash
pixi shell -e humble      # required: `pixi run setup` calls rosdep, which needs ROS on PATH
```

| Task | Command |
| --- | --- |
| Clone/pull `src/` repos for a profile + rosdep + snap deps | `pixi run setup [profile]` |
| List install profiles | `pixi run profiles` |
| Move submodules to branch tips | `pixi run update-src` |
| Sync env after setup | `pixi install -e humble` |
| Build all packages | `pixi run build` |
| Wipe `build/ install/ log/` | `pixi run clean` (or `pixi run clean-build`) |
| Tests (pytest + coverage) | `pixi run test` |
| Launch driver + IK/safety layer (tmux) | `pixi run robot` |
| VR teleoperation (Meta Quest) | `pixi run teleop` |
| Data collection | `pixi run data-collection` |
| Policy client | `pixi run client` |

Single test: `pixi run -e humble pytest path/to/test_file.py::test_name`. For a single ROS
package: `colcon build --symlink-install --packages-select <pkg>` and
`colcon test --packages-select <pkg>`.

`pixi run build` intermittently dies with `cc1plus: internal compiler error: Segmentation fault`
while compiling `crisp_controllers`. This is suspected faulty RAM, not a code bug — just re-run
the build; colcon resumes where it stopped. See `BACKLOG.md`.

## Configuration flow (the thing to understand first)

`config/robot_overrides.yaml` is the **single source of truth** for per-arm settings (IPs, gripper
type/port, end-effector frame, arm controller, `use_fake_hardware`, `bypass_safety`). It overrides
the defaults shipped in `src/franka_launch/config`. It is consumed in two ways:

- **Shell side:** `scripts/python/read_overrides.py` prints the file as `KEY=VALUE` lines; every
  wrapper script does `eval "$(pixi run -e humble python3 scripts/python/read_overrides.py ...)"`
  and forwards the values as `ros2 launch` arguments.
- **ROS side:** the full path is passed through as `overrides_file:=` so the launch files in
  `franka_launch` / `robot_ik_layer` re-read the YAML themselves.

Consequence: adding a new setting means touching the YAML, `read_overrides.py` (only if a wrapper
script needs it), and the launch file that reads `overrides_file`. Gripper/end-effector-frame
values are coupled — see the inline comments in the YAML for the valid combinations.

Simulation is not a separate environment or launch file: set `use_fake_hardware: true` and run
`pixi run robot`. It relies on `fake_effort_hardware` (ROS 2's stock `mock_components/GenericSystem`
cannot serve the effort command interface the CRISP cartesian controller needs). Teleop still
requires physical Quest hardware.

## Install profiles (`config/workspace.yaml`)

`scripts/setup.sh` does not hardcode a repository list. `config/workspace.yaml` declares three
independent levels — `repos:` (url, branch, pin, visibility), `groups:` (bundles; each repo is in
exactly one), and `profiles:` (lists of groups) — plus `gripper_requirements:`, which maps a
`robot_overrides.yaml` `gripper_type` to the group that provides its driver.

`scripts/python/workspace.py` resolves that manifest and is the only thing that reads it:

```bash
python3 scripts/python/workspace.py profiles
python3 scripts/python/workspace.py repos  --profile dfki [--visibility private]
python3 scripts/python/workspace.py specs  --profile dfki   # pipe-separated, for setup.sh
python3 scripts/python/workspace.py check  --profile dfki --overrides config/robot_overrides.yaml
```

`specs` output is **pipe-separated, not tab-separated** — bash treats tab as IFS whitespace and
collapses runs of it, which silently shifts empty fields in `read`.

`load_manifest()` self-checks on every call: unknown repo in a group, repo in two groups, orphaned
repo, unknown group in a profile. Adding a repository therefore means one `repos:` entry plus one
group membership — nothing in `setup.sh`.

Two flags change acquisition behaviour: `special: true` (excluded from submodule management and
handled by the bespoke `franka_ros2` block) and `optional: true` (skipped when unreachable, so an
unpublished repo does not block a whole profile).

**`setup.sh` populates `src/` purely through submodules.** `.gitmodules` is committed, so a clone
already carries every gitlink; setup sets `submodule.active` from the profile and runs
`git submodule update --init --recursive`, checking out exactly the profile's groups. It also sets
the submodule ergonomics (`push.recurseSubmodules=on-demand`, `submodule.recurse`,
`status.submoduleSummary`, `diff.submodule=log`), which are local config and so do not survive a
clone. A missing `.gitmodules` is a hard error, not a fallback to cloning.

`workspace.py verify-submodules` reports drift between `.gitmodules` and the manifest — missing
entries, url drift, entries the manifest does not know about. Run it in CI.

Consequence of pinning: `pixi run setup` installs pinned commits and no longer pulls latest. Use
`pixi run update-src` (`git submodule update --remote --merge`) to move to branch tips, then commit
or discard. Narrowing a profile deactivates submodules but does **not** remove their working trees,
so colcon keeps building them until they are `git submodule deinit`-ed.

`setup.sh` selects a profile from `$1`, else `git config --local workspace.profile`, else `dfki`;
it persists the choice, runs `check` against `robot_overrides.yaml`, then preflights every URL with
`git ls-remote` so all inaccessible repos are reported at once instead of failing one at a time.

Division of authority: **the profile is the install-time decision, `robot_overrides.yaml` is the
runtime one.** They are reconciled by `check`, which errors against real hardware and only warns
when `use_fake_hardware: true`.

## Environment details

- Pixi environments are feature compositions in `pixi.toml`: `humble` (default), `jazzy`,
  `humble-viser`/`jazzy-viser`, `humble-clean`. ROS comes from the RoboStack conda channels, so
  there is no `/opt/ros` — never source a system ROS setup.
- `scripts/set_ros_env.sh` runs on every activation: it prepends `$CONDA_PREFIX/lib` to
  `LD_LIBRARY_PATH`, sources `install/setup.bash` if present, and restarts the `ros2` daemon.
  It sources `scripts/personal_ros_env.sh` if you create one (gitignored, per-machine
  `ROS_DOMAIN_ID` / `RMW_IMPLEMENTATION` overrides); otherwise defaults to domain 5 and
  `rmw_fastrtps_cpp`.
- Most `src/` repos are cloned over **SSH** (`git@github.com:DFKI-SAIROL/...`) even though the
  DFKI-SAIROL repos are public — so `pixi run setup` currently needs an SSH key for repos that
  could be fetched anonymously. Switching those manifest URLs to `https://` is a pending
  one-line-per-repo change. The only genuinely private dependencies are `libfranka` and
  `franka_robot_state_broadcaster`; they moved from IAS GitLab to `DFKI-SAIROL` on GitHub
  (2026-08-27) but are still private, and they gate *every* profile. Check with
  `workspace.py repos --profile minimal --visibility private`.
- `franka_ros2` is pinned to commit `7ed0458` with several subpackages deleted and replaced by
  internal forks. That surgery is a bespoke block in `setup.sh`; the three repos it handles are
  flagged `special: true` in the manifest so the generic clone loop skips them. It goes away when
  the planned DFKI `franka_ros2` fork lands.
- `numpy` is pinned `<=2.3` on the **conda** side (`feature.extra.dependencies`) to match
  `crisp-python`'s bound. Declaring it as a pypi dependency instead makes the environment
  unsolvable: conda pulls numpy in regardless and its pin wins. Re-pin if crisp_py's bound moves.
- Dynamixel grippers must appear as `/dev/dynamixel_left` / `/dev/dynamixel_right` (udev rules in
  `src/franka_launch/udev/`). `run_robot.sh` pings them with `scripts/python/check_gripper.py`
  before launching and aborts if a configured gripper is unpowered.

## Conventions

- Python is linted/formatted with ruff (config in `pyproject.toml`): line length 100, double
  quotes, Google-style docstrings, and `D`/`ANN` rules enabled — public functions need docstrings
  and type annotations. `examples/` is excluded from linting.
- `examples/*.py` are `# %%`-cell scripts against the `crisp_py` API, meant to be run
  interactively; they are not part of the package build.
- `BACKLOG.md` is the running log: open TODOs at the top, dated problem/solution entries below.
  Add an entry there when fixing something non-obvious about the hardware or build.
