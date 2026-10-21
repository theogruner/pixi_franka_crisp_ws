# Porting the bimanual reference workspace

Comparison of this workspace (**ours**, `~/projects/pixi_franka_crisp_ws`, branch `humble`) against
Martin's working bimanual workspace (**ref**, copied to
`~/projects/reference_pixi_ws/pixi_franka_crisp_ws`, branch `lewerkzeugkasten`).

Snapshot taken 2026-10-13. Our side is on branch `dev/bimanual` (see the in-flight section below). Ref content is from 2026-09-24 (`src/`) / 2026-10-02 (top level).
Goal: port what works for bimanual manipulation **into ours**, which stays the maintained one.

## The one thing to understand first

The two workspaces diverged **structurally**, not just in content:

| | ours | ref |
| --- | --- | --- |
| `src/` acquisition | git submodules, pinned, manifest-driven (`config/workspace.yaml` + `scripts/setup.sh`) | loose clones, `vcs`-ish, `scripts/setup.sh` hardcodes the list |
| `franka_py` | **split** into separate repos: `franka_py`, `franka_launch`, `franka_custom_msgs`, `franka_data_collection` | **monorepo** `BuchZn/pix_franka_crips_lwk_fork` @ `feature/crisp` holding all of them + `franka_ui`, `custom_interfaces`, `franka_joint_trajectory_controller` |
| per-arm config | `config/robot_overrides.yaml` is single source of truth, forwarded as `overrides_file:=` | hardcoded `spawn_franka_left:=true spawn_franka_right:=true` in the scripts; per-arm YAML inside `franka_robot_description/config` |
| pixi project name | `pixi_franka_crisp_ws` | `frankapy` (editable `frankapy` + local editable `crisp_py`) |

So **nothing can be copied wholesale.** Every port is: take the ref's *content* change, re-express it
against our repo split and our `robot_overrides.yaml` indirection.

A second trap: a large share of the ref's working bimanual behaviour is **uncommitted** in its
`src/` repos (see the table below). It is not on any branch and would be lost by a plain
`git submodule update`.

## Package inventory

Legend: ✅ nothing to do · ⬆️ ours is ahead · ⬇️ ref is ahead, port candidate · 🆕 ref-only · 🏠 ours-only

| package | ours | ref | verdict |
| --- | --- | --- | --- |
| `crisp_py` | `d8e4431` | `d8e4431` | ✅ byte-identical |
| `crisp_gym` | `ab002ba` | `ab002ba` | ✅ byte-identical |
| `dynamixel_hardware_interface` | `103f061` | `103f061` | ✅ |
| `dynamixel_interfaces` | `9e2ea84` | `9e2ea84` | ✅ |
| `DynamixelSDK` | `0d3403d` | `0d3403d` | ✅ |
| `crisp_controllers` | `c3af67e` | `26b743d` | ⬆️ ours is 3 commits ahead (incl. negative-nullspace-damping fix #56). Ref's origin is `utiasDSL`, ours `DFKI-SAIROL`. |
| `robot_ik_layer` | `726cf69` (`dev/bimanual`) | `b141acd` + **uncommitted** | ⬇️ ours 1 commit ahead, but ref carries +138 lines uncommitted in `franka_ijk.{hpp,cpp}` |
| `robot_safety_layer` | `c5d085d` | `8fbc341` + **uncommitted** | ⬇️ ours 2 commits ahead, but ref carries +261 lines uncommitted in `franka_controller_node.cpp` + `config/safety_params.yaml` |
| `franka_robot_description` | `8eb6e6d` | `b3fb11d` (`lewerkzeugkasten`) + **uncommitted** | ⬇️ unrelated branch; +91/-26 uncommitted across the bimanual xacro, ZED macros, gripper xacro |
| `franka_launch` | own repo, `c079968` (`dev/bimanual`) | inside `franka_py` + **uncommitted** | ⬇️ ours is a file superset; ref has newer `franka.launch.py` / `example.launch.py` / `gripper.launch.py` |
| `franka_py` (client, control_module, meta_quest, visualization, zed_rig_aggregator) | `e0d495f` (`dev/bimanual`) | fork, unrelated history + **uncommitted** | ⬇️ `franka_meta_quest/oculus_action/main.py` +178 lines, teleop_left/right yaml |
| `franka_data_collection` | own repo, `38eb8bf` | inside `franka_py` + **uncommitted** | ⬇️ `data_collector_node.py` rewritten (326 lines changed), new `episode_writer.py`, 3 new configs |
| `franka_ros2` | `7ed0458` (2.3.0) | `652adea` (2.4.0), **48 ahead**; upstream `humble` is now **+100** | ⬇️ version bump; ref also keeps `franka_mobile` (ours skips it in the build) and `franka_description` 2.7.0 vs our 1.6.1 |
| `franka_custom_msgs` | own repo, `b10d264` | inside `franka_py` | ⬇️ compare msg definitions |
| `custom_interfaces` (`RigSnapshot`, `RigSegmentations`) | — | in `franka_py` | 🆕 needed by the ZED rig aggregator path |
| `franka_joint_trajectory_controller` | — | in `franka_py` (source only, **no `package.xml`**) | 🆕 custom JTC with `interpolate_from_desired_state`; ref's `controllers.yaml` already selects it instead of the stock one |
| `franka_ui` | — | in `franka_py` | 🆕 rqt-style process manager GUI |
| `franka_policy` | — | `src/franka_policy`, **not a git repo at all** | 🆕 pi0.5 policy server/client, lerobot-based. Vendored, no history — needs a home repo before it can be a submodule. |
| `zed_remote_launcher` | — | `3c820af` + uncommitted | 🆕 remote ZED bringup on the Jetson, per-project configs (`lwk_2`, `gsplat_policy`, `lieflows`, `calibration`) |
| `fake_effort_hardware` | `ca81b69` | — | 🏠 ours only (sim path) |

## Workspace-level differences

### `pixi.toml`
- ref renames the project to `frankapy`, makes `frankapy` and `src/crisp_py` **editable local** pypi
  deps; ours pulls `crisp-python` from git. Ours is the right call for a meta-workspace — do not port.
- ref drops our `pinocchio = ">=3,<4"` pin (we pin because pinocchio 4 moved headers and breaks
  `crisp_controllers` + `franka_control_module`). **Do not port.**
- ref drops our `numpy <=2.3` conda pin. **Do not port** (see CLAUDE.md).
- ref adds two no-default-feature environments worth having: `lerobot` (policy server, py3.12,
  editable `../lerobot` with `pi`/`async`/`dataset`/`molmoact2` extras) and `viz` (Rerun data viewer).
- ref adds `ros-humble-joint-state-publisher-gui`, `oopsie-data-tools` pypi dep, an `oopsie-stats`
  task, and a `cameras` task (`scripts/view_wrist_cameras.py`).
- ref's `build` task has no `--packages-skip franka_mobile`; ours does.

### `config/`
| file | side | note |
| --- | --- | --- |
| `robot_overrides.yaml`, `workspace.yaml` | ours | our indirection layer; ref has no equivalent |
| `cyclone_config.xml`, `fastdds_config.xml` | ours | |
| `controllers.yaml` | ref | duplicate of the franka_launch one? check before porting |
| `gripper_left.yaml`, `gripper_right.yaml` | ref | per-arm Dynamixel gripper config — **bimanual-relevant** |
| `left_franka_config.rviz` | ref | |
| `fastdds_pc.xml` (+ `.bak-broken-2026-07-06`) | ref | DDS tuning for the PC side |

### `scripts/`
ours: `run_{robot,teleop,client,data_collection}.sh`, `set_ros_env.sh`, `setup.sh`,
`python/{workspace,read_overrides,check_gripper}.py`.

ref adds: `run_policy.sh` (server in `lerobot` env + client in `humble`, gRPC on `:8080`),
`view_wrist_cameras.py`, `apply_patches.sh` + `patches/` (libfranka `mock_server.cpp`,
`controller_manager.cpp`, franka_gripper `gripper.launch.py`), `deploy_cluster_policy.py`,
`copy_checkpoints_from_cluster.sh`, `sync_dagger_to_cluster.sh`, `oopsie_viz.py`,
`compare_gripper_motors.py`, `probe_right_gripper.py`, `fix_right_gripper_mode.py`,
`launch/franka.launch.py`, `udev/`.

ref's `run_robot.sh` is the bimanual entry point and differs substantially from ours:
- `export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-5}"` before the ssh hop ("never ship an empty value over
  ssh — that silently puts the cameras on domain 0")
- `tmux kill-session` first (a stale session makes `new-session` fail while the `split-window`s
  still succeed → half-built layout, no nodes running)
- extra panes: ssh to `jetson` running `launch_zed.sh bimanual`, `zed_rig_aggregator_node`,
  `view_wrist_cameras.py` (delayed 15 s), `tmux select-layout tiled`
- `use_rviz:=false`, `bypass_safety:=true`
- guards `attach-session` when already inside tmux

Ours in exchange has the gripper preflight (`check_gripper.py`) and the whole `overrides_file`
plumbing that ref lacks.

### ref-only, not source
- `docu/docu_ip_project/` — 8 Obsidian notes: Data Pipeline, Project Overview, Gripper Config,
  Action Processor, Dataset Labeling, Safety/IK layer explainer, pi0.5 cluster training+deployment,
  Bugs & Fixes. **Read these before porting anything** — they are the only prose on how the
  bimanual setup is meant to work.
- `logs/`, `frankapy.egg-info/`, `.ruff_cache/`

## In-flight: xiaoqi's `dev/xiaoqi` branches

Separate from the reference port, and reviewed 2026-10-13. There are **five** `dev/xiaoqi`
branches, not four — the fifth is on the superproject itself and behaves completely differently
from the other four.

### The four submodule branches — clean fast-forwards, adopted

`franka_launch` (2 commits), `franka_py`, `robot_ik_layer`, `franka_data_collection` (1 each), all
dated 2026-09-23, ~130 lines, every one a strict fast-forward from our `main`. One coherent
refactor: a boolean override precedence chain `CLI > overrides file > built-in default`
(`load_overrides` + `resolve_bool_override` in `franka_launch/utils/launch_utils.py`), plus
auto-derivation of `end_effector_frame` from `gripper_type` in `merge_overrides`.

Three of them are **adopted**: we sit on `dev/bimanual`, branched off `origin/dev/xiaoqi`, pinned
in `config/workspace.yaml` with the base `rev` recorded. `franka_data_collection` branches off
`main` instead — its only commit hardcodes `/home/xiaoqi/projects/data`.

Two things this settles for the reference port:

- **The config-location question goes our way.** `franka_py`'s commit repoints `robot_config` from
  `franka_robot_description/config` → `franka_launch/config`, the *opposite* direction from the
  reference's move. Two of three parties agree; skip ref's relocation with confidence.
- **It collides with ref's new `stick` gripper type.** The auto-derivation only knows
  `none → fr3_link8` and everything-else → `grasp_point`. `stick` lands in the `grasp_point`
  branch, which is correct — but by luck, not intent. Port them together and handle `stick`
  explicitly.

It also makes part of our shell plumbing redundant: once the launch files read `overrides_file`
themselves, `run_robot.sh` / `run_teleop.sh` no longer need to forward `spawn_franka_*`,
`use_fake_hardware` or `bypass_safety` — only `overrides_file:=`. `read_overrides.py` survives
solely for what the shell genuinely needs (`gripper_type_*`, for the `check_gripper` preflight).

Textual overlap with the reference is almost nil. Only `franka_launch/{example.launch.py,
launch_utils.py}`, and there **the reference is strictly behind us** — ref's `launch_utils.py` has
no `merge_overrides` at all, ref's `example.launch.py` has no `overrides_file`, no
`merge_overrides`, no `gripper_port`. The one thing worth taking from ref in those files is its
`use_rviz` argument, which ref's `run_robot.sh` depends on (`use_rviz:=false`).

### The superproject branch — a divergent reimplementation, NOT merged

`upstream/dev/xiaoqi` (7 commits) left the same merge-base as our own submodule work,
`72bd7f4` (2026-08-27), and built the same feature a second time:

- ours — `a1a8e51` "feat: pin src/ as submodules with manifest-driven install profiles"
- theirs — `a33aa95` "Replace setup clones with pinned submodules"

`humble...upstream/dev/xiaoqi` is **2 ahead, 7 behind** — diverged, not fast-forward. Merging it
would delete `config/workspace.yaml` (-97) and `scripts/python/workspace.py` (-314) in favour of a
flat 20-entry `.gitmodules` with no manifest and no profiles, including the four repos we
deliberately keep out of submodule management (`franka_ros2`, `libfranka`,
`franka_robot_state_broadcaster`, plus `franka_description` and `olvx_descriptions_module`).
This is a design fork, not a conflict git can resolve. **Do not merge.**

Cherry-pick from it instead:

| item | verdict |
| --- | --- |
| `tests/test_end_effector_frames.py` (+73) | **take** — pure addition, tests exactly the `auto` resolution we adopted |
| `config/robot_overrides.yaml` (+11) | **take** — the `auto` frame values that go with it |
| deleting `scripts/python/read_overrides.py` | **take, later** — once the submodule PRs land |
| `scripts/python/check_robot_config.py` (+82) | evaluate — overlaps our `workspace.py check`; one of the two should go |
| `scripts/run_robot.sh` / `run_teleop.sh` | evaluate — probably the same simplification noted above |
| tracking `pixi.lock` (+37362) | decide — ours is gitignored, theirs committed; committed is more defensible for a robot workspace |
| `.gitmodules`, `setup.sh`, deleting `workspace.yaml` / `workspace.py` | **reject** |

One real gap it exposes: their branch carries `src/bspline_controller` as a submodule, and our
`franka_launch/config/controllers.yaml` references `bspline_controller/BsplineJointController` —
but `config/workspace.yaml` declares no such repo. **Our manifest is missing a repository our own
config needs.** Worth fixing independently of this port.

### Merge policy, and what happens when the PRs land

- merge commit or fast-forward → xiaoqi's commits become ancestors of `main`; `git merge
  origin/main` is trivial or a no-op.
- **squash-merge** → the squashed commit has no ancestry link, so a merge re-applies the change and
  conflicts with our copy. Amputate the base instead:
  `git rebase --onto origin/main <rev> dev/bimanual`. Only works while our commits sit strictly on
  top of xiaoqi's — so **never amend, reorder, squash or interleave with them.**
- revised during review (no force-push) → `git rebase origin/dev/xiaoqi`.
- force-pushed → same `--onto` recovery, using the recorded `rev`.
- closed unmerged → we own those commits and ship them in our own PR.

Evidence on which to expect: `franka_launch`'s three prior PRs all landed as merge commits
(`Merge pull request #3 from DFKI-SAIROL/feature/bspline`), so the easy case looks likely where it
matters most. `franka_py` and `robot_ik_layer` have **no** GitHub PR merges in their history — their
four merge commits each are inherited IAS GitLab monorepo merges from
`git.ias.informatik.tu-darmstadt.de`. Assume squash is possible there.

With the reviewer unavailable indefinitely and two incompatible submodule-pinning designs already
open, the cleanest end state is for `dev/bimanual` to *become* the PR — xiaoqi's commits as its
base, their PRs closed as superseded. That is a team-norms call, not a git one.

## Suggested port order

1. **Read `docu/`** (free, highest information density).
2. `franka_robot_description` — bimanual URDF + ZED + gripper xacro. Everything downstream
   depends on the model being right.
3. `franka_launch` — `franka.launch.py` / `gripper.launch.py` / namespace-agnostic
   `dynamixel_controllers.yaml`, re-expressed against `overrides_file`.
4. `robot_safety_layer` + `robot_ik_layer` — the uncommitted C++ (+399 lines total).
5. `franka_joint_trajectory_controller` + the `controllers.yaml` switch (needs a `package.xml`
   written; it has none).
6. `franka_meta_quest` bimanual teleop (+178 lines in `oculus_action/main.py`).
7. `franka_data_collection` rewrite (`episode_writer.py` + reworked `data_collector_node.py`).
8. ZED path: `custom_interfaces`, `zed_remote_launcher`, `zed_rig_aggregator_node` diffs,
   `view_wrist_cameras.py`.
9. `franka_ros2` 2.3.0 → 2.4.0 (48 commits) — independent of the rest, do it in isolation.
10. Policy stack: `franka_policy` + `lerobot`/`viz` pixi envs + `run_policy.sh`. Needs a repo of
    its own first.

## Open questions

- Does `franka_joint_trajectory_controller` replace or complement our `bspline_joint_controller`
  (which ref does not have)? Ref's `controllers.yaml` deleted both our stock
  `joint_trajectory_controller` and `bspline_joint_controller` entries.
- Ref's nullspace weights are halved (100→50 etc.) with the old values commented out. Deliberate
  tuning for bimanual, or a leftover experiment?
- Where should `franka_policy` live — a new `DFKI-SAIROL` repo, or inside `franka_py`?
- Is `patches/` still needed, or has upstream `franka_ros2` 2.4.0 absorbed those three patches?
