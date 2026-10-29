# Franka Studio

One web UI for the whole workflow: start the stack, load and run policies, collect demos and
DAgger corrections, ship data to the cluster, and see in 3D exactly what a policy is fed.

```bash
pixi run studio            # → http://localhost:8090   (3D viewer on :8091, embedded)
pixi run studio-debug      # same, with verbose logs (every HTTP request)
pixi run studio-test       # unit tests (no ROS graph needed)
```

Options: `python -m franka_studio --host 0.0.0.0` (reachable from the network, set
`viewer.host` too), `--no-viewer`, `--keep-components` (leave the stack running on exit),
`--config other.yaml`, `--debug` (verbose logs incl. every HTTP request; by default only
warnings and errors reach the terminal). Append `?theme=light` or `?theme=dark` to the URL to force a theme; the
toggle in the top bar is remembered per browser.

## What it does

| Widget | |
| --- | --- |
| **Stack** | Start/stop robot, cameras, teleop and the data collector (any command from `config/studio.yaml`), with live logs. Components started elsewhere are detected through a ROS topic/service probe. |
| **Policy** | Lists the project's `policies/` folder (refresh rescans it), filter by name and by *Vision* / *Vision + Language*. Load → Run (prompt + suggestions for V+L policies) → Stop, Home, Unload. Queue fill, control rate and chunk rate are shown live. Space stops a running policy. |
| **Data collection** | Dataset + task (the last ones are preselected), instruction, operator, one record button, **Discard last**. Recording is allowed whenever no policy is *running*; with a policy loaded the episode is tagged as **DAgger** (policy, checkpoint and prompt go into its metadata). |
| **Cluster** | *Copy* (rsync `<dataset>/<task>` to the cluster) or *Copy + convert* (then runs the project's conversion command, e.g. `convert_batched.py` + the quantile fix via sbatch). *Start training* uploads a config from `train_configs/` and runs `train_cmd`. SLURM job ids are tracked. |
| **Scene / Observation** | The active env, live: robot meshes from `robot_description` posed by the joint states, URDF frames, poses (targets, EE), point clouds, the observation images as camera frustums, and the exact images/state the policy gets (crops and resizes included). No rviz or image windows needed. |

## Architecture

```
browser ──HTTP/WS──► server/app.py ──► studio.py (all state & actions)
   │                                      ├─ processes.py      components (process groups, logs)
   └─iframe──► viz/scene_viewer.py        ├─ projects.py       project.yaml, policies/, configs
               (viser, renders the        ├─ env/scene_env.py  SceneEnv: the gym view of the scene
                active env's records)     ├─ policy/runner.py  PolicyRunner: chunked control loop
                                          ├─ collection/       recording on top of the collector
                                          └─ cluster/          rsync / ssh / sbatch
```

**`SceneEnv` is the centre.** It is built from a policy client config — the same `mappings`
(observations, through franka_client's message extractors) and `actions` (franka_client's action
processors) a policy uses — so:

* `env.observe(prompt)` is literally what the policy server is sent,
* `env.step(action)` is what is done with its output,
* `env.reset()` sends the `home_pose`, `env.render()` returns the scene info (`scene:` section).

Every `step` (and a passive `tick` while nothing acts) is broadcast as a `StepRecord`. The viewer
and the page's camera tiles only consume those records, so they show what the policy sees — or,
with no policy loaded, the project env (`env.yaml`), i.e. what is being recorded. The interface
follows gymnasium (`reset/step/render/close`, `observation_space`, `action_space`); there is no
reward or termination on the real robot.

The policy runner keeps franka_client's RobiClient semantics (one action per tick, new
observation once `chunk_size_threshold` actions are left, newer chunks replace queued steps) and
speaks the same gRPC/pickle protocol; `wire:` in studio.yaml names the module paths the server
expects to unpickle.

Subscriptions start BEST_EFFORT and switch to RELIABLE while no message arrives and every
publisher offers it: with Fast DDS a best-effort reader practically never completes a ~5 MB
`RigSnapshot`. A mapping can pin it with `qos: reliable` / `qos: best_effort`.

## Projects

```
projects/<name>/
    project.yaml      name, data_root, env, tasks, cluster, components
    env.yaml          scene env while no policy is loaded (mappings + scene)
    policies/         one yaml per policy, sub-folders allowed
    conversion/       conversion configs offered for "Copy + convert"
    train_configs/    training configs offered for "Start training"
```

Start from `projects/example/`. A policy file is either a full client config or an entry that
reuses one unchanged:

```yaml
studio:
  name: X-VLA · place cup · EE abs
  kind: vision_language        # or vision; inferred from the policy type when missing
  prompts: [Pick up the cup and place it under the coffee machine.]
client_config: ../../client_configs/robi_client_place_cup.yaml
```

Policies without a `scene:` section inherit the project's. See `env/scene.py` for the `scene`
format (robots, frames, poses with optional meshes, point clouds, cameras from a URDF link or a
fixed pose).

### Datasets

Episodes end up in `<data_root>/<dataset>/<task>/episode_<ts>/` with a `studio_episode.json`
(instruction, operator, project, demo/DAgger, policy). The data collector itself is unchanged: it
records into its own `storage_path`, the studio moves each episode once the bag is closed and no
`ffmpeg` writes into it any more. Keep `storage_path` on the same disk as `data_root` so this is a
rename. Discard deletes the last episode wherever it is in that pipeline.

### Observation space and point clouds

`modalities` in `env.yaml` lists what the observation can contain (robot state, RGB cameras, the
point cloud, …) with their mapping keys, scene clouds, extra recorded topics and the component
that produces them. The *Observation space* card switches them per project (remembered): an off
modality is dropped from the project env (observation, `observation_space`, scene) and from the
recording; switching the point cloud on starts the *Point cloud* component (Fast-FoundationStereo
on the Jetson, `config/cameras.yaml` `pointcloud`) and shows its rate and per-camera status.
Recording sets the data collector's `extra_topics` parameter to the enabled modalities' `record`
topics (e.g. `/ffs/points` into the episode's bag). Policies keep their own mappings.

`observation.pointcloud` uses franka_client's `sensor_msgs/msg/PointCloud2` extractor:
`num_points`, `fields: xyz|xyzrgb`, `sampling: voxel|random|fps`, `crop`. The viewer draws the full
merged cloud and, larger and on top, the sampled cloud the policy gets.

### Cameras without a URDF link

Static cameras (e.g. `zed_middle`) have no pose in the robot description. The viewer draws them
in amber at a guessed pose; open *Camera placement*, tick *Move with gizmo*, drag until the
frustum's image lines up with the scene (with the point cloud on, the camera's points follow the
gizmo live), then *Align to point clouds* (ICP on the Jetson against the wrist cameras' clouds;
repeat to refine) and press *Save pose to project*. The pose is stored in
`projects/<name>/camera_poses.yaml` and merged into `scene.cameras` on load (a fixed `position`
in `env.yaml` works too). *Camera image size* in *Display* scales the frustums; the page's
camera strips have their own size slider.

### Stop / Resume

**Stop** (top bar, or Space while not typing) publishes the latched `/robot_halt`: both safety nodes
(`robot_safety_layer`) freeze their joint command and ignore every input -- teleop, policy, joint
homing, the calibration, scripts -- and the studio stops its own runner, calibration and homing.
**Resume** releases it; the safety nodes first re-target the IK at the held pose, and the teleop has
kept re-anchoring meanwhile, so nothing drives back to an old target. *Run policy* resumes by itself,
*Home* and *Calibrate* refuse while stopped. The halt stays in the safety nodes if the studio dies.

### Camera calibration

The *Camera calibration* card calibrates every camera that sees the ArUco board (settings:
`calibration` in `config/studio.yaml`, defaults in `config.py`), wrist cameras on unknown holders
included. Switch on *board* to see which cameras detect it, bring each arm (teleop) to a pose where
its wrist camera sees the board, check the arms and press *Calibrate*. Per arm, one after the other:
three small probe tilts about the grasp point, a first hand-eye estimate, then 8 planned views on a
cap above the board (slow Cartesian targets through `target_pose`, within 20 cm / 45 deg of the start
and 8 cm above the board), a capture at every still pose and back to the start. The solve is
`scripts/python/board_calibration.py`'s bundle adjustment over all captures (mounts, the left base
relative to the right, static cameras). *Save* stores the result by name in
`config/calibrations/<name>.yaml` (captures next to it); *Apply* writes a saved one into
`src/franka_launch/config/dfki_fr3_<side>.yaml` and the project's `camera_poses.yaml` -- then
*Restart robots* to load the new description.

## Notes

* `pixi run robot` runs its launches in tmux; the studio sets `ROBOT_NO_TMUX=1` so they stay in
  its process group (output streamed, one Stop for everything).
* The cameras component defaults to the ZED rig aggregator; prepend your ZED launch (e.g.
  zed_remote_launcher) in `config/studio.yaml` if the cameras are not started elsewhere.
* The 3D viewer needs WebGL; it stays dark in the light theme by design.
