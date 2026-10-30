"""Unit tests for the parts of the studio that need no running ROS graph.

    pixi run -e humble pytest studio/tests
"""

import io
import json
import pickle
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from franka_studio.cluster.cluster import Cluster, fill  # noqa: E402
from franka_studio.collection.collector import DataCollection, list_datasets, valid_name  # noqa: E402
from franka_studio.paths import WORKSPACE, add_import_paths  # noqa: E402
from franka_studio.policy import remote  # noqa: E402
from franka_studio.policy.registry import VISION, VISION_LANGUAGE, discover, load_policy  # noqa: E402
from franka_studio.processes import ProcessManager  # noqa: E402


# ---- policies ------------------------------------------------------------------------------------

def _write(path: Path, data: dict) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(data))
    return path


CLIENT = {
    "frequency": 30,
    "mappings": {"observation.state": {"topic": "/js", "msgtype": "sensor_msgs/msg/JointState"}},
    "actions": [],
    "server_address": "127.0.0.1:8080",
    "policy": {"policy_type": "act",
               "pretrained_name_or_path": "/ckpt/run_2026-10-01_12-30-00/checkpoints/040000/pretrained_model"},
}


def test_policy_kind_inferred_and_checkpoint_parsed(tmp_path):
    spec = load_policy(_write(tmp_path / "act.yaml", CLIENT), scene_defaults={"robots": []})
    assert spec.kind == VISION and not spec.error
    assert spec.step == "040000" and spec.trained.isoformat() == "2026-10-01T12:30:00"
    assert spec.config["scene"] == {"robots": []}  # inherited from the project env

    vla = dict(CLIENT, policy={**CLIENT["policy"], "policy_type": "xvla", "task": "Pick"})
    spec = load_policy(_write(tmp_path / "xvla.yaml", vla))
    assert spec.kind == VISION_LANGUAGE and spec.prompts == ["Pick"]


def test_policy_entry_referencing_client_config(tmp_path):
    _write(tmp_path / "configs" / "client.yaml", CLIENT)
    entry = _write(tmp_path / "policies" / "sub" / "entry.yaml", {
        "studio": {"name": "Nice name", "kind": "vision_language", "prompts": ["a", "b"]},
        "client_config": "../../configs/client.yaml",
    })
    specs = discover(tmp_path / "policies")
    spec = specs[next(iter(specs))]
    assert spec.file == entry and spec.name == "Nice name" and spec.language and spec.prompts == ["a", "b"]
    assert spec.config["mappings"] == CLIENT["mappings"]


def test_broken_policy_reports_error(tmp_path):
    spec = load_policy(_write(tmp_path / "x.yaml", {"client_config": "missing.yaml"}))
    assert spec.error


def test_example_project_policies_parse():
    specs = discover(WORKSPACE / "projects" / "example" / "policies")
    assert specs and all(not s.error for s in specs.values())


# ---- wire format ---------------------------------------------------------------------------------

def test_observation_pickles_under_wire_module_and_actions_unpickle_from_anywhere():
    remote.configure_wire("franka_client.client", "studio_test_lerobot_utils")
    obs = remote.TimedObservation(1.0, 3, {"x": np.zeros(2)}, True)
    assert b"franka_client.client" in pickle.dumps(obs)
    assert b"studio_test_lerobot_utils" in pickle.dumps(remote.RemotePolicyConfig("act", "p", {}, 8))

    class TimedAction:  # what a server pickles from its own module
        def __init__(self, ts: int) -> None:
            self.timestamp, self.timestep, self.action = 0.0, ts, np.ones(3)

    TimedAction.__module__ = "some.server.module"
    TimedAction.__qualname__ = "TimedAction"
    sys.modules["some"] = type(sys)("some")
    sys.modules["some.server"] = type(sys)("some.server")
    sys.modules["some.server.module"] = mod = type(sys)("some.server.module")
    mod.TimedAction = TimedAction
    data = pickle.dumps([TimedAction(5)])
    for name in ("some", "some.server", "some.server.module"):
        del sys.modules[name]
    (action,) = remote._ActionUnpickler(io.BytesIO(data)).load()
    assert isinstance(action, remote.TimedAction) and action.get_timestep() == 5


# ---- data collection -------------------------------------------------------------------------------

class FakeCollector:
    """The collector's trigger service: toggles, writes an episode like the real node."""

    def __init__(self, staging: Path) -> None:
        self.staging, self.current = staging, None

    def __call__(self) -> tuple[bool, str]:
        if self.current is None:
            self.current = self.staging / f"episode_{time.time_ns()}"
            (self.current / "mcap").mkdir(parents=True)
            return True, f"Started recording to {self.current}"
        (self.current / "mcap" / "metadata.yaml").write_text("x: 1\n")
        (self.current / "videos").mkdir()
        self.current = None
        return True, "Stopped recording. Encoding 3 frames."


def _wait(pred, timeout=10.0):
    t = time.time()
    while time.time() - t < timeout:
        if pred():
            return True
        time.sleep(0.1)
    return False


def test_episode_is_moved_to_dataset_task_with_metadata(tmp_path):
    dc = DataCollection(FakeCollector(tmp_path / "staging"))
    root = tmp_path / "data"
    assert dc.start(root, "ds", "place-cup", {"kind": "demo", "instruction": "pick"})[0]
    assert not dc.start(root, "ds", "place-cup", {})[0]  # already recording
    assert dc.stop()[0]
    assert _wait(lambda: dc.history[0].status == "saved")
    ep = dc.history[0].target
    assert ep.parent == root / "ds" / "place-cup" and (ep / "mcap" / "metadata.yaml").exists()
    assert json.loads((ep / "studio_episode.json").read_text())["instruction"] == "pick"
    assert list_datasets(root) == {"ds": {"place-cup": 1}}
    assert dc.discard_last()[0] and not ep.exists()
    assert list_datasets(root) == {"ds": {"place-cup": 0}}


def test_discard_waits_for_the_encoder(tmp_path):
    dc = DataCollection(FakeCollector(tmp_path / "staging"))
    dc.start(tmp_path / "data", "ds", "t", {})
    src = dc.current.source
    dc.stop()
    # an "ffmpeg" still writing into the episode keeps it from being touched
    enc = subprocess.Popen(["bash", "-c", f"sleep 2.5 # ffmpeg {src}/videos/rgb.mp4"])
    assert "discarded once" in dc.discard_last()[1]
    time.sleep(1.5)
    assert src.exists()
    enc.wait()
    assert _wait(lambda: dc.history[0].status == "discarded")
    assert not src.exists() and not (tmp_path / "data" / "ds" / "t" / src.name).exists()


def test_names_are_validated():
    assert valid_name("place-cup") and valid_name("v2 data_1.0")
    assert not valid_name("") and not valid_name("../x") and not valid_name("a/b")


# ---- cluster ---------------------------------------------------------------------------------------

def test_templates_fill_known_and_keep_unknown_placeholders():
    vals = Cluster._values("proj", {"host": "h", "remote_raw_root": "/raw/{project}",
                                    "remote_dataset_root": "/ds/{project}"}, "coffee", "place-cup")
    assert vals["remote_raw_dir"] == "/raw/proj/coffee/place-cup"
    assert vals["remote_dataset_dir"] == "/ds/proj/coffee_place-cup"
    assert fill("x {remote_raw_dir} {unknown}", vals) == "x /raw/proj/coffee/place-cup {unknown}"


# ---- processes ---------------------------------------------------------------------------------------

def test_stop_reaches_the_whole_process_group():
    pm = ProcessManager()
    # the child ignores SIGINT like a background job of a script would; the manager escalates
    pm.configure({"demo": {"cmd": "trap '' INT; (trap '' INT; sleep 60) & echo ready; wait"}})
    assert pm.start("demo")[0]
    proc = pm.get("demo")
    assert _wait(lambda: any("ready" in line for _, line in proc.lines_since(0)))
    assert proc.alive
    import franka_studio.processes as processes

    old = processes.STOP_TIMEOUTS_S
    processes.STOP_TIMEOUTS_S = (0.5, 0.5)
    try:
        assert pm.stop("demo")[0]
        assert _wait(lambda: not proc.alive, timeout=8)
    finally:
        processes.STOP_TIMEOUTS_S = old
    assert _wait(lambda: proc.snapshot()["state"] == "stopped")


def test_components_of_an_earlier_studio_are_adopted_and_stoppable(tmp_path):
    """A studio that died without stopping its components: the next one finds them in the registry."""
    import franka_studio.processes as processes

    registry = tmp_path / "components.json"
    spec = {"demo": {"cmd": "echo ready; sleep 60"}}
    first = ProcessManager(registry)
    first.configure(spec)
    assert first.start("demo")[0]
    assert _wait(lambda: any("ready" in line for _, line in first.get("demo").lines_since(0)))
    pgid = json.loads(registry.read_text())["groups"]["demo"]["pgid"]

    second = ProcessManager(registry)   # "restarted studio": same registry, first one forgotten
    second.configure(spec)
    adopted = second.get("demo")
    assert adopted.alive and adopted.snapshot()["state"] == "running"
    assert not second.start("demo")[0]   # no second copy
    old = processes.STOP_TIMEOUTS_S
    processes.STOP_TIMEOUTS_S = (0.5, 0.5)
    try:
        assert second.stop("demo")[0]
        assert _wait(lambda: not adopted.alive, timeout=8)
    finally:
        processes.STOP_TIMEOUTS_S = old
    assert _wait(lambda: "demo" not in json.loads(registry.read_text())["groups"])
    third = ProcessManager(registry)
    third.configure(spec)
    assert not third.get("demo").alive
    with pytest.raises(ProcessLookupError):
        import os

        os.killpg(pgid, 0)


def test_unconfigured_component_refuses_to_start():
    pm = ProcessManager()
    pm.configure({"cams": {"cmd": None}})
    ok, msg = pm.start("cams")
    assert not ok and "no command" in msg


# ---- registry (needs franka_client on the path) ---------------------------------------------------------

def test_registry_resolves_type_spellings():
    pytest.importorskip("rclpy")
    from franka_studio.ros import registry

    add_import_paths(["src/franka_py/franka_client/franka_client"])
    registry.load()
    # franka_client registers the image extractor as "sensor_msgs/Image"
    assert registry.extractor_class("sensor_msgs/msg/Image") is registry.extractor_class("sensor_msgs/Image")
    assert registry.extractor_class("franka_custom_msgs/msg/RigSnapshot") is not None
    assert registry.processor_class("GripperAbsolute") is not None


# ---- camera placement -------------------------------------------------------------------------------

def test_placed_camera_pose_is_saved_and_merged_on_load(tmp_path):
    from franka_studio.projects import load_project, save_camera_pose

    root = tmp_path / "proj"
    _write(root / "project.yaml", {"name": "P"})
    _write(root / "env.yaml", {"scene": {"cameras": {"wrist": {"link": "cam_link"}}}})
    project = load_project(root)
    save_camera_pose(project, "zed_middle", [1.0, 0.0, 0.7], [0.0, 0.7071, 0.0, 0.7071])
    save_camera_pose(project, "wrist", [9.0, 9.0, 9.0], [1.0, 0.0, 0.0, 0.0])  # has a link: ignored on load
    cams = load_project(root).env_config["scene"]["cameras"]
    assert cams["zed_middle"]["position"] == [1.0, 0.0, 0.7]
    assert cams["wrist"] == {"link": "cam_link"}


def test_look_at_points_the_optical_axis_at_the_target():
    pytest.importorskip("viser")
    from scipy.spatial.transform import Rotation

    from franka_studio.viz.scene_viewer import look_at_wxyz

    w, x, y, z = look_at_wxyz((1.0, 0.0, 1.0), (0.0, 0.0, 0.0))
    R = Rotation.from_quat([x, y, z, w]).as_matrix()
    assert np.allclose(R[:, 2], np.array([-1.0, 0.0, -1.0]) / np.sqrt(2), atol=1e-6)  # z looks at target
    assert R[2, 1] < 0  # image "down" (y) points down in the world


# ---- observation space ---------------------------------------------------------------------------

ENV_WITH_MODALITIES = {
    "mappings": {
        "observation.state": {"topic": "/js", "msgtype": "sensor_msgs/msg/JointState"},
        "observation.images.cam": {"topic": "/img", "msgtype": "sensor_msgs/msg/Image"},
        "observation.pointcloud": {"topic": "/ffs/points", "msgtype": "sensor_msgs/msg/PointCloud2"},
    },
    "scene": {"point_clouds": [{"name": "merged", "topic": "/ffs/points"}, {"name": "other", "topic": "/x"}]},
    "modalities": {
        "state": {"keys": ["observation.state"], "required": True},
        "rgb": {"keys": ["observation.images.*"]},
        "pointcloud": {"keys": ["observation.pointcloud"], "point_clouds": ["merged"],
                       "record": ["/ffs/points"], "component": "pointcloud", "default": False},
    },
}


def test_modalities_filter_mappings_scene_and_recording():
    from franka_studio import modalities as mods

    ms = mods.load_modalities(ENV_WITH_MODALITIES)
    assert mods.resolve_enabled(ms, None) == {"state", "rgb"}           # defaults
    assert mods.resolve_enabled(ms, ["pointcloud"]) == {"state", "pointcloud"}  # required stays
    off = mods.apply(ENV_WITH_MODALITIES, ms, {"state", "rgb"})
    assert set(off["mappings"]) == {"observation.state", "observation.images.cam"}
    assert [c["name"] for c in off["scene"]["point_clouds"]] == ["other"]
    assert mods.record_topics(ms, {"state", "rgb"}) == []
    on = mods.apply(ENV_WITH_MODALITIES, ms, {"state", "rgb", "pointcloud"})
    assert "observation.pointcloud" in on["mappings"] and len(on["scene"]["point_clouds"]) == 2
    assert mods.record_topics(ms, {"state", "pointcloud"}) == ["/ffs/points"]
    assert "observation.pointcloud" in ENV_WITH_MODALITIES["mappings"]  # the project config is untouched


def test_camera_extrinsics_for_the_point_cloud_node():
    from franka_studio.pointcloud import camera_extrinsics

    ext = camera_extrinsics({"wrist": {"robot": "r", "link": "r_zed_left_camera_optical_frame"},
                             "middle": {"position": [1, 0, 0.7], "wxyz": [0, 1, 0, 0]}, "unknown": {}})
    assert ext == {"wrist": {"frame": "r_zed_left_camera_optical_frame"},
                   "middle": {"position": [1.0, 0.0, 0.7], "wxyz": [0.0, 1.0, 0.0, 0.0]}}


def _cloud_msg(xyz: np.ndarray, rgb: np.ndarray | None = None) -> object:
    from array import array

    from sensor_msgs.msg import PointCloud2, PointField

    cols = 4 if rgb is not None else 3
    data = np.zeros((len(xyz), cols), np.float32)
    data[:, :3] = xyz
    names = ["x", "y", "z"] + (["rgb"] if rgb is not None else [])
    if rgb is not None:
        rgb = rgb.astype(np.uint32)
        data[:, 3] = ((rgb[:, 0] << 16) | (rgb[:, 1] << 8) | rgb[:, 2]).view(np.float32)
    msg = PointCloud2()
    msg.height, msg.width, msg.point_step, msg.row_step = 1, len(xyz), 4 * cols, 4 * cols * len(xyz)
    msg.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1) for i, n in enumerate(names)]
    msg.data = array("B", data.tobytes())
    return msg


@pytest.mark.parametrize("sampling", ["random", "voxel", "fps"])
def test_point_cloud_extractor_fixed_size_crop_and_colour(sampling):
    pytest.importorskip("rclpy")
    from franka_studio.ros import registry

    add_import_paths(["src/franka_py/franka_client/franka_client"])
    registry.load()
    cls = registry.extractor_class("sensor_msgs/msg/PointCloud2")
    rng = np.random.default_rng(0)
    xyz = rng.uniform(-1, 1, (5000, 3)).astype(np.float32)
    rgb = np.tile(np.array([255, 128, 0]), (5000, 1))
    ext = cls(num_points=512, fields="xyzrgb", sampling=sampling, crop={"min": [0, 0, 0], "max": [1, 1, 1]})
    out = ext.extract(_cloud_msg(xyz, rgb))
    assert out.shape == ext.shape == (512, 6) and out.dtype == np.float32
    assert (out[:, :3] >= 0).all() and (out[:, :3] <= 1).all()
    assert np.allclose(out[0, 3:], [1.0, 128 / 255, 0.0])
    few = cls(num_points=64).extract(_cloud_msg(xyz[:10]))        # padded by repetition
    assert few.shape == (64, 3) and len(np.unique(few, axis=0)) == 10
    assert not cls(num_points=8).extract(_cloud_msg(np.zeros((0, 3), np.float32))).any()


def test_camera_rig_modes_and_rgb_only_aggregator_params(tmp_path):
    cfg = yaml.safe_load((WORKSPACE / "config" / "cameras.yaml").read_text())
    sys.path.insert(0, str(WORKSPACE / "scripts" / "python"))
    import camera_rig

    assert camera_rig.zed_mode({"depth": True}) == "depth"
    assert camera_rig.zed_mode({"depth": False}) == ""
    assert camera_rig.zed_mode({"depth": False, "stereo": True, "zed_overrides": "~/x"}) == "~/x/zed_stereo.yaml"
    assert camera_rig.zed_mode({"depth": True, "stereo": True, "zed_overrides": "~/x/"}) == "~/x/zed_stereo_depth.yaml"
    node = camera_rig.pointcloud_config(cfg, ["a", "b"])
    assert node["cameras"] == ["a", "b"] and "extrinsics" in node
    pc = {"host": "pc", "engine": "jetson.engine", "rate": 5.0, "pc": {"engine": "pc.engine", "rate": 20.0}}
    assert camera_rig.pointcloud_config({"pointcloud": pc}, ["a"])["engine"] == "pc.engine"
    assert camera_rig.pointcloud_config({"pointcloud": pc}, ["a"])["rate"] == 20.0
    jetson = camera_rig.pointcloud_config({"pointcloud": {**pc, "host": "jetson"}}, ["a"])
    assert jetson["engine"] == "jetson.engine" and jetson["rate"] == 5.0
    cfg["jetson"]["depth"] = False
    path = tmp_path / "cams.yaml"
    path.write_text(yaml.safe_dump(cfg))
    out = tmp_path / "params.yaml"
    subprocess.run([sys.executable, str(WORKSPACE / "scripts/python/camera_rig.py"), str(path), "", "--params", str(out)],
                   check=True, capture_output=True)
    params = yaml.safe_load(out.read_text())["zed_rig_aggregator"]["ros__parameters"]
    assert "depth_topics" not in params  # an empty list would have no type for ROS
    assert all(t.endswith("rgb/color/rect/camera_info") for t in params["camera_info_topics"])


# ---- object pose tracking ------------------------------------------------------------------------------

def test_object_specs_scene_poses_and_modality_hiding(tmp_path):
    pytest.importorskip("rclpy")
    from franka_studio import modalities as mods
    from franka_studio.pose_tracking import object_specs, with_object_poses

    (tmp_path / "objects" / "mug").mkdir(parents=True)
    (tmp_path / "objects" / "mug" / "mesh.ply").write_text("ply")
    specs = object_specs(["mug", {"name": "box", "prompt": "cardboard box", "scale": 0.001}], tmp_path)
    assert specs[0] == {"name": "mug", "prompt": "mug", "mesh": str(tmp_path / "objects/mug/mesh.ply"), "scale": 1.0}
    assert specs[1]["mesh"] is None and specs[1]["prompt"] == "cardboard box"   # not scanned yet
    env = {"scene": {"poses": [{"name": "right_ee", "topic": "/ee"}]},
           "modalities": {"object_poses": {"keys": ["observation.object_poses"], "poses": ["object/*"],
                                           "default": False}}}
    env["mappings"] = {"observation.object_poses": {"topic": "/pose_tracking/poses", "n_poses": 1}}
    cfg = with_object_poses(env, specs)
    assert cfg["mappings"]["observation.object_poses"]["n_poses"] == 2 and env["mappings"]["observation.object_poses"]["n_poses"] == 1
    assert [p["name"] for p in cfg["scene"]["poses"]] == ["right_ee", "object/mug", "object/box"]
    assert cfg["scene"]["poses"][2]["field"] == "poses.1" and env["scene"]["poses"] == [{"name": "right_ee", "topic": "/ee"}]
    ms = mods.load_modalities(env)
    assert [p["name"] for p in mods.apply(cfg, ms, set())["scene"]["poses"]] == ["right_ee"]
    assert len(mods.apply(cfg, ms, {"object_poses"})["scene"]["poses"]) == 3


def test_object_list_file_replaces_env_yaml(tmp_path):
    pytest.importorskip("rclpy")
    from franka_studio.pose_tracking import clean_objects, object_list, save_object_list

    env = {"objects": ["mug"]}
    assert object_list(env, tmp_path) == ["mug"]
    cleaned, err = clean_objects([{"prompt": "white mug"}, {"name": "box", "prompt": ""}, {"prompt": " "}, "cup"])
    assert not err and cleaned == [{"name": "white_mug", "prompt": "white mug"}, {"name": "box", "prompt": "box"},
                                   {"name": "cup", "prompt": "cup"}]
    assert clean_objects(["cup", {"name": "cup", "prompt": "red cup"}])[1]
    assert clean_objects([{"name": "../x", "prompt": "x"}])[1]
    save_object_list(tmp_path, cleaned)
    assert object_list(env, tmp_path) == cleaned


def test_object_scan_views_look_at_the_objects():
    pytest.importorskip("rclpy")
    from franka_studio.config import DEFAULTS
    from franka_studio.motion import angle_deg
    from franka_studio.object_scan import look_at, plan_views

    cfg = DEFAULTS["object_scan"]
    centre = np.array([0.5, 0.0, 0.05])
    C0 = look_at(np.array([0.5, -0.3, 0.35]), centre, np.array([1.0, 0, 0]))
    G0 = C0 @ np.array([[1, 0, 0, 0.0], [0, 1, 0, -0.06], [0, 0, 1, 0.12], [0, 0, 0, 1]])   # grasp point ahead of the camera
    targets, skipped = plan_views(C0, G0, centre, cfg)
    assert 4 <= len(targets) <= cfg["views"]
    CG = np.linalg.inv(C0) @ G0
    for G in targets:
        C = G @ np.linalg.inv(CG)
        to_centre = centre - C[:3, 3]
        assert C[:3, 2] @ to_centre / np.linalg.norm(to_centre) > 1 - 1e-9   # looks at it
        assert angle_deg(G0[:3, :3].T @ G[:3, :3]) <= cfg["max_rotation_deg"] + 1e-6
        assert np.linalg.norm(G[:3, 3] - G0[:3, 3]) <= cfg["max_offset"] + 1e-9
        assert G[2, 3] - cfg["support_z"] >= cfg["min_clearance"]
        assert cfg["min_distance"] - 1e-9 <= np.linalg.norm(to_centre) <= cfg["max_distance"] + 1e-9


def test_scene_pose_from_a_pose_array_item():
    pytest.importorskip("rclpy")
    from geometry_msgs.msg import Pose, PoseArray

    from franka_studio.env.scene import get_path, to_pose

    msg = PoseArray(poses=[Pose(), Pose()])
    msg.poses[1].position.x, msg.poses[1].orientation.w = 0.5, 1.0
    pose = to_pose(get_path(msg, "poses.1"), 0.0)
    assert pose.position[0] == 0.5 and pose.wxyz[0] == 1.0
    with pytest.raises(IndexError):
        get_path(msg, "poses.2")


def test_pose_array_extractor_fixed_size():
    pytest.importorskip("rclpy")
    from geometry_msgs.msg import Pose, PoseArray

    from franka_studio.ros import registry

    add_import_paths(["src/franka_py/franka_client/franka_client"])
    registry.load()
    cls = registry.extractor_class("geometry_msgs/msg/PoseArray")
    p = Pose()
    p.position.x, p.position.y, p.position.z, p.orientation.w = 1.0, 2.0, 3.0, 1.0
    ext = cls(n_poses=2, fields=["position", "orientation"], rot_format="ee6d")
    out = ext.extract(PoseArray(poses=[p]))   # second object missing -> zeros
    assert out.shape == ext.shape == (18,) and out.dtype == np.float32
    assert np.allclose(out[:9], [1, 2, 3, 1, 0, 0, 0, 1, 0]) and not out[9:].any()
