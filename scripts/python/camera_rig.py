"""Reads config/cameras.yaml for scripts/run_cameras.sh and scripts/run_pointcloud.sh.

    camera_rig.py <cameras.yaml> [rig]        -> shell assignments (eval'd by the script)
    camera_rig.py <cameras.yaml> [rig] --params <out.yaml>   also writes the aggregator params
    camera_rig.py <cameras.yaml> [rig] --pointcloud          adds the point cloud node's config
"""

import base64
import json
import shlex
import sys
from pathlib import Path

import yaml


def zed_mode(jetson: dict) -> str:
    """2nd argument of the Jetson's launch_zed.sh: '', 'depth' or a ZED override file."""
    depth, stereo = jetson.get("depth", True), jetson.get("stereo", False)
    if stereo:
        base = str(jetson.get("zed_overrides", "~/Projects/Fast-FoundationStereo/jetson")).rstrip("/")
        return f"{base}/zed_stereo_depth.yaml" if depth else f"{base}/zed_stereo.yaml"
    return "depth" if depth else ""


def pointcloud_config(cfg: dict, cams: list) -> dict:
    """The ffs_rig_node.py config (see DEFAULTS there) for this rig."""
    pc = dict(cfg.get("pointcloud") or {})
    agg = cfg.get("aggregator", {})
    node_cfg = {"cameras": cams, "topic_prefix": agg.get("topic_prefix", "/zed_multi")}
    for key in ("engine", "rate", "topic", "world_frame", "zmin", "zfar", "crop", "voxel", "max_points",
                "edge_threshold", "extrinsics", "baseline", "max_age", "support", "flip_images"):
        if key in pc:
            node_cfg[key] = pc[key]
    return node_cfg


def main() -> None:
    args = sys.argv[1:]
    params_out = None
    if "--params" in args:
        i = args.index("--params")
        params_out = args[i + 1]
        del args[i:i + 2]
    with_pointcloud = "--pointcloud" in args
    if with_pointcloud:
        args.remove("--pointcloud")
    cfg = yaml.safe_load(Path(args[0]).read_text())
    rig = args[1] if len(args) > 1 and args[1] else cfg["rig"]
    if rig not in cfg["rigs"]:
        sys.exit(f"Unknown rig {rig!r}; known: {', '.join(cfg['rigs'])}")
    cams = cfg["rigs"][rig]
    jetson, agg = cfg.get("jetson", {}), cfg.get("aggregator", {})
    prefix = agg.get("topic_prefix", "/zed_multi").rstrip("/")
    depth = bool(jetson.get("depth", True))
    if params_out:
        info = "depth/camera_info" if depth else "rgb/color/rect/camera_info"
        ros_params = {
            "camera_names": cams,
            "rgb_topics": [f"{prefix}/{c}/rgb/color/rect/image" for c in cams],
            "camera_info_topics": [f"{prefix}/{c}/{info}" for c in cams],
            "publish_topic": agg.get("publish_topic", "/zed_rig/synced_raw_snapshot"),
        }
        # No depth_topics (an empty YAML list has no type for ROS): RGB-only snapshots
        if depth:
            ros_params["depth_topics"] = [f"{prefix}/{c}/depth/depth_registered" for c in cams]
        params = {"zed_rig_aggregator": {"ros__parameters": ros_params}}
        Path(params_out).write_text(yaml.safe_dump(params, sort_keys=False))
    values = {
        "rig": rig,
        "cameras": " ".join(cams),
        "ssh_target": jetson.get("ssh_target", "jetson"),
        "launch_script": jetson.get("launch_script", "~/Projects/ros2_ws/launch_zed.sh"),
        "remote_setup": jetson.get("remote_setup", "~/.bashrc_ros"),
        "depth": "depth" if depth else "",
        "zed_mode": zed_mode(jetson),
    }
    if with_pointcloud:
        pc = cfg.get("pointcloud") or {}
        values["pc_repo"] = pc.get("repo", "~/Projects/Fast-FoundationStereo")
        values["pc_config_b64"] = base64.b64encode(json.dumps(pointcloud_config(cfg, cams)).encode()).decode()
    for key, value in values.items():
        # launch_script / pc_repo keep their ~ for the remote shell
        print(f"{key}={shlex.quote(str(value))}")


if __name__ == "__main__":
    main()
