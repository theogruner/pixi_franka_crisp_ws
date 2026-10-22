"""Live correction of one camera's pose in the point cloud node (no robot / node restart).

    pixi run -e humble python scripts/python/ffs_correction.py zed_left_arm x 180   # rotate 180 deg about optical x
    pixi run -e humble python scripts/python/ffs_correction.py zed_left_arm off      # back to the TF pose

Takes the latched /ffs/extrinsics, adds ``rotate`` (in the camera's optical frame) to one camera
and republishes it. The studio sends its own extrinsics again on project load / pose save.
"""

import json
import sys
import time

import numpy as np
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from scipy.spatial.transform import Rotation
from std_msgs.msg import String

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)


def main() -> None:
    cam, axis = sys.argv[1], sys.argv[2]
    rclpy.init()
    node = rclpy.create_node("ffs_correction")
    current: list = []
    node.create_subscription(String, "/ffs/extrinsics", lambda m: current.append(json.loads(m.data)), LATCHED)
    t = time.time()
    while not current and time.time() - t < 5:
        rclpy.spin_once(node, timeout_sec=0.1)
    ext = current[-1] if current else {}
    entry = dict(ext.get(cam) or {})
    if axis == "off":
        entry.pop("rotate", None)
    else:
        x, y, z, w = Rotation.from_rotvec(np.radians(float(sys.argv[3])) * np.eye(3)["xyz".index(axis)]).as_quat()
        entry["rotate"] = [float(w), float(x), float(y), float(z)]
    ext[cam] = entry
    pub = node.create_publisher(String, "/ffs/extrinsics", LATCHED)
    pub.publish(String(data=json.dumps(ext)))
    t = time.time()
    while time.time() - t < 2:  # let the node pick it up
        rclpy.spin_once(node, timeout_sec=0.1)
    print(f"{cam}: {entry}")


if __name__ == "__main__":
    main()
