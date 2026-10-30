"""Slow autonomous arm motions of the studio (camera calibration, object scan): Cartesian targets of
the grasp point through the safety node's ``target_pose`` service (the IK follows), TF lookups, and
the studio's Stop (``/robot_halt``), which aborts at once."""

from __future__ import annotations

import threading
import time

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

ARM_ID = "fr3"
RATE_HZ = 50.0
STILL_M, STILL_DEG = 0.001, 0.2       # flange motion within 0.3 s that counts as still


class Aborted(Exception):
    pass


def angle_deg(R: np.ndarray) -> float:
    return float(np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1))))


def tf_matrix(tf) -> np.ndarray:
    t, q = tf.transform.translation, tf.transform.rotation
    M = np.eye(4)
    M[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    M[:3, 3] = [t.x, t.y, t.z]
    return M


def interp(A: np.ndarray, B: np.ndarray, s: float) -> np.ndarray:
    T = np.eye(4)
    T[:3, 3] = (1 - s) * A[:3, 3] + s * B[:3, 3]
    T[:3, :3] = Slerp([0.0, 1.0], Rotation.from_matrix([A[:3, :3], B[:3, :3]]))(s).as_matrix()
    return T


class ArmMover:
    """Base of the studio's autonomous runs. ``cfg``: speed (m/s), rotation_speed_deg, settle_s."""

    def _init_motion(self, runtime, node_name: str, robots: list[str], cfg: dict) -> None:
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from std_msgs.msg import Bool
        from tf2_ros import Buffer, TransformListener

        from franka_custom_msgs.srv import SetPoseStamped

        self.motion_cfg = cfg
        self.node = Node(node_name)
        runtime.add_node(self.node)
        self.tf = Buffer()
        self._tf_listener = TransformListener(self.tf, self.node)
        self.robots = list(robots)
        self._pose_clients = {robot: self.node.create_client(SetPoseStamped, f"/{robot}/target_pose")
                              for robot in self.robots}
        self._SetPoseStamped = SetPoseStamped
        self.halted = False
        self._stop = threading.Event()
        self.node.create_subscription(
            Bool, "/robot_halt", self._on_halt,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def _on_halt(self, msg) -> None:
        self.halted = bool(msg.data)
        if self.halted:
            self._stop.set()

    def _check(self) -> None:
        if self._stop.is_set() or self.halted:
            raise Aborted()

    def _lookup(self, target: str, source: str, stamp: float | None = None, timeout: float = 0.5):
        from rclpy.duration import Duration
        from rclpy.time import Time
        t = Time() if stamp is None else Time(seconds=stamp)
        try:
            return tf_matrix(self.tf.lookup_transform(target, source, t, timeout=Duration(seconds=timeout)))
        except Exception:
            return None

    def _settle(self) -> None:
        """Waits settle_s, then until every arm's flange is still (as a capture requires), at most 5 s more."""
        t_end = time.time() + float(self.motion_cfg["settle_s"])
        while time.time() < t_end:
            self._check()
            time.sleep(0.05)
        t_end = time.time() + 5.0
        while time.time() < t_end:
            self._check()
            if all(self._still(robot) for robot in self.robots):
                return
            time.sleep(0.1)

    def _still(self, robot: str) -> bool:
        """No flange motion over the last 0.3 s (TF; True when unknown: the capture reports that)."""
        link8 = f"{robot}_{ARM_ID}_link8"
        now = self.node.get_clock().now().nanoseconds * 1e-9
        F0 = self._lookup("world", link8, now - 0.4)
        F1 = self._lookup("world", link8, now - 0.1)
        if F0 is None or F1 is None:
            return True
        D = np.linalg.inv(F0) @ F1
        return np.linalg.norm(D[:3, 3]) < 0.7 * STILL_M and angle_deg(D[:3, :3]) < 0.7 * STILL_DEG

    def _grasp(self, robot: str) -> np.ndarray:
        G = self._lookup("world", f"{robot}_grasp_point")
        if G is None:
            raise RuntimeError(f"no TF world <- {robot}_grasp_point")
        return G

    def _send(self, robot: str, T: np.ndarray) -> None:
        from geometry_msgs.msg import PoseStamped
        client = self._pose_clients[robot]
        if not client.service_is_ready():
            raise RuntimeError(f"/{robot}/target_pose is not available")
        req = self._SetPoseStamped.Request()
        req.pose = PoseStamped()
        req.pose.header.stamp = self.node.get_clock().now().to_msg()
        req.pose.header.frame_id = "world"
        x, y, z, w = Rotation.from_matrix(T[:3, :3]).as_quat()
        req.pose.pose.position.x, req.pose.pose.position.y, req.pose.pose.position.z = map(float, T[:3, 3])
        req.pose.pose.orientation.x, req.pose.pose.orientation.y = float(x), float(y)
        req.pose.pose.orientation.z, req.pose.pose.orientation.w = float(z), float(w)
        client.call_async(req)

    def _move(self, robot: str, target: np.ndarray) -> bool:
        """Slow straight-line move of the grasp point (slerped orientation); True when arrived."""
        cfg = self.motion_cfg
        start = self._grasp(robot)
        dist = float(np.linalg.norm(target[:3, 3] - start[:3, 3]))
        ang = angle_deg(start[:3, :3].T @ target[:3, :3])
        T = max(dist / float(cfg["speed"]), ang / float(cfg["rotation_speed_deg"]), 1.0)
        n = max(int(T * RATE_HZ), 1)
        for k in range(1, n + 1):
            self._check()
            self._send(robot, interp(start, target, k / n))
            time.sleep(1.0 / RATE_HZ)
        t_end = time.time() + 4.0     # the IK lags behind its target
        while time.time() < t_end:
            self._check()
            G = self._grasp(robot)
            if (np.linalg.norm(G[:3, 3] - target[:3, 3]) < 0.01
                    and angle_deg(G[:3, :3].T @ target[:3, :3]) < 2.0):
                return True
            self._send(robot, target)
            time.sleep(0.1)
        return False
