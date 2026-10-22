"""One rclpy context and executor for the whole studio.

Every scene env gets its own node so switching policies can drop all of its subscriptions and
publishers at once. Nodes are added to the shared executor; a removed node is destroyed after a
grace period, because callbacks the executor already took may still be running on it.
"""

from __future__ import annotations

import threading
import time

import rclpy
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger

RETIRE_GRACE_S = 1.0
LATCHED_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST, depth=1)


class RosRuntime:
    """Owns rclpy, the executor thread and the studio's own helper node."""

    def __init__(self, node_name: str = "franka_studio", threads: int = 8) -> None:
        if not rclpy.ok():
            rclpy.init()
        self.executor = MultiThreadedExecutor(num_threads=threads)
        self.node = Node(node_name)
        self.executor.add_node(self.node)
        self._thread = threading.Thread(target=self._spin, daemon=True, name="ros-executor")
        self._thread.start()
        self._trigger_clients: dict[str, object] = {}
        self._param_clients: dict[str, object] = {}

    def _spin(self) -> None:
        try:
            self.executor.spin()
        except Exception as e:  # shutdown races are harmless
            if rclpy.ok():
                self.node.get_logger().error(f"executor stopped: {e}")

    def add_node(self, node: Node) -> None:
        self.executor.add_node(node)

    def retire_node(self, node: Node) -> None:
        """Removes a node from the executor and destroys it once in-flight callbacks are done."""
        self.executor.remove_node(node)

        def _destroy() -> None:
            time.sleep(RETIRE_GRACE_S)
            try:
                node.destroy_node()
            except Exception:
                pass

        threading.Thread(target=_destroy, daemon=True).start()

    # ---- graph queries (component probes) ------------------------------------------------------

    def topic_has_publishers(self, topic: str) -> bool:
        try:
            return self.node.count_publishers(topic) > 0
        except Exception:
            return False

    def service_exists(self, service: str) -> bool:
        try:
            return any(name == service for name, _ in self.node.get_service_names_and_types())
        except Exception:
            return False

    def call_trigger(self, service: str, timeout_s: float = 5.0) -> tuple[bool, str]:
        """Calls a ``std_srvs/Trigger`` service; returns (success, message)."""
        client = self._trigger_clients.get(service)
        if client is None:
            client = self.node.create_client(Trigger, service)
            self._trigger_clients[service] = client
        if not client.wait_for_service(timeout_sec=min(timeout_s, 2.0)):
            return False, f"Service {service} is not available."
        future = client.call_async(Trigger.Request())
        deadline = time.time() + timeout_s
        while not future.done() and time.time() < deadline:
            time.sleep(0.01)
        if not future.done():
            future.cancel()
            return False, f"{service} did not answer within {timeout_s:.0f} s."
        result = future.result()
        return bool(result.success), str(result.message)

    def _wait(self, future: object, timeout_s: float) -> bool:
        deadline = time.time() + timeout_s
        while not future.done() and time.time() < deadline:
            time.sleep(0.01)
        if not future.done():
            future.cancel()
            return False
        return True

    def set_parameters(self, node_name: str, params: dict, timeout_s: float = 3.0) -> tuple[bool, str]:
        """Sets parameters of another node (str, bool, float, int and string lists)."""
        service = f"{node_name.rstrip('/')}/set_parameters"
        client = self._param_clients.get(service)
        if client is None:
            client = self._param_clients[service] = self.node.create_client(SetParameters, service)
        if not client.wait_for_service(timeout_sec=min(timeout_s, 2.0)):
            return False, f"{node_name} is not running."
        req = SetParameters.Request()
        for name, value in params.items():
            v = ParameterValue()
            if isinstance(value, bool):
                v.type, v.bool_value = ParameterType.PARAMETER_BOOL, value
            elif isinstance(value, int):
                v.type, v.integer_value = ParameterType.PARAMETER_INTEGER, value
            elif isinstance(value, float):
                v.type, v.double_value = ParameterType.PARAMETER_DOUBLE, value
            elif isinstance(value, (list, tuple)):
                v.type, v.string_array_value = ParameterType.PARAMETER_STRING_ARRAY, [str(x) for x in value]
            else:
                v.type, v.string_value = ParameterType.PARAMETER_STRING, str(value)
            req.parameters.append(Parameter(name=name, value=v))
        future = client.call_async(req)
        if not self._wait(future, timeout_s):
            return False, f"{service} did not answer within {timeout_s:.0f} s."
        failed = [f"{p.name}: {r.reason or 'rejected'}" for p, r in zip(req.parameters, future.result().results)
                  if not r.successful]
        return (not failed), "; ".join(failed) or "ok"

    def publisher(self, msg_cls: type, topic: str, latched: bool = False) -> object:
        qos = LATCHED_QOS if latched else QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        return self.node.create_publisher(msg_cls, topic, qos)

    def subscribe(self, msg_cls: type, topic: str, callback: object, depth: int = 10) -> object:
        return self.node.create_subscription(msg_cls, topic, callback,
                                             QoSProfile(depth=depth, reliability=ReliabilityPolicy.RELIABLE))

    def shutdown(self) -> None:
        try:
            self.executor.shutdown(timeout_sec=1.0)
            self.node.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()
