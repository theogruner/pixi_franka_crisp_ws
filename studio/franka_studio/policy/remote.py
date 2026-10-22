"""gRPC client for the async policy server (LeRobot async-inference protocol).

The server unpickles what the client sends, so the classes must be importable on the server under
the module names it expects. ``wire.observation_module`` / ``wire.policy_config_module`` in
studio.yaml name them (the franka policy server aliases ``franka_client.client`` and
``lerobot_utils``). Received action chunks are unpickled with any ``TimedAction`` mapped to the
local class, whatever module the server pickled it from.
"""

from __future__ import annotations

import importlib
import io
import pickle  # nosec - trusted policy server on the lab network
import sys
import threading
import types
from dataclasses import dataclass, field

import numpy as np


@dataclass
class TimedData:
    timestamp: float
    timestep: int

    def get_timestamp(self) -> float:
        return self.timestamp

    def get_timestep(self) -> int:
        return self.timestep


@dataclass
class TimedAction(TimedData):
    action: np.ndarray

    def get_action(self) -> np.ndarray:
        return self.action


@dataclass
class TimedObservation(TimedData):
    observation: dict
    must_go: bool = False

    def get_observation(self) -> dict:
        return self.observation


@dataclass
class RemotePolicyConfig:
    policy_type: str
    pretrained_name_or_path: str
    lerobot_features: dict
    actions_per_chunk: int
    device: str = "cpu"
    fps: float = 30.0
    rename_map: dict = field(default_factory=dict)


def _publish_as(module_name: str, *classes: type) -> None:
    """Makes ``module_name.<Class>`` resolve to our classes, so pickle writes that path."""
    module = sys.modules.get(module_name)
    if module is None and "." not in module_name:  # a real flat module (e.g. lerobot_utils) wins
        try:
            module = importlib.import_module(module_name)
        except ImportError:
            module = None
    if module is None:
        module = types.ModuleType(module_name)
        sys.modules[module_name] = module
        parent, _, child = module_name.rpartition(".")
        if parent and parent not in sys.modules:
            sys.modules[parent] = types.ModuleType(parent)
        if parent:
            setattr(sys.modules[parent], child, module)
    for cls in classes:
        setattr(module, cls.__name__, cls)
        cls.__module__ = module_name


class _ActionUnpickler(pickle.Unpickler):
    LOCAL = {"TimedData": TimedData, "TimedAction": TimedAction, "TimedObservation": TimedObservation}

    def find_class(self, module: str, name: str) -> type:
        if name in self.LOCAL:
            return self.LOCAL[name]
        return super().find_class(module, name)


def configure_wire(observation_module: str, policy_config_module: str) -> None:
    _publish_as(observation_module, TimedData, TimedAction, TimedObservation)
    _publish_as(policy_config_module, RemotePolicyConfig)


class RemotePolicy:
    """Connection to one policy on the policy server."""

    def __init__(self, server_address: str, policy_cfg: dict, fps: float, logger: object) -> None:
        import grpc
        import services_pb2
        import services_pb2_grpc
        from lerobot_utils import grpc_channel_options

        self._pb2 = services_pb2
        self.logger = logger
        self.address = server_address
        self.channel = grpc.insecure_channel(server_address, grpc_channel_options())
        self.stub = services_pb2_grpc.AsyncInferenceStub(self.channel)
        self.config = RemotePolicyConfig(
            policy_type=policy_cfg.get("policy_type"),
            pretrained_name_or_path=policy_cfg.get("pretrained_name_or_path"),
            lerobot_features=policy_cfg.get("lerobot_features"),
            actions_per_chunk=int(policy_cfg.get("actions_per_chunk", 32)),
            device=policy_cfg.get("policy_device", "cuda:0"),
            fps=fps,
        )
        self._closed = threading.Event()

    def connect(self, timeout_s: float = 10.0) -> None:
        """Handshake and send the policy instructions; the server loads the checkpoint."""
        import grpc

        grpc.channel_ready_future(self.channel).result(timeout=timeout_s)
        self.stub.Ready(self._pb2.Empty(), timeout=timeout_s)
        self.stub.SendPolicyInstructions(self._pb2.PolicySetup(data=pickle.dumps(self.config)))

    def send_observation(self, obs: TimedObservation) -> None:
        from lerobot_utils import send_bytes_in_chunks

        payload = pickle.dumps(obs)
        self.stub.SendObservations(send_bytes_in_chunks(
            payload, self._pb2.Observation, log_prefix="[STUDIO] Observation", silent=True))

    def get_actions(self) -> list[TimedAction]:
        """Blocks until the server has a chunk (empty list on an empty answer)."""
        chunk = self.stub.GetActions(self._pb2.Empty())
        if not chunk.data:
            return []
        return _ActionUnpickler(io.BytesIO(chunk.data)).load()

    def close(self) -> None:
        self._closed.set()
        try:
            self.channel.close()
        except Exception:
            pass
