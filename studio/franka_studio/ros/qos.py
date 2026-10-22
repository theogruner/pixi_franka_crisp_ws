"""Subscriptions whose reliability follows the publishers', shared per topic.

A BEST_EFFORT subscriber is compatible with every publisher, but with Fast DDS large messages
(e.g. a ~5 MB RigSnapshot) are fragmented and a best-effort reader practically never completes
one. A RELIABLE reader gets them, but is incompatible with best-effort publishers. So each topic
starts best-effort and, while no message arrives, switches to RELIABLE once every publisher on
the topic offers it (and back if that changes). ``mode`` "reliable"/"best_effort" pins it.

All users of one topic share one subscription: rclpy deserializes every message in Python, which
for camera snapshots costs real CPU per subscription (three image features and the scene's rig
view of the same snapshot topic would otherwise decode each ~5 MB message four times).
"""

from __future__ import annotations

import logging
import time
from typing import Callable

from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

log = logging.getLogger(__name__)

BEST_EFFORT_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE,
                             history=HistoryPolicy.KEEP_LAST, depth=5)
RELIABLE_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE,
                          history=HistoryPolicy.KEEP_LAST, depth=1)
SILENT_AFTER_S = 2.0


class AdaptiveSubscription:
    def __init__(self, node: Node, msg_cls: type, topic: str, callback_group: object = None,
                 mode: str = "auto") -> None:
        self.node, self.msg_cls, self.topic = node, msg_cls, topic
        self.group, self.mode = callback_group, mode
        self.callbacks: list[Callable] = []
        self.reliable = mode == "reliable"
        self.last_msg = 0.0
        self._sub = None
        self._create()

    def _on_msg(self, msg: object) -> None:
        self.last_msg = time.time()
        for cb in self.callbacks:
            try:
                cb(msg)
            except Exception as e:  # one consumer must not starve the others
                log.warning("%s: callback failed: %s", self.topic, e)

    def _create(self) -> None:
        if self._sub is not None:
            self.node.destroy_subscription(self._sub)
        self._sub = self.node.create_subscription(
            self.msg_cls, self.topic, self._on_msg, RELIABLE_QOS if self.reliable else BEST_EFFORT_QOS,
            callback_group=self.group)

    def check(self) -> None:
        if self.mode != "auto" or time.time() - self.last_msg < SILENT_AFTER_S:
            return
        try:
            infos = self.node.get_publishers_info_by_topic(self.topic)
        except Exception:
            return
        if not infos:
            return
        want = all(i.qos_profile.reliability == ReliabilityPolicy.RELIABLE for i in infos)
        if want != self.reliable:
            self.reliable = want
            self._create()


class AdaptiveSubscriptions:
    """Adaptive subscriptions on one node, one per (topic, type), re-checked periodically."""

    def __init__(self, node: Node, callback_group: object = None, period_s: float = 1.0) -> None:
        self.node, self.group = node, callback_group
        self.subs: dict[tuple, AdaptiveSubscription] = {}
        node.create_timer(period_s, self._check, callback_group=callback_group)

    def add(self, msg_cls: type, topic: str, callback: Callable, mode: str = "auto") -> AdaptiveSubscription:
        key = (topic, msg_cls)
        sub = self.subs.get(key)
        if sub is None:
            sub = self.subs[key] = AdaptiveSubscription(self.node, msg_cls, topic, self.group, mode)
        elif mode != "auto" and sub.mode == "auto":  # an explicit choice wins over auto
            sub.mode, sub.reliable = mode, mode == "reliable"
            sub._create()
        sub.callbacks.append(callback)
        return sub

    def _check(self) -> None:
        for sub in list(self.subs.values()):
            sub.check()
