#!/usr/bin/env python3

from __future__ import annotations

from dataclasses import dataclass
import time
from typing import Iterable

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from ros_gz_interfaces.msg import Contacts
from std_msgs.msg import Empty, String


@dataclass
class AttachmentTarget:
    name: str
    contact_name: str
    attach_pub: object
    detach_pub: object


class AttachDetachController(Node):
    """Control the Gazebo detachable joint from end-effector contact events."""

    def __init__(self) -> None:
        super().__init__("attach_detach_controller")

        self.declare_parameter("contact_topic", "/contact_end_effector")
        self.declare_parameter(
            "object_names", ["wood_cube_5cm", "steel_cube_5cm"]
        )
        self.declare_parameter(
            "attach_topics", ["/wood_cube_5cm/attach", "/steel_cube_5cm/attach"]
        )
        self.declare_parameter(
            "detach_topics", ["/wood_cube_5cm/detach", "/steel_cube_5cm/detach"]
        )
        self.declare_parameter(
            "state_topics", ["/wood_cube_5cm/state", "/steel_cube_5cm/state"]
        )
        self.declare_parameter(
            "required_contact_names", ["wood_cube_5cm", "steel_cube_5cm"]
        )
        self.declare_parameter("attached_object_topic", "/gripper/attached_object")
        self.declare_parameter("release_topic", "/gripper/release")
        self.declare_parameter("startup_detach_count", 20)
        self.declare_parameter("startup_detach_period", 0.25)
        self.declare_parameter("release_contact_suppression_time", 2.0)
        self.declare_parameter("release_detach_count", 8)
        self.declare_parameter("release_detach_period", 0.1)

        self._contact_topic = self.get_parameter("contact_topic").value
        self._startup_detach_remaining = int(
            self.get_parameter("startup_detach_count").value
        )
        startup_detach_period = float(
            self.get_parameter("startup_detach_period").value
        )
        self._release_contact_suppression_time = float(
            self.get_parameter("release_contact_suppression_time").value
        )
        self._release_detach_count = max(
            1, int(self.get_parameter("release_detach_count").value)
        )
        release_detach_period = float(
            self.get_parameter("release_detach_period").value
        )

        self._attached_target: str | None = None
        self._startup_detaching = self._startup_detach_remaining > 0
        self._pending_attach_target: str | None = None
        self._suppress_contact_until = 0.0
        self._release_detach_targets: list[AttachmentTarget] = []
        self._release_detach_remaining = 0
        self._state_subs = []
        self._command_subs = []

        command_qos = QoSProfile(depth=10)
        contact_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        self._targets = self._configure_targets(command_qos)
        self._target_by_name = {target.name: target for target in self._targets}
        self._attached_state_pub = self.create_publisher(
            String, self.get_parameter("attached_object_topic").value, command_qos
        )
        self._release_sub = self.create_subscription(
            Empty,
            self.get_parameter("release_topic").value,
            self._on_release,
            command_qos,
        )
        self._contact_sub = self.create_subscription(
            Contacts, self._contact_topic, self._on_contact, contact_qos
        )

        self._startup_timer = self.create_timer(
            startup_detach_period, self._publish_startup_detach
        )
        self._release_detach_timer = self.create_timer(
            release_detach_period, self._publish_release_detach
        )
        self._release_detach_timer.cancel()
        self._publish_startup_detach()

        target_names = ", ".join(target.name for target in self._targets)
        self.get_logger().info(
            "Attach-detach controller started for "
            f"{target_names}; publishing startup detach commands"
        )
        self._publish_attached_state()

    def _configure_targets(self, command_qos: QoSProfile) -> list[AttachmentTarget]:
        names = self._string_list_parameter("object_names")
        attach_topics = self._string_list_parameter("attach_topics")
        detach_topics = self._string_list_parameter("detach_topics")
        state_topics = self._string_list_parameter("state_topics")
        contact_names = self._string_list_parameter("required_contact_names")

        if not names:
            raise ValueError("At least one object name must be configured")

        for parameter_name, values in (
            ("attach_topics", attach_topics),
            ("detach_topics", detach_topics),
            ("state_topics", state_topics),
            ("required_contact_names", contact_names),
        ):
            if values and len(values) != len(names):
                self.get_logger().warn(
                    f"Parameter '{parameter_name}' has {len(values)} values for "
                    f"{len(names)} objects; missing entries will use defaults"
                )

        targets: list[AttachmentTarget] = []
        for index, name in enumerate(names):
            attach_topic = self._topic_at(attach_topics, index, name, "attach")
            detach_topic = self._topic_at(detach_topics, index, name, "detach")
            state_topic = self._topic_at(state_topics, index, name, "state")
            contact_name = self._value_at(contact_names, index, name).lower()

            attach_pub = self.create_publisher(Empty, attach_topic, command_qos)
            detach_pub = self.create_publisher(Empty, detach_topic, command_qos)
            self._state_subs.append(
                self.create_subscription(
                    String,
                    state_topic,
                    lambda msg, target_name=name: self._on_state(target_name, msg),
                    10,
                )
            )
            self._command_subs.append(
                self.create_subscription(
                    Empty,
                    attach_topic,
                    lambda msg, target_name=name: self._on_attach_command(
                        target_name, msg
                    ),
                    command_qos,
                )
            )
            targets.append(
                AttachmentTarget(
                    name=name,
                    contact_name=contact_name,
                    attach_pub=attach_pub,
                    detach_pub=detach_pub,
                )
            )

        return targets

    def _publish_startup_detach(self) -> None:
        if self._startup_detach_remaining <= 0:
            self._startup_detaching = False
            self._startup_timer.cancel()
            return

        for target in self._targets:
            target.detach_pub.publish(Empty())

        self._attached_target = None
        self._publish_attached_state()
        self._startup_detach_remaining -= 1

        if self._startup_detach_remaining == 0:
            self._startup_detaching = False
            self._startup_timer.cancel()
            self.get_logger().info("Startup detach commands sent")
            if self._pending_attach_target:
                target = self._target_by_name.get(self._pending_attach_target)
                if target:
                    self._publish_attach(
                        target,
                        f"Queued {target.name} contact detected; requesting attach joint",
                    )
                else:
                    self._pending_attach_target = None

    def _on_state(self, target_name: str, msg: String) -> None:
        state = msg.data.strip().lower()
        if self._state_is_detached(state):
            if self._attached_target == target_name:
                self._attached_target = None
                self._publish_attached_state()
            return

        if self._state_is_attached(state):
            if self._attached_target == target_name:
                self._publish_attached_state()
                return

            if self._attached_target and self._attached_target != target_name:
                target = self._target_by_name.get(target_name)
                if target:
                    target.detach_pub.publish(Empty())
                    self.get_logger().warn(
                        f"{target_name} reported attached while "
                        f"{self._attached_target} is active; detaching {target_name}"
                    )
                return

            if self._pending_attach_target == target_name:
                self._attached_target = target_name
                self._pending_attach_target = None
                self._publish_attached_state()
                return

            if not self._attached_target:
                target = self._target_by_name.get(target_name)
                if target:
                    target.detach_pub.publish(Empty())
                    self.get_logger().warn(
                        f"{target_name} reported attached without contact; detaching"
                    )
                return

        if state:
            self.get_logger().warn(
                f"Ignoring unknown detachable joint state for {target_name}: {state}"
            )

    def _on_attach_command(self, target_name: str, _msg: Empty) -> None:
        if self._startup_detaching:
            return

        if (
            self._attached_target == target_name
            or self._pending_attach_target == target_name
        ):
            return

        self.get_logger().warn(
            f"Ignoring direct attach command for {target_name}; waiting for contact"
        )

    def _on_contact(self, msg: Contacts) -> None:
        if time.monotonic() < self._suppress_contact_until:
            return
        if self._attached_target or self._pending_attach_target or not msg.contacts:
            return

        for contact in msg.contacts:
            target = self._contact_target(contact)
            if target:
                if self._startup_detaching:
                    if self._pending_attach_target is None:
                        self._pending_attach_target = target.name
                    return

                self._publish_attach(
                    target,
                    f"{target.name} contact detected; requesting attach joint",
                )
                return

    def _publish_attach(self, target: AttachmentTarget, log_message: str) -> None:
        if self._attached_target and self._attached_target != target.name:
            return
        if self._pending_attach_target and self._pending_attach_target != target.name:
            return

        target.attach_pub.publish(Empty())
        self._pending_attach_target = target.name
        self.get_logger().info(log_message)

    def _on_release(self, _msg: Empty) -> None:
        active_target = self._attached_target or self._pending_attach_target
        if active_target:
            target = self._target_by_name.get(active_target)
            if target:
                self._start_release_detach([target])
                self.get_logger().info(f"Release requested; detaching {target.name}")
        else:
            self._start_release_detach(self._targets)
            self.get_logger().info("Release requested; broadcasting detach")

        self._attached_target = None
        self._pending_attach_target = None
        self._publish_attached_state()

    def _start_release_detach(self, targets: Iterable[AttachmentTarget]) -> None:
        self._release_detach_targets = list(targets)
        self._release_detach_remaining = self._release_detach_count
        self._suppress_contact_until = (
            time.monotonic() + self._release_contact_suppression_time
        )
        self._publish_release_detach()
        if self._release_detach_remaining > 0:
            self._release_detach_timer.reset()

    def _publish_release_detach(self) -> None:
        if self._release_detach_remaining <= 0 or not self._release_detach_targets:
            self._release_detach_timer.cancel()
            return

        for target in self._release_detach_targets:
            target.detach_pub.publish(Empty())

        self._release_detach_remaining -= 1
        if self._release_detach_remaining <= 0:
            self._release_detach_timer.cancel()

    def _publish_attached_state(self) -> None:
        msg = String()
        msg.data = self._attached_target or ""
        self._attached_state_pub.publish(msg)

    @staticmethod
    def _state_is_attached(state: str) -> bool:
        return state in {"1", "true", "attached"} or "attach" in state

    @staticmethod
    def _state_is_detached(state: str) -> bool:
        return state in {"0", "false", "detached"} or "detach" in state

    def _contact_target(self, contact) -> AttachmentTarget | None:
        names = [name.lower() for name in self._contact_names(contact)]
        for target in self._targets:
            if not target.contact_name:
                return target
            if any(target.contact_name in name for name in names):
                return target

        return None

    def _string_list_parameter(self, parameter_name: str) -> list[str]:
        value = self.get_parameter(parameter_name).value
        if value is None:
            return []
        if isinstance(value, str):
            return [item.strip() for item in value.split(",") if item.strip()]
        return [str(item).strip() for item in value if str(item).strip()]

    @staticmethod
    def _topic_at(values: list[str], index: int, object_name: str, suffix: str) -> str:
        if index < len(values):
            return values[index]

        return f"/{object_name}/{suffix}"

    @staticmethod
    def _value_at(values: list[str], index: int, default: str) -> str:
        if index < len(values):
            return values[index]

        return default

    @staticmethod
    def _contact_names(contact) -> Iterable[str]:
        for entity in (contact.collision1, contact.collision2):
            if entity.name:
                yield entity.name


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AttachDetachController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
