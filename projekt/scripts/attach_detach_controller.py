#!/usr/bin/env python3

from __future__ import annotations

from typing import Iterable

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from ros_gz_interfaces.msg import Contacts
from std_msgs.msg import Empty, String


class AttachDetachController(Node):
    """Control the Gazebo detachable joint from end-effector contact events."""

    def __init__(self) -> None:
        super().__init__("attach_detach_controller")

        self.declare_parameter("contact_topic", "/contact_end_effector")
        self.declare_parameter("attach_topic", "/wood_cube_5cm/attach")
        self.declare_parameter("detach_topic", "/wood_cube_5cm/detach")
        self.declare_parameter("state_topic", "/wood_cube_5cm/state")
        self.declare_parameter("required_contact_name", "wood_cube_5cm")
        self.declare_parameter("startup_detach_count", 8)
        self.declare_parameter("startup_detach_period", 0.25)

        self._contact_topic = self.get_parameter("contact_topic").value
        self._required_contact_name = self.get_parameter(
            "required_contact_name"
        ).value
        self._startup_detach_remaining = int(
            self.get_parameter("startup_detach_count").value
        )
        startup_detach_period = float(
            self.get_parameter("startup_detach_period").value
        )

        self._attached = False
        self._startup_detaching = self._startup_detach_remaining > 0
        self._pending_attach = False

        command_qos = QoSProfile(depth=10)
        contact_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        self._attach_pub = self.create_publisher(
            Empty, self.get_parameter("attach_topic").value, command_qos
        )
        self._detach_pub = self.create_publisher(
            Empty, self.get_parameter("detach_topic").value, command_qos
        )
        self._contact_sub = self.create_subscription(
            Contacts, self._contact_topic, self._on_contact, contact_qos
        )
        self._state_sub = self.create_subscription(
            String, self.get_parameter("state_topic").value, self._on_state, 10
        )

        self._startup_timer = self.create_timer(
            startup_detach_period, self._publish_startup_detach
        )
        self._publish_startup_detach()

        self.get_logger().info(
            "Attach-detach controller started with joint detached by default"
        )

    def _publish_startup_detach(self) -> None:
        if self._startup_detach_remaining <= 0:
            self._startup_detaching = False
            self._startup_timer.cancel()
            return

        self._detach_pub.publish(Empty())
        self._attached = False
        self._startup_detach_remaining -= 1

        if self._startup_detach_remaining == 0:
            self._startup_detaching = False
            self._startup_timer.cancel()
            self.get_logger().info("Startup detach command sent")
            if self._pending_attach:
                self._publish_attach(
                    "Queued wood cube contact detected; attaching joint"
                )

    def _on_state(self, msg: String) -> None:
        state = msg.data.lower()
        if "detach" in state:
            self._attached = False
        elif "attach" in state:
            self._attached = True

    def _on_contact(self, msg: Contacts) -> None:
        if self._attached or not msg.contacts:
            return

        for contact in msg.contacts:
            if self._is_wood_cube_contact(contact):
                if self._startup_detaching:
                    self._pending_attach = True
                    return

                self._publish_attach("Wood cube contact detected; attaching joint")
                return

    def _publish_attach(self, log_message: str) -> None:
        self._attach_pub.publish(Empty())
        self._attached = True
        self._pending_attach = False
        self.get_logger().info(log_message)

    def _is_wood_cube_contact(self, contact) -> bool:
        if not self._required_contact_name:
            return True

        required_name = str(self._required_contact_name).lower()
        return any(
            required_name in name.lower() for name in self._contact_names(contact)
        )

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
