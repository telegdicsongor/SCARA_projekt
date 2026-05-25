#!/usr/bin/env python3

from __future__ import annotations

from datetime import datetime
from pathlib import Path

import cv2
from cv_bridge import CvBridge, CvBridgeError
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image


class TrainingImageCollector(Node):
    """Save labeled table camera frames for cube classifier training."""

    LABEL_KEYS = {
        ord("w"): "wood_cube",
        ord("s"): "steel_cube",
        ord("n"): "not_cube",
    }

    def __init__(self) -> None:
        super().__init__("training_image_collector")

        self.declare_parameter("image_topic", "/table_camera/image")
        self.declare_parameter("save_root", "training_images")
        self.declare_parameter("window_name", "SCARA training image collector")

        self._image_topic = self.get_parameter("image_topic").value
        self._save_root = self._resolve_save_root(
            self.get_parameter("save_root").value
        )
        self._window_name = self.get_parameter("window_name").value
        self._bridge = CvBridge()
        self._latest_frame = None
        self._latest_stamp = None
        self._save_counts = {label: 0 for label in self.LABEL_KEYS.values()}

        for label in self._save_counts:
            (self._save_root / label).mkdir(parents=True, exist_ok=True)

        self._subscription = self.create_subscription(
            Image,
            self._image_topic,
            self._on_image,
            1,
        )
        self._timer = self.create_timer(0.03, self._update_window)

        self.get_logger().info(f"Subscribed to {self._image_topic}")
        self.get_logger().info(f"Saving training images under {self._save_root}")
        self.get_logger().info("Keys: w=wood cube, s=steel cube, n=not cube, q=quit")

    def _resolve_save_root(self, save_root: str) -> Path:
        path = Path(save_root).expanduser()
        if not path.is_absolute():
            path = Path.cwd() / path
        return path

    def _on_image(self, msg: Image) -> None:
        try:
            self._latest_frame = self._bridge.imgmsg_to_cv2(
                msg, desired_encoding="bgr8"
            )
            self._latest_stamp = msg.header.stamp
        except CvBridgeError as exc:
            self.get_logger().warning(f"Could not convert image: {exc}")

    def _update_window(self) -> None:
        if self._latest_frame is None:
            return

        display = self._latest_frame.copy()
        self._draw_overlay(display)
        cv2.imshow(self._window_name, display)

        key = cv2.waitKey(1) & 0xFF
        if key in self.LABEL_KEYS:
            self._save_frame(self.LABEL_KEYS[key])
        elif key == ord("q"):
            self.get_logger().info("Quit requested from OpenCV window")
            rclpy.shutdown()

    def _draw_overlay(self, frame) -> None:
        lines = [
            "w: wood cube",
            "s: steel cube",
            "n: not cube / background",
            "q: quit",
        ]
        y = 24
        for line in lines:
            cv2.putText(
                frame,
                line,
                (12, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )
            cv2.putText(
                frame,
                line,
                (12, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (20, 20, 20),
                1,
                cv2.LINE_AA,
            )
            y += 24

        counts = " | ".join(
            f"{label}: {count}" for label, count in self._save_counts.items()
        )
        cv2.putText(
            frame,
            counts,
            (12, frame.shape[0] - 16),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            frame,
            counts,
            (12, frame.shape[0] - 16),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (20, 20, 20),
            1,
            cv2.LINE_AA,
        )

    def _save_frame(self, label: str) -> None:
        if self._latest_frame is None:
            return

        self._save_counts[label] += 1
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        filename = f"{label}_{timestamp}_{self._save_counts[label]:04d}.png"
        output_path = self._save_root / label / filename

        if cv2.imwrite(str(output_path), self._latest_frame):
            self.get_logger().info(f"Saved {output_path}")
        else:
            self.get_logger().error(f"Failed to save {output_path}")

    def destroy_node(self) -> bool:
        cv2.destroyAllWindows()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TrainingImageCollector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
