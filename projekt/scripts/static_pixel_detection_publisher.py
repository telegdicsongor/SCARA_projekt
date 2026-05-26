#!/usr/bin/env python3

from __future__ import annotations

from dataclasses import dataclass
from math import cos, sin
import os
from typing import Sequence
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Header
from tf2_ros import Buffer, ConnectivityException, ExtrapolationException
from tf2_ros import LookupException, TransformListener

from projekt.msg import PixelDetection, PixelDetectionArray
from scara_sorter import project_point_to_pixel, rigid_transform_from_msg


@dataclass(frozen=True)
class StaticObject:
    object_id: str
    object_class: str
    base_point: tuple[float, float, float]
    confidence: float
    bbox_width: float
    bbox_height: float
    target_bin: str


class StaticPixelDetectionPublisher(Node):
    """Publish demo pixel detections through the future NN interface."""

    def __init__(self) -> None:
        super().__init__("static_pixel_detection_publisher")

        self.declare_parameter("detections_topic", "/sorting/pixel_detections")
        self.declare_parameter("camera_info_topic", "/table_camera/camera_info")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("camera_frame", "table_camera_link_optical")
        self.declare_parameter("use_camera_info_frame", False)
        self.declare_parameter("load_object_poses_from_world", True)
        self.declare_parameter("world_file", "world.sdf")
        self.declare_parameter("base_world_x", 0.0)
        self.declare_parameter("base_world_y", -0.3)
        self.declare_parameter("base_world_z", 1.02)
        self.declare_parameter("base_world_yaw", 1.5708)
        self.declare_parameter("cube_height", 0.05)
        self.declare_parameter("publish_period", 1.0)
        self.declare_parameter(
            "object_ids", ["wood_cube_5cm", "steel_cube_5cm"]
        )
        self.declare_parameter("object_classes", ["wood", "steel"])
        self.declare_parameter("object_base_x", [0.48, 0.48])
        self.declare_parameter("object_base_y", [0.12, -0.12])
        self.declare_parameter("object_base_z", [0.045, 0.045])
        self.declare_parameter("confidences", [0.98, 0.98])
        self.declare_parameter("bbox_widths", [60.0, 60.0])
        self.declare_parameter("bbox_heights", [60.0, 60.0])
        self.declare_parameter(
            "target_bins", ["wood_collection_bin", "steel_collection_bin"]
        )

        self._base_frame = self.get_parameter("base_frame").value
        self._camera_frame = self.get_parameter("camera_frame").value
        self._use_camera_info_frame = bool(
            self.get_parameter("use_camera_info_frame").value
        )
        self._load_object_poses_from_world = bool(
            self.get_parameter("load_object_poses_from_world").value
        )
        self._base_world_pose = (
            float(self.get_parameter("base_world_x").value),
            float(self.get_parameter("base_world_y").value),
            float(self.get_parameter("base_world_z").value),
            float(self.get_parameter("base_world_yaw").value),
        )
        self._cube_height = float(self.get_parameter("cube_height").value)
        self._camera_info: CameraInfo | None = None
        self._warned_waiting_for_camera = False
        self._warned_tf = False

        self._objects = self._load_objects()
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._publisher = self.create_publisher(
            PixelDetectionArray,
            self.get_parameter("detections_topic").value,
            10,
        )
        self._camera_info_sub = self.create_subscription(
            CameraInfo,
            self.get_parameter("camera_info_topic").value,
            self._on_camera_info,
            10,
        )
        self._timer = self.create_timer(
            float(self.get_parameter("publish_period").value), self._publish
        )

        self.get_logger().info(
            "Static pixel detector publishing demo cube detections"
        )

    def _load_objects(self) -> list[StaticObject]:
        ids = self._string_list_parameter("object_ids")
        classes = self._string_list_parameter("object_classes")
        xs = self._float_list_parameter("object_base_x")
        ys = self._float_list_parameter("object_base_y")
        zs = self._float_list_parameter("object_base_z")
        world_poses = self._load_world_object_poses(ids)
        confidences = self._float_list_parameter("confidences")
        widths = self._float_list_parameter("bbox_widths")
        heights = self._float_list_parameter("bbox_heights")
        bins = self._string_list_parameter("target_bins")

        count = len(ids)
        objects: list[StaticObject] = []
        for index, object_id in enumerate(ids):
            base_point = (
                self._value_at(xs, index, 0.0),
                self._value_at(ys, index, 0.0),
                self._value_at(zs, index, 0.045),
            )
            if object_id in world_poses:
                base_point = world_poses[object_id]

            objects.append(
                StaticObject(
                    object_id=object_id,
                    object_class=self._value_at(classes, index, object_id),
                    base_point=base_point,
                    confidence=self._value_at(confidences, index, 1.0),
                    bbox_width=self._value_at(widths, index, 60.0),
                    bbox_height=self._value_at(heights, index, 60.0),
                    target_bin=self._value_at(bins, index, ""),
                )
            )

        if not objects:
            raise ValueError("At least one static object must be configured")

        if any(len(values) not in (0, count) for values in (classes, xs, ys, zs)):
            self.get_logger().warning(
                "Static detector parameter list lengths differ; missing values use defaults"
            )

        for static_object in objects:
            x, y, z = static_object.base_point
            self.get_logger().info(
                f"Static detection target {static_object.object_id}: "
                f"base=({x:.3f}, {y:.3f}, {z:.3f})"
            )

        return objects

    def _load_world_object_poses(
        self, object_ids: Sequence[str]
    ) -> dict[str, tuple[float, float, float]]:
        if not self._load_object_poses_from_world:
            return {}

        world_path = self._resolve_world_path(self.get_parameter("world_file").value)
        try:
            root = ET.parse(world_path).getroot()
        except (OSError, ET.ParseError) as exc:
            self.get_logger().warning(
                f"Could not read world poses from {world_path}: {exc}"
            )
            return {}

        object_id_set = set(object_ids)
        world_poses: dict[str, tuple[float, float, float]] = {}
        for include in root.findall(".//include"):
            name = include.findtext("name", default="").strip()
            if name not in object_id_set:
                continue

            pose_text = include.findtext("pose", default="").strip()
            pose_values = pose_text.split()
            if len(pose_values) < 3:
                self.get_logger().warning(
                    f"World include {name} has no usable pose; using parameters"
                )
                continue

            try:
                world_x = float(pose_values[0])
                world_y = float(pose_values[1])
                world_z = float(pose_values[2]) + self._cube_height
            except ValueError:
                self.get_logger().warning(
                    f"World include {name} has non-numeric pose; using parameters"
                )
                continue

            world_poses[name] = self._world_to_base_point(world_x, world_y, world_z)

        missing = object_id_set - set(world_poses)
        if missing:
            self.get_logger().warning(
                "World file did not contain poses for "
                f"{', '.join(sorted(missing))}; using configured base points"
            )

        return world_poses

    def _resolve_world_path(self, world_file: str) -> str:
        world_file = os.path.expanduser(world_file)
        if os.path.isabs(world_file):
            return world_file

        package_share = get_package_share_directory("projekt")
        return os.path.join(package_share, "worlds", world_file)

    def _world_to_base_point(
        self, world_x: float, world_y: float, world_z: float
    ) -> tuple[float, float, float]:
        base_x, base_y, base_z, base_yaw = self._base_world_pose
        dx = world_x - base_x
        dy = world_y - base_y
        yaw_cos = cos(base_yaw)
        yaw_sin = sin(base_yaw)

        return (
            yaw_cos * dx + yaw_sin * dy,
            -yaw_sin * dx + yaw_cos * dy,
            world_z - base_z,
        )

    def _on_camera_info(self, msg: CameraInfo) -> None:
        self._camera_info = msg
        self._warned_waiting_for_camera = False

    def _publish(self) -> None:
        if self._camera_info is None:
            if not self._warned_waiting_for_camera:
                self.get_logger().info("Waiting for camera info before publishing")
                self._warned_waiting_for_camera = True
            return

        camera_frame = self._camera_frame
        if self._use_camera_info_frame and self._camera_info.header.frame_id:
            camera_frame = self._camera_info.header.frame_id
        try:
            transform_msg = self._tf_buffer.lookup_transform(
                camera_frame, self._base_frame, Time()
            )
            base_to_camera = rigid_transform_from_msg(transform_msg)
            self._warned_tf = False
        except (LookupException, ConnectivityException, ExtrapolationException) as exc:
            if not self._warned_tf:
                self.get_logger().warning(
                    f"Waiting for TF {self._base_frame} -> {camera_frame}: {exc}"
                )
                self._warned_tf = True
            return

        msg = PixelDetectionArray()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = camera_frame

        for static_object in self._objects:
            try:
                u, v = project_point_to_pixel(
                    static_object.base_point, self._camera_info, base_to_camera
                )
            except ValueError as exc:
                self.get_logger().warning(
                    f"Could not project {static_object.object_id}: {exc}"
                )
                continue

            if not self._inside_image(u, v):
                continue

            detection = PixelDetection()
            detection.object_id = static_object.object_id
            detection.object_class = static_object.object_class
            detection.center_x = float(u)
            detection.center_y = float(v)
            detection.bbox_width = static_object.bbox_width
            detection.bbox_height = static_object.bbox_height
            detection.confidence = static_object.confidence
            detection.target_bin = static_object.target_bin
            msg.detections.append(detection)

        self._publisher.publish(msg)

    def _inside_image(self, u: float, v: float) -> bool:
        width = int(self._camera_info.width)
        height = int(self._camera_info.height)
        if width <= 0 or height <= 0:
            return True
        return 0.0 <= u < width and 0.0 <= v < height

    def _string_list_parameter(self, parameter_name: str) -> list[str]:
        value = self.get_parameter(parameter_name).value
        if value is None:
            return []
        if isinstance(value, str):
            return [item.strip() for item in value.split(",")]
        return [str(item).strip() for item in value]

    def _float_list_parameter(self, parameter_name: str) -> list[float]:
        value = self.get_parameter(parameter_name).value
        if value is None:
            return []
        if isinstance(value, (float, int)):
            return [float(value)]
        return [float(item) for item in value]

    @staticmethod
    def _value_at(values: Sequence, index: int, default):
        if index < len(values):
            return values[index]
        return default


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StaticPixelDetectionPublisher()
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
