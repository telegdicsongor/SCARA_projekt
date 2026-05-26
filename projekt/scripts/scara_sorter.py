#!/usr/bin/env python3

from __future__ import annotations

from dataclasses import dataclass
from math import atan2, cos, hypot, isfinite, sin, sqrt
import os
import threading
import time
from typing import Sequence
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from control_msgs.action import FollowJointTrajectory
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Empty, String
from tf2_ros import Buffer, ConnectivityException, ExtrapolationException
from tf2_ros import LookupException, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from projekt.msg import PixelDetection, PixelDetectionArray


SUCCESSFUL_TRAJECTORY = 0


@dataclass(frozen=True)
class RigidTransform:
    translation: tuple[float, float, float]
    rotation: tuple[
        tuple[float, float, float],
        tuple[float, float, float],
        tuple[float, float, float],
    ]


@dataclass(frozen=True)
class PickCandidate:
    detection: PixelDetection
    key: str
    x: float
    y: float
    joints: tuple[float, float]


class ScaraKinematics:
    def __init__(
        self,
        link1_length: float = 0.30,
        link2_length: float = 0.20,
        joint1_min: float = -2.5,
        joint1_max: float = 2.5,
        joint2_min: float = -2.5,
        joint2_max: float = 2.5,
    ) -> None:
        self.link1_length = link1_length
        self.link2_length = link2_length
        self.joint1_min = joint1_min
        self.joint1_max = joint1_max
        self.joint2_min = joint2_min
        self.joint2_max = joint2_max

    def solve_xy(self, x: float, y: float) -> tuple[float, float]:
        if not isfinite(x) or not isfinite(y):
            raise ValueError("Target point must be finite")

        l1 = self.link1_length
        l2 = self.link2_length
        distance = hypot(x, y)
        if distance < abs(l1 - l2) or distance > l1 + l2:
            raise ValueError(f"Target ({x:.3f}, {y:.3f}) is outside SCARA reach")

        c2 = (x * x + y * y - l1 * l1 - l2 * l2) / (2.0 * l1 * l2)
        c2 = max(-1.0, min(1.0, c2))
        s2 = sqrt(max(0.0, 1.0 - c2 * c2))
        q2 = atan2(s2, c2)
        q1 = atan2(y, x) - atan2(l2 * s2, l1 + l2 * c2)

        if not (self.joint1_min <= q1 <= self.joint1_max):
            raise ValueError(f"joint1 solution {q1:.3f} exceeds limits")
        if not (self.joint2_min <= q2 <= self.joint2_max):
            raise ValueError(f"joint2 solution {q2:.3f} exceeds limits")

        return q1, q2

    def is_reachable(self, x: float, y: float) -> bool:
        try:
            self.solve_xy(x, y)
        except ValueError:
            return False
        return True


def quaternion_to_matrix(
    x: float, y: float, z: float, w: float
) -> tuple[
    tuple[float, float, float],
    tuple[float, float, float],
    tuple[float, float, float],
]:
    norm = sqrt(x * x + y * y + z * z + w * w)
    if norm == 0.0:
        raise ValueError("Quaternion has zero length")

    x /= norm
    y /= norm
    z /= norm
    w /= norm

    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z

    return (
        (1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)),
        (2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)),
        (2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)),
    )


def rotate_vector(
    rotation: Sequence[Sequence[float]], vector: Sequence[float]
) -> tuple[float, float, float]:
    return (
        rotation[0][0] * vector[0]
        + rotation[0][1] * vector[1]
        + rotation[0][2] * vector[2],
        rotation[1][0] * vector[0]
        + rotation[1][1] * vector[1]
        + rotation[1][2] * vector[2],
        rotation[2][0] * vector[0]
        + rotation[2][1] * vector[1]
        + rotation[2][2] * vector[2],
    )


def transform_point(
    transform: RigidTransform, point: Sequence[float]
) -> tuple[float, float, float]:
    rotated = rotate_vector(transform.rotation, point)
    return (
        transform.translation[0] + rotated[0],
        transform.translation[1] + rotated[1],
        transform.translation[2] + rotated[2],
    )


def rigid_transform_from_msg(transform_msg) -> RigidTransform:
    translation = transform_msg.transform.translation
    rotation = transform_msg.transform.rotation
    return RigidTransform(
        translation=(translation.x, translation.y, translation.z),
        rotation=quaternion_to_matrix(rotation.x, rotation.y, rotation.z, rotation.w),
    )


def project_pixel_to_plane(
    u: float,
    v: float,
    camera_info: CameraInfo,
    camera_to_base: RigidTransform,
    plane_z: float,
) -> tuple[float, float, float]:
    fx = float(camera_info.k[0])
    fy = float(camera_info.k[4])
    cx = float(camera_info.k[2])
    cy = float(camera_info.k[5])
    if fx == 0.0 or fy == 0.0:
        raise ValueError("Camera intrinsics contain zero focal length")

    ray_camera = ((u - cx) / fx, (v - cy) / fy, 1.0)
    origin_base = camera_to_base.translation
    direction_base = rotate_vector(camera_to_base.rotation, ray_camera)

    if abs(direction_base[2]) < 1e-9:
        raise ValueError("Camera ray is parallel to the pick plane")

    scale = (plane_z - origin_base[2]) / direction_base[2]
    if scale <= 0.0:
        raise ValueError("Camera ray intersects the pick plane behind the camera")

    return (
        origin_base[0] + direction_base[0] * scale,
        origin_base[1] + direction_base[1] * scale,
        plane_z,
    )


def project_point_to_pixel(
    point_base: Sequence[float],
    camera_info: CameraInfo,
    base_to_camera: RigidTransform,
) -> tuple[float, float]:
    point_camera = transform_point(base_to_camera, point_base)
    if point_camera[2] <= 0.0:
        raise ValueError("Point is behind the camera")

    fx = float(camera_info.k[0])
    fy = float(camera_info.k[4])
    cx = float(camera_info.k[2])
    cy = float(camera_info.k[5])
    if fx == 0.0 or fy == 0.0:
        raise ValueError("Camera intrinsics contain zero focal length")

    return (
        fx * point_camera[0] / point_camera[2] + cx,
        fy * point_camera[1] / point_camera[2] + cy,
    )


class ScaraSorter(Node):
    def __init__(self) -> None:
        super().__init__("scara_sorter")

        self.declare_parameter("detections_topic", "/sorting/pixel_detections")
        self.declare_parameter("camera_info_topic", "/table_camera/camera_info")
        self.declare_parameter(
            "controller_action", "/arm_controller/follow_joint_trajectory"
        )
        self.declare_parameter("gripper_release_topic", "/gripper/release")
        self.declare_parameter("gripper_attached_topic", "/gripper/attached_object")
        self.declare_parameter("direct_attach", False)
        self.declare_parameter(
            "attach_object_names", ["wood_cube_5cm", "steel_cube_5cm"]
        )
        self.declare_parameter(
            "attach_topics", ["/wood_cube_5cm/attach", "/steel_cube_5cm/attach"]
        )
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("default_detection_frame", "table_camera_link_optical")
        self.declare_parameter("joint_names", ["joint1", "joint2", "joint3"])
        self.declare_parameter("link1_length", 0.30)
        self.declare_parameter("link2_length", 0.20)
        self.declare_parameter("joint1_min", -2.5)
        self.declare_parameter("joint1_max", 2.5)
        self.declare_parameter("joint2_min", -2.5)
        self.declare_parameter("joint2_max", 2.5)
        self.declare_parameter("min_confidence", 0.30)
        self.declare_parameter("cube_top_z", 0.045)
        self.declare_parameter("pick_region_min_x", 0.05)
        self.declare_parameter("pick_region_max_x", 0.50)
        self.declare_parameter("pick_region_min_y", -0.35)
        self.declare_parameter("pick_region_max_y", 0.35)
        self.declare_parameter("shared_bin_x", 0.12)
        self.declare_parameter("shared_bin_y", 0.40)
        self.declare_parameter("load_bin_poses_from_world", True)
        self.declare_parameter("world_file", "world.sdf")
        self.declare_parameter("base_world_x", 0.0)
        self.declare_parameter("base_world_y", -0.3)
        self.declare_parameter("base_world_z", 1.02)
        self.declare_parameter("base_world_yaw", 1.5708)
        self.declare_parameter(
            "bin_names", ["wood_collection_bin", "steel_collection_bin"]
        )
        self.declare_parameter("bin_base_x", [0.15, 0.15])
        self.declare_parameter("bin_base_y", [0.30, -0.30])
        self.declare_parameter("home_joint1", -1.5708)
        self.declare_parameter("home_joint2", 0.0)
        self.declare_parameter("home_joint3", 0.05)
        self.declare_parameter("travel_joint3", 0.05)
        self.declare_parameter("pick_joint3", -0.115)
        self.declare_parameter("drop_joint3", -0.08)
        self.declare_parameter("move_duration", 2.0)
        self.declare_parameter("vertical_duration", 1.0)
        self.declare_parameter("settle_time", 0.4)
        self.declare_parameter("attach_timeout", 4.0)
        self.declare_parameter("direct_attach_period", 0.5)
        self.declare_parameter("release_timeout", 2.0)
        self.declare_parameter("tick_period", 0.5)

        self._base_frame = self.get_parameter("base_frame").value
        self._default_detection_frame = self.get_parameter(
            "default_detection_frame"
        ).value
        self._joint_names = list(self.get_parameter("joint_names").value)
        self._min_confidence = float(self.get_parameter("min_confidence").value)
        self._cube_top_z = float(self.get_parameter("cube_top_z").value)
        self._pick_region = (
            float(self.get_parameter("pick_region_min_x").value),
            float(self.get_parameter("pick_region_max_x").value),
            float(self.get_parameter("pick_region_min_y").value),
            float(self.get_parameter("pick_region_max_y").value),
        )
        self._shared_bin = (
            float(self.get_parameter("shared_bin_x").value),
            float(self.get_parameter("shared_bin_y").value),
        )
        self._load_bin_poses_from_world = bool(
            self.get_parameter("load_bin_poses_from_world").value
        )
        self._base_world_pose = (
            float(self.get_parameter("base_world_x").value),
            float(self.get_parameter("base_world_y").value),
            float(self.get_parameter("base_world_z").value),
            float(self.get_parameter("base_world_yaw").value),
        )
        self._bin_points = self._load_bin_points()
        self._home = (
            float(self.get_parameter("home_joint1").value),
            float(self.get_parameter("home_joint2").value),
            float(self.get_parameter("home_joint3").value),
        )
        self._travel_joint3 = float(self.get_parameter("travel_joint3").value)
        self._pick_joint3 = float(self.get_parameter("pick_joint3").value)
        self._drop_joint3 = float(self.get_parameter("drop_joint3").value)
        self._move_duration = float(self.get_parameter("move_duration").value)
        self._vertical_duration = float(
            self.get_parameter("vertical_duration").value
        )
        self._settle_time = float(self.get_parameter("settle_time").value)
        self._attach_timeout = float(self.get_parameter("attach_timeout").value)
        self._direct_attach = bool(self.get_parameter("direct_attach").value)
        self._direct_attach_period = float(
            self.get_parameter("direct_attach_period").value
        )
        self._release_timeout = float(self.get_parameter("release_timeout").value)

        self._kinematics = ScaraKinematics(
            link1_length=float(self.get_parameter("link1_length").value),
            link2_length=float(self.get_parameter("link2_length").value),
            joint1_min=float(self.get_parameter("joint1_min").value),
            joint1_max=float(self.get_parameter("joint1_max").value),
            joint2_min=float(self.get_parameter("joint2_min").value),
            joint2_max=float(self.get_parameter("joint2_max").value),
        )

        self._camera_info: CameraInfo | None = None
        self._latest_detections: list[PixelDetection] = []
        self._latest_detection_frame = self._default_detection_frame
        self._completed_keys: set[str] = set()
        self._attached_object = ""
        self._homed_after_empty = False
        self._busy = False
        self._state_lock = threading.Lock()

        detections_topic = self.get_parameter("detections_topic").value
        camera_info_topic = self.get_parameter("camera_info_topic").value
        release_topic = self.get_parameter("gripper_release_topic").value
        attached_topic = self.get_parameter("gripper_attached_topic").value
        controller_action = self.get_parameter("controller_action").value
        attach_object_names = self._string_list_parameter("attach_object_names")
        attach_topics = self._string_list_parameter("attach_topics")

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._action_client = ActionClient(
            self, FollowJointTrajectory, controller_action
        )
        self._release_pub = self.create_publisher(Empty, release_topic, 10)
        self._attach_pubs = {}
        for index, object_name in enumerate(attach_object_names):
            attach_topic = self._topic_at(
                attach_topics, index, object_name, "attach"
            )
            self._attach_pubs[object_name] = self.create_publisher(
                Empty, attach_topic, 10
            )
        self._detections_sub = self.create_subscription(
            PixelDetectionArray, detections_topic, self._on_detections, 10
        )
        self._camera_info_sub = self.create_subscription(
            CameraInfo, camera_info_topic, self._on_camera_info, 10
        )
        self._attached_sub = self.create_subscription(
            String, attached_topic, self._on_attached_object, 10
        )
        self._timer = self.create_timer(
            float(self.get_parameter("tick_period").value), self._tick
        )

        self.get_logger().info(
            "SCARA sorter ready: pixel detections -> IK -> shared bin"
        )

    def _on_camera_info(self, msg: CameraInfo) -> None:
        self._camera_info = msg

    def _on_attached_object(self, msg: String) -> None:
        with self._state_lock:
            self._attached_object = msg.data

    def _on_detections(self, msg: PixelDetectionArray) -> None:
        with self._state_lock:
            self._latest_detections = list(msg.detections)
            self._latest_detection_frame = (
                msg.header.frame_id or self._default_detection_frame
            )
            if any(
                self._detection_key(detection) not in self._completed_keys
                for detection in msg.detections
            ):
                self._homed_after_empty = False

    def _tick(self) -> None:
        with self._state_lock:
            if self._busy:
                return

        candidate, waiting_for_projection = self._select_candidate()
        if candidate is not None:
            self._start_worker(self._run_pick_place, candidate)
            return

        if waiting_for_projection:
            return

        with self._state_lock:
            should_home = not self._homed_after_empty and not self._busy
        if should_home:
            self._start_worker(self._run_home)

    def _select_candidate(self) -> tuple[PickCandidate | None, bool]:
        with self._state_lock:
            detections = list(self._latest_detections)
            detection_frame = self._latest_detection_frame
            completed_keys = set(self._completed_keys)

        if not detections:
            return None, False

        camera_info = self._camera_info
        if camera_info is None:
            return None, True

        camera_frame = (
            detection_frame
            or camera_info.header.frame_id
            or self._default_detection_frame
        )
        try:
            transform_msg = self._tf_buffer.lookup_transform(
                self._base_frame, camera_frame, Time()
            )
            camera_to_base = rigid_transform_from_msg(transform_msg)
        except (LookupException, ConnectivityException, ExtrapolationException) as exc:
            self.get_logger().warning(
                f"Waiting for TF {camera_frame} -> {self._base_frame}: {exc}"
            )
            return None, True

        candidates: list[PickCandidate] = []
        for detection in sorted(
            detections, key=lambda item: item.confidence, reverse=True
        ):
            if detection.confidence < self._min_confidence:
                continue

            key = self._detection_key(detection)
            if key in completed_keys:
                continue

            try:
                point_base = project_pixel_to_plane(
                    detection.center_x,
                    detection.center_y,
                    camera_info,
                    camera_to_base,
                    self._cube_top_z,
                )
                x, y, _ = point_base
                if not self._inside_pick_region(x, y):
                    continue

                joints = self._kinematics.solve_xy(x, y)
            except ValueError as exc:
                self.get_logger().warning(
                    f"Ignoring detection {key}: {exc}"
                )
                continue

            candidates.append(PickCandidate(detection, key, x, y, joints))

        if not candidates:
            return None, False

        return candidates[0], False

    def _inside_pick_region(self, x: float, y: float) -> bool:
        min_x, max_x, min_y, max_y = self._pick_region
        return min_x <= x <= max_x and min_y <= y <= max_y

    @staticmethod
    def _detection_key(detection: PixelDetection) -> str:
        if detection.object_id:
            return detection.object_id
        return (
            f"{detection.object_class}:"
            f"{round(detection.center_x)}:{round(detection.center_y)}"
        )

    def _start_worker(self, target, *args) -> None:
        with self._state_lock:
            if self._busy:
                return
            self._busy = True

        worker = threading.Thread(target=target, args=args, daemon=True)
        worker.start()

    def _run_home(self) -> None:
        try:
            if self._move_joints(self._home, self._move_duration, "home"):
                with self._state_lock:
                    self._homed_after_empty = True
        finally:
            with self._state_lock:
                self._busy = False

    def _run_pick_place(self, candidate: PickCandidate) -> None:
        try:
            q1, q2 = candidate.joints
            drop_point = self._drop_point_for(candidate)
            bin_q1, bin_q2 = self._kinematics.solve_xy(*drop_point)
            object_label = candidate.key

            self.get_logger().info(
                f"Picking {object_label} at base XY "
                f"({candidate.x:.3f}, {candidate.y:.3f})"
            )
            self.get_logger().info(
                f"Drop target for {object_label}: bin center base XY "
                f"({drop_point[0]:.3f}, {drop_point[1]:.3f})"
            )

            pick_above = (q1, q2, self._travel_joint3)
            pick_low = (q1, q2, self._pick_joint3)
            bin_above = (bin_q1, bin_q2, self._travel_joint3)
            bin_low = (bin_q1, bin_q2, self._drop_joint3)

            if not self._move_joints(pick_above, self._move_duration, "above pick"):
                return
            if not self._move_joints(pick_low, self._vertical_duration, "pick"):
                return

            time.sleep(self._settle_time)
            if not self._wait_for_attached(self._attach_timeout, candidate):
                self.get_logger().warning(
                    f"No attachment detected for {object_label}; skipping this detection"
                )
                self._move_joints(
                    pick_above, self._vertical_duration, "lift after missed attach"
                )
                self._mark_candidate_skipped(candidate)
                return

            if not self._move_joints(pick_above, self._vertical_duration, "lift"):
                return
            if not self._move_joints(bin_above, self._move_duration, "above bin"):
                return
            if not self._move_joints(bin_low, self._vertical_duration, "drop"):
                return

            self._release_pub.publish(Empty())
            self._wait_for_released(self._release_timeout)
            time.sleep(self._settle_time)

            self._move_joints(bin_above, self._vertical_duration, "lift from bin")
            with self._state_lock:
                self._completed_keys.add(candidate.key)
            self.get_logger().info(f"Finished sorting {object_label}")
        except ValueError as exc:
            self.get_logger().error(str(exc))
        finally:
            with self._state_lock:
                self._busy = False

    def _move_joints(
        self, positions: Sequence[float], duration: float, label: str
    ) -> bool:
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warning("Waiting for arm_controller action server")
            return False

        goal_msg = FollowJointTrajectory.Goal()
        trajectory = JointTrajectory()
        trajectory.joint_names = list(self._joint_names)

        point = JointTrajectoryPoint()
        point.positions = list(positions)
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
        trajectory.points = [point]
        goal_msg.trajectory = trajectory

        send_future = self._action_client.send_goal_async(goal_msg)
        if not self._wait_for_future(send_future, timeout=10.0):
            self.get_logger().error(f"Timed out sending {label} trajectory")
            return False

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f"Trajectory goal rejected for {label}")
            return False

        result_future = goal_handle.get_result_async()
        if not self._wait_for_future(result_future, timeout=duration + 5.0):
            self.get_logger().error(f"Timed out executing {label} trajectory")
            return False

        result = result_future.result().result
        if result.error_code != SUCCESSFUL_TRAJECTORY:
            self.get_logger().error(
                f"{label} trajectory failed: "
                f"{result.error_code} {result.error_string}"
            )
            return False

        return True

    @staticmethod
    def _wait_for_future(future, timeout: float) -> bool:
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            if future.done():
                return True
            time.sleep(0.02)
        return future.done()

    def _command_direct_attach(
        self, candidate: PickCandidate, *, announce: bool
    ) -> bool:
        if not self._direct_attach:
            return False

        target_name = self._direct_attach_target(candidate)
        if target_name is None:
            if announce:
                self.get_logger().warning(
                    f"No direct attach topic configured for {candidate.key}"
                )
            return False

        self._attach_pubs[target_name].publish(Empty())
        if announce:
            self.get_logger().info(
                f"Direct attach command sent for {target_name}"
            )
        return True

    def _direct_attach_target(self, candidate: PickCandidate) -> str | None:
        labels = [
            candidate.detection.object_id,
            candidate.key,
            candidate.detection.object_class,
        ]

        lowered_targets = {
            target_name.lower(): target_name for target_name in self._attach_pubs
        }
        for label in labels:
            label = label.strip()
            if not label:
                continue

            lowered_label = label.lower()
            if lowered_label in lowered_targets:
                return lowered_targets[lowered_label]

            for lowered_target, target_name in lowered_targets.items():
                if lowered_label in lowered_target or lowered_target in lowered_label:
                    return target_name

        return None

    def _mark_candidate_skipped(self, candidate: PickCandidate) -> None:
        with self._state_lock:
            self._completed_keys.add(candidate.key)

    def _drop_point_for(self, candidate: PickCandidate) -> tuple[float, float]:
        target_bin = candidate.detection.target_bin.strip()
        if target_bin:
            bin_point = self._bin_point_by_label(target_bin)
            if bin_point is not None:
                return bin_point
            self.get_logger().warning(
                f"Detection requested unknown bin '{target_bin}'; using fallback bin"
            )

        labels = [
            candidate.detection.object_class,
            candidate.detection.object_id,
            candidate.key,
        ]
        for label in labels:
            bin_point = self._bin_point_by_label(label)
            if bin_point is not None:
                return bin_point

        return self._shared_bin

    def _bin_point_by_label(self, label: str) -> tuple[float, float] | None:
        label = label.strip().lower()
        if not label:
            return None

        lowered_bins = {
            bin_name.lower(): point for bin_name, point in self._bin_points.items()
        }
        if label in lowered_bins:
            return lowered_bins[label]

        for bin_name, point in lowered_bins.items():
            if label in bin_name or bin_name in label:
                return point

        return None

    def _wait_for_attached(
        self, timeout: float, candidate: PickCandidate | None = None
    ) -> bool:
        deadline = time.monotonic() + timeout
        expected_target = (
            self._direct_attach_target(candidate) if candidate is not None else None
        )
        while rclpy.ok() and time.monotonic() < deadline:
            with self._state_lock:
                attached_object = self._attached_object.strip()

            if attached_object:
                if expected_target is None or attached_object == expected_target:
                    return True
                self.get_logger().warning(
                    f"Expected {expected_target} while picking {candidate.key}, "
                    f"but {attached_object} attached; releasing it"
                )
                self._release_pub.publish(Empty())
                self._wait_for_released(self._release_timeout)
                return False
            time.sleep(0.05)
        return False

    def _wait_for_released(self, timeout: float) -> bool:
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            with self._state_lock:
                if not self._attached_object:
                    return True
            time.sleep(0.05)
        return False

    def _load_bin_points(self) -> dict[str, tuple[float, float]]:
        bin_names = self._string_list_parameter("bin_names")
        bin_xs = self._float_list_parameter("bin_base_x")
        bin_ys = self._float_list_parameter("bin_base_y")

        bin_points = {
            bin_name: (
                self._value_at(bin_xs, index, self._shared_bin[0]),
                self._value_at(bin_ys, index, self._shared_bin[1]),
            )
            for index, bin_name in enumerate(bin_names)
        }

        if self._load_bin_poses_from_world:
            bin_points.update(self._load_world_bin_points(bin_names))

        for bin_name, (x, y) in bin_points.items():
            self.get_logger().info(
                f"Drop target {bin_name}: base XY=({x:.3f}, {y:.3f})"
            )

        return bin_points

    def _load_world_bin_points(
        self, bin_names: Sequence[str]
    ) -> dict[str, tuple[float, float]]:
        if not bin_names:
            return {}

        world_path = self._resolve_world_path(self.get_parameter("world_file").value)
        try:
            root = ET.parse(world_path).getroot()
        except (OSError, ET.ParseError) as exc:
            self.get_logger().warning(
                f"Could not read bin poses from {world_path}: {exc}"
            )
            return {}

        bin_name_set = set(bin_names)
        bin_points: dict[str, tuple[float, float]] = {}
        for include in root.findall(".//include"):
            name = include.findtext("name", default="").strip()
            if name not in bin_name_set:
                continue

            pose_text = include.findtext("pose", default="").strip()
            pose_values = pose_text.split()
            if len(pose_values) < 2:
                self.get_logger().warning(
                    f"World include {name} has no usable XY pose; using parameters"
                )
                continue

            try:
                world_x = float(pose_values[0])
                world_y = float(pose_values[1])
                world_z = float(pose_values[2]) if len(pose_values) >= 3 else 0.0
            except ValueError:
                self.get_logger().warning(
                    f"World include {name} has non-numeric pose; using parameters"
                )
                continue

            base_x, base_y, _ = self._world_to_base_point(world_x, world_y, world_z)
            bin_points[name] = (base_x, base_y)

        missing = bin_name_set - set(bin_points)
        if missing:
            self.get_logger().warning(
                "World file did not contain bin poses for "
                f"{', '.join(sorted(missing))}; using configured bin points"
            )

        return bin_points

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

    def _string_list_parameter(self, parameter_name: str) -> list[str]:
        value = self.get_parameter(parameter_name).value
        if value is None:
            return []
        if isinstance(value, str):
            return [item.strip() for item in value.split(",") if item.strip()]
        return [str(item).strip() for item in value if str(item).strip()]

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

    @staticmethod
    def _topic_at(values: list[str], index: int, object_name: str, suffix: str) -> str:
        if index < len(values):
            return values[index]

        return f"/{object_name}/{suffix}"


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ScaraSorter()
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
