import math
from pathlib import Path
import sys
import threading
import types

import pytest
from sensor_msgs.msg import CameraInfo

SCRIPT_DIR = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPT_DIR))

if "projekt.msg" not in sys.modules:
    projekt_module = sys.modules.setdefault("projekt", types.ModuleType("projekt"))
    msg_module = types.ModuleType("projekt.msg")
    msg_module.PixelDetection = type("PixelDetection", (), {})
    msg_module.PixelDetectionArray = type("PixelDetectionArray", (), {})
    projekt_module.msg = msg_module
    sys.modules["projekt.msg"] = msg_module

import scara_sorter  # noqa: E402
from attach_detach_controller import AttachDetachController  # noqa: E402
from scara_sorter import (  # noqa: E402
    PickCandidate,
    RigidTransform,
    ScaraKinematics,
    ScaraSorter,
    project_pixel_to_plane,
    project_point_to_pixel,
)
from static_pixel_detection_publisher import StaticPixelDetectionPublisher  # noqa: E402


def make_camera_info() -> CameraInfo:
    msg = CameraInfo()
    msg.width = 640
    msg.height = 480
    msg.k = [500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0]
    return msg


def identity_transform() -> RigidTransform:
    return RigidTransform(
        translation=(0.0, 0.0, 0.0),
        rotation=((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
    )


def test_home_position_is_parallel_to_table_long_side() -> None:
    kinematics = ScaraKinematics()

    q1, q2 = kinematics.solve_xy(0.0, -0.5)

    assert q1 == pytest.approx(-math.pi / 2.0, abs=1e-4)
    assert q2 == pytest.approx(0.0, abs=1e-4)


def test_current_cube_positions_are_reachable() -> None:
    kinematics = ScaraKinematics()

    assert kinematics.is_reachable(0.48, 0.12)
    assert kinematics.is_reachable(0.48, -0.12)


def test_current_bin_centers_are_reachable() -> None:
    kinematics = ScaraKinematics()

    assert kinematics.is_reachable(0.15, 0.30)
    assert kinematics.is_reachable(0.15, -0.30)


def test_pixel_projection_round_trip_on_synthetic_camera() -> None:
    camera_info = make_camera_info()
    transform = identity_transform()
    point_base = (0.20, -0.10, 2.0)

    u, v = project_point_to_pixel(point_base, camera_info, transform)
    projected = project_pixel_to_plane(u, v, camera_info, transform, plane_z=2.0)

    assert projected[0] == pytest.approx(point_base[0], abs=1e-6)
    assert projected[1] == pytest.approx(point_base[1], abs=1e-6)
    assert projected[2] == pytest.approx(point_base[2], abs=1e-6)


def test_world_pose_conversion_matches_default_table_scene() -> None:
    node = object.__new__(StaticPixelDetectionPublisher)
    node._base_world_pose = (0.0, -0.3, 1.02, 1.5708)

    wood = node._world_to_base_point(-0.12, 0.18, 1.065)
    steel = node._world_to_base_point(0.12, 0.18, 1.065)

    assert wood == pytest.approx((0.48, 0.12, 0.045), abs=1e-4)
    assert steel == pytest.approx((0.48, -0.12, 0.045), abs=1e-4)


def test_bin_pose_conversion_matches_current_world_centers() -> None:
    sorter = object.__new__(ScaraSorter)
    sorter._base_world_pose = (0.0, -0.3, 1.02, 1.5708)

    wood_bin = sorter._world_to_base_point(-0.30, -0.15, 1.015)
    steel_bin = sorter._world_to_base_point(0.30, -0.15, 1.015)

    assert wood_bin[:2] == pytest.approx((0.15, 0.30), abs=1e-4)
    assert steel_bin[:2] == pytest.approx((0.15, -0.30), abs=1e-4)


def test_wait_for_attached_releases_unexpected_object(monkeypatch) -> None:
    class FakeLogger:
        def warning(self, _message: str) -> None:
            pass

    class FakePublisher:
        def __init__(self) -> None:
            self.count = 0

        def publish(self, _msg) -> None:
            self.count += 1

    detection = sys.modules["projekt.msg"].PixelDetection()
    detection.object_id = "wood_cube_5cm"
    detection.object_class = "wood"
    candidate = PickCandidate(
        detection=detection,
        key="wood_cube_5cm",
        x=0.48,
        y=0.12,
        joints=(0.0, 0.0),
    )

    sorter = object.__new__(ScaraSorter)
    sorter._state_lock = threading.Lock()
    sorter._attached_object = "steel_cube_5cm"
    sorter._attach_pubs = {
        "wood_cube_5cm": object(),
        "steel_cube_5cm": object(),
    }
    sorter._release_pub = FakePublisher()
    sorter._release_timeout = 0.0
    sorter.get_logger = lambda: FakeLogger()
    sorter._wait_for_released = lambda _timeout: True
    monkeypatch.setattr(scara_sorter.rclpy, "ok", lambda: True)

    assert not sorter._wait_for_attached(1.0, candidate)
    assert sorter._release_pub.count == 1


def test_detachable_joint_state_parser_accepts_boolean_strings() -> None:
    assert AttachDetachController._state_is_attached("true")
    assert AttachDetachController._state_is_attached("1")
    assert AttachDetachController._state_is_attached("attached")
    assert AttachDetachController._state_is_detached("false")
    assert AttachDetachController._state_is_detached("0")
    assert AttachDetachController._state_is_detached("detached")
