from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    world_arg = DeclareLaunchArgument(
        "world",
        default_value="world.sdf",
        description="World file used for bin poses and optional static demo detections",
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation time",
    )
    static_pixel_detections_arg = DeclareLaunchArgument(
        "static_pixel_detections",
        default_value="false",
        description="Use world.sdf cube detections instead of the camera neural network",
    )
    detector_model_arg = DeclareLaunchArgument(
        "detector_model",
        default_value=PathJoinSubstitution(
            [
                FindPackageShare("projekt"),
                "runs",
                "detect",
                "train-3",
                "weights",
                "best.onnx",
            ]
        ),
        description="Path to a trained YOLO .onnx or .pt cube detector model",
    )
    detector_backend_arg = DeclareLaunchArgument(
        "detector_backend",
        default_value="auto",
        description="YOLO backend: auto, onnxruntime, opencv, or ultralytics",
    )
    detector_confidence_arg = DeclareLaunchArgument(
        "detector_confidence",
        default_value="0.35",
        description="Minimum object detection confidence",
    )
    detector_startup_delay_arg = DeclareLaunchArgument(
        "detector_startup_delay",
        default_value="2.0",
        description="Seconds to wait before taking the home-position detection snapshot",
    )
    detector_publish_once_arg = DeclareLaunchArgument(
        "detector_publish_once",
        default_value="true",
        description="Publish one detection snapshot while the robot is home",
    )
    attach_controller_arg = DeclareLaunchArgument(
        "attach_controller",
        default_value="true",
        description="Start the contact-based cube attach controller",
    )
    x_arg = DeclareLaunchArgument(
        "x", default_value="0.0", description="x coordinate of the fixed robot base"
    )
    y_arg = DeclareLaunchArgument(
        "y", default_value="-0.3", description="y coordinate of the fixed robot base"
    )
    z_arg = DeclareLaunchArgument(
        "z", default_value="1.02", description="z coordinate of the fixed robot base"
    )
    yaw_arg = DeclareLaunchArgument(
        "yaw", default_value="1.5708", description="yaw angle of the fixed robot base"
    )

    static_detector_node = Node(
        package="projekt",
        executable="static_pixel_detection_publisher.py",
        name="static_pixel_detection_publisher",
        output="screen",
        condition=IfCondition(LaunchConfiguration("static_pixel_detections")),
        parameters=[
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "detections_topic": "/sorting/pixel_detections",
                "camera_info_topic": "/table_camera/camera_info",
                "base_frame": "base_link",
                "camera_frame": "table_camera_link_optical",
                "load_object_poses_from_world": True,
                "world_file": LaunchConfiguration("world"),
                "base_world_x": LaunchConfiguration("x"),
                "base_world_y": LaunchConfiguration("y"),
                "base_world_z": LaunchConfiguration("z"),
                "base_world_yaw": LaunchConfiguration("yaw"),
            }
        ],
    )

    yolo_detector_node = Node(
        package="projekt",
        executable="yolo_cube_detector.py",
        name="yolo_cube_detector",
        output="screen",
        condition=UnlessCondition(LaunchConfiguration("static_pixel_detections")),
        parameters=[
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "compressed_image_topic": "/table_camera/image/compressed",
                "camera_info_topic": "/table_camera/camera_info",
                "detections_topic": "/sorting/pixel_detections",
                "base_frame": "base_link",
                "camera_frame": "table_camera_link_optical",
                "model_path": LaunchConfiguration("detector_model"),
                "backend": LaunchConfiguration("detector_backend"),
                "class_names": ["wood_cube", "steel_cube"],
                "target_bins": ["wood_collection_bin", "steel_collection_bin"],
                "confidence_threshold": LaunchConfiguration("detector_confidence"),
                "startup_delay": LaunchConfiguration("detector_startup_delay"),
                "publish_once": LaunchConfiguration("detector_publish_once"),
                "mask_plane_z": 0.045,
                "mask_base_rectangles": (
                    "-0.08,0.08,-0.58,0.08;"
                    "0.02,0.28,0.18,0.42;"
                    "0.02,0.28,-0.42,-0.18"
                ),
            }
        ],
    )

    sorter_node = Node(
        package="projekt",
        executable="scara_sorter.py",
        name="scara_sorter",
        output="screen",
        parameters=[
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "detections_topic": "/sorting/pixel_detections",
                "camera_info_topic": "/table_camera/camera_info",
                "shared_bin_x": 0.12,
                "shared_bin_y": 0.40,
                "load_bin_poses_from_world": True,
                "world_file": LaunchConfiguration("world"),
                "base_world_x": LaunchConfiguration("x"),
                "base_world_y": LaunchConfiguration("y"),
                "base_world_z": LaunchConfiguration("z"),
                "base_world_yaw": LaunchConfiguration("yaw"),
                "bin_names": ["wood_collection_bin", "steel_collection_bin"],
                "home_joint1": -1.5708,
                "home_joint2": 0.0,
                "home_joint3": 0.05,
                "travel_joint3": 0.05,
                "direct_attach": False,
                "attach_object_names": ["wood_cube_5cm", "steel_cube_5cm"],
                "attach_topics": ["/wood_cube_5cm/attach", "/steel_cube_5cm/attach"],
            }
        ],
    )

    attach_detach_controller_node = Node(
        package="projekt",
        executable="attach_detach_controller.py",
        name="attach_detach_controller",
        output="screen",
        condition=IfCondition(LaunchConfiguration("attach_controller")),
        parameters=[
            {
                "use_sim_time": False,
                "contact_topic": "/contact_end_effector",
                "object_names": ["wood_cube_5cm", "steel_cube_5cm"],
                "attach_topics": ["/wood_cube_5cm/attach", "/steel_cube_5cm/attach"],
                "detach_topics": ["/wood_cube_5cm/detach", "/steel_cube_5cm/detach"],
                "state_topics": ["/wood_cube_5cm/state", "/steel_cube_5cm/state"],
                "required_contact_names": ["wood_cube_5cm", "steel_cube_5cm"],
                "attached_object_topic": "/gripper/attached_object",
                "release_topic": "/gripper/release",
                "startup_detach_count": 20,
                "startup_detach_period": 0.25,
                "release_contact_suppression_time": 2.0,
                "release_detach_count": 8,
                "release_detach_period": 0.1,
            }
        ],
    )

    return LaunchDescription(
        [
            world_arg,
            use_sim_time_arg,
            static_pixel_detections_arg,
            detector_model_arg,
            detector_backend_arg,
            detector_confidence_arg,
            detector_startup_delay_arg,
            detector_publish_once_arg,
            x_arg,
            y_arg,
            z_arg,
            yaw_arg,
            attach_controller_arg,
            attach_detach_controller_node,
            yolo_detector_node,
            static_detector_node,
            sorter_node,
        ]
    )
