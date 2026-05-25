from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    world_arg = DeclareLaunchArgument(
        "world",
        default_value="world.sdf",
        description="World file used for static demo object poses",
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation time",
    )
    static_pixel_detections_arg = DeclareLaunchArgument(
        "static_pixel_detections",
        default_value="true",
        description="Publish world.sdf cube detections until the neural network node exists",
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
                "direct_attach": True,
                "attach_object_names": ["wood_cube_5cm", "steel_cube_5cm"],
                "attach_topics": ["/wood_cube_5cm/attach", "/steel_cube_5cm/attach"],
            }
        ],
    )

    return LaunchDescription(
        [
            world_arg,
            use_sim_time_arg,
            static_pixel_detections_arg,
            x_arg,
            y_arg,
            z_arg,
            yaw_arg,
            static_detector_node,
            sorter_node,
        ]
    )
