import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    projekt = get_package_share_directory("projekt")

    rviz_arg = DeclareLaunchArgument(
        "rviz", default_value="true", description="Open RViz"
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="true", description="Use simulation time"
    )

    spawn_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(projekt, "launch", "spawn_robot.launch.py")
        ),
        launch_arguments={
            "rviz": LaunchConfiguration("rviz"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "sorting": "true",
            "static_pixel_detections": "true",
        }.items(),
    )

    return LaunchDescription(
        [
            rviz_arg,
            use_sim_time_arg,
            spawn_robot_launch,
        ]
    )
