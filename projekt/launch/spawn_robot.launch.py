import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_prefix, get_package_share_directory

def generate_launch_description():

    projekt = get_package_share_directory('projekt')

    gazebo_models_path, ignore_last_dir = os.path.split(projekt)
    os.environ["GZ_SIM_RESOURCE_PATH"] = os.environ.get("GZ_SIM_RESOURCE_PATH", "") + os.pathsep + gazebo_models_path
    gz_ros2_control_lib_path = os.path.join(get_package_prefix('gz_ros2_control'), 'lib')
    os.environ["GZ_SIM_SYSTEM_PLUGIN_PATH"] = os.environ.get("GZ_SIM_SYSTEM_PLUGIN_PATH", "") + os.pathsep + gz_ros2_control_lib_path

    rviz_launch_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Open RViz'
    )

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config', default_value='rviz.rviz',
        description='RViz config file'
    )

    world_arg = DeclareLaunchArgument(
        'world', default_value='world.sdf',
        description='Name of the Gazebo world file to load'
    )

    model_arg = DeclareLaunchArgument(
        'model', default_value='scara.urdf',
        description='Name of the URDF description to load'
    )

    x_arg = DeclareLaunchArgument(
        'x', default_value='0.0',
        description='x coordinate of spawned robot'
    )

    y_arg = DeclareLaunchArgument(
        'y', default_value='-0.3',
        description='y coordinate of spawned robot'
    )

    z_arg = DeclareLaunchArgument(
        'z', default_value='1.02',
        description='z coordinate of spawned robot'
    )

    yaw_arg = DeclareLaunchArgument(
        'yaw', default_value='1.5708',
        description='yaw angle of spawned robot'
    )

    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='True',
        description='Flag to enable use_sim_time'
    )

    fake_joint_states_arg = DeclareLaunchArgument(
        'fake_joint_states', default_value='true',
        description='Publish zero joint states so RViz has transforms before controllers publish'
    )

    # Define the path to your URDF or Xacro file
    urdf_file_path = PathJoinSubstitution([
        projekt,  # Replace with your package name
        "urdf",
        LaunchConfiguration('model')  # Replace with your URDF or Xacro file
    ])

    gz_bridge_params_path = os.path.join(
        get_package_share_directory('projekt'),
        'config',
        'gz_bridge.yaml'
    )

    robot_controllers = PathJoinSubstitution(
        [
            get_package_share_directory('projekt'),
            'config',
            'controller_position.yaml',
        ]
    )

    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(projekt, 'launch', 'world.launch.py'),
        ),
        launch_arguments={
        'world': LaunchConfiguration('world'),
        }.items()
    )

    # Launch rviz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', PathJoinSubstitution([projekt, 'rviz', LaunchConfiguration('rviz_config')])],
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )

    # Spawn the URDF model using the `/world/<world_name>/create` service
    spawn_urdf_node = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "scara",
            "-topic", "robot_description",
            "-x", LaunchConfiguration('x'), "-y", LaunchConfiguration('y'), "-z", LaunchConfiguration('z'), "-Y", LaunchConfiguration('yaw')  # Initial spawn position
        ],
        output="screen",
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )

    # Node to bridge topics between ROS and Gazebo
    gz_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args', '-p',
            f'config_file:={gz_bridge_params_path}'
        ],
        output="screen",
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': Command(['xacro', ' ', urdf_file_path]),
             'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        condition=IfCondition(LaunchConfiguration('fake_joint_states')),
        parameters=[
            {'robot_description': Command(['xacro', ' ', urdf_file_path]),
             'rate': 10,
             'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    world_to_base_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_base_footprint_tf',
        arguments=[
            '--x',
            LaunchConfiguration('x'),
            '--y',
            LaunchConfiguration('y'),
            '--z',
            LaunchConfiguration('z'),
            '--roll',
            '0.0',
            '--pitch',
            '0.0',
            '--yaw',
            LaunchConfiguration('yaw'),
            '--frame-id',
            'world',
            '--child-frame-id',
            'base_footprint',
        ],
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    joint_trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'arm_controller',
            #'gripper_controller',
            '--param-file',
            robot_controllers,
            '--controller-manager-timeout',
            '60',
            ],
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager-timeout',
            '60',
        ],
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )

    home_trajectory_goal = (
        "{trajectory: {joint_names: [joint1, joint2, joint3], "
        "points: [{positions: [0.0, 0.0, 0.0], "
        "time_from_start: {sec: 1, nanosec: 0}}]}}"
    )

    home_robot_node = ExecuteProcess(
        cmd=[
            'ros2',
            'action',
            'send_goal',
            '/arm_controller/follow_joint_trajectory',
            'control_msgs/action/FollowJointTrajectory',
            home_trajectory_goal,
        ],
        output='screen',
    )

    joint_state_broadcaster_after_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_urdf_node,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    arm_controller_after_joint_states = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[joint_trajectory_controller_spawner],
        )
    )

    home_robot_after_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_trajectory_controller_spawner,
            on_exit=[home_robot_node],
        )
    )

    attach_detach_controller_node = Node(
        package='projekt',
        executable='attach_detach_controller.py',
        name='attach_detach_controller',
        output='screen',
        parameters=[
            {'use_sim_time': False,
             'contact_topic': '/contact_end_effector',
             'attach_topic': '/wood_cube_5cm/attach',
             'detach_topic': '/wood_cube_5cm/detach',
             'state_topic': '/wood_cube_5cm/state',
             'required_contact_name': 'wood_cube_5cm',
             'startup_detach_count': 40,
             'startup_detach_period': 0.25},
        ],
    )

    attach_detach_controller_after_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_urdf_node,
            on_exit=[
                TimerAction(
                    period=1.0,
                    actions=[attach_detach_controller_node],
                )
            ],
        )
    )

    rviz_after_start = TimerAction(
        period=2.0,
        actions=[rviz_node],
    )

    # Node to bridge camera topics
    gz_image_bridge_node = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=[
            #"/gripper_camera/image",
            "/table_camera/image",
        ],
        output="screen",
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time'),
             #'gripper_camera.image.compressed.jpeg_quality': 75,
             'table_camera.image.compressed.jpeg_quality': 75,},
        ],
    )

    # Relay node to republish camera_info to image/camera_info
    relay_gripper_camera_info_node = Node(
        package='topic_tools',
        executable='relay',
        name='relay_camera_info',
        output='screen',
        arguments=['gripper_camera/camera_info', 'gripper_camera/image/camera_info'],
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )

    # Relay node to republish camera_info to image/camera_info
    relay_table_camera_info_node = Node(
        package='topic_tools',
        executable='relay',
        name='relay_camera_info',
        output='screen',
        arguments=['table_camera/camera_info', 'table_camera/image/camera_info'],
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
    )

    launchDescriptionObject = LaunchDescription()

    launchDescriptionObject.add_action(rviz_launch_arg)
    launchDescriptionObject.add_action(rviz_config_arg)
    launchDescriptionObject.add_action(world_arg)
    launchDescriptionObject.add_action(model_arg)
    launchDescriptionObject.add_action(x_arg)
    launchDescriptionObject.add_action(y_arg)
    launchDescriptionObject.add_action(z_arg)
    launchDescriptionObject.add_action(yaw_arg)
    launchDescriptionObject.add_action(sim_time_arg)
    launchDescriptionObject.add_action(fake_joint_states_arg)
    launchDescriptionObject.add_action(joint_state_broadcaster_after_spawn)
    launchDescriptionObject.add_action(arm_controller_after_joint_states)
    launchDescriptionObject.add_action(home_robot_after_controller)
    launchDescriptionObject.add_action(attach_detach_controller_after_spawn)
    launchDescriptionObject.add_action(rviz_after_start)
    launchDescriptionObject.add_action(world_launch)
    launchDescriptionObject.add_action(gz_bridge_node)
    launchDescriptionObject.add_action(world_to_base_tf_node)
    launchDescriptionObject.add_action(robot_state_publisher_node)
    launchDescriptionObject.add_action(joint_state_publisher_node)
    launchDescriptionObject.add_action(spawn_urdf_node)
    launchDescriptionObject.add_action(gz_image_bridge_node)
    #launchDescriptionObject.add_action(relay_gripper_camera_info_node)
    launchDescriptionObject.add_action(relay_table_camera_info_node)
    #launchDescriptionObject.add_action(joint_state_publisher_gui_node)

    return launchDescriptionObject
