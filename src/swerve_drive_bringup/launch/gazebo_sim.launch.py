import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='true',
        description='Run Gazebo in headless mode (server only, recommended for Foxglove Studio)',
    )
    headless = LaunchConfiguration('headless')

    pkg_share = FindPackageShare('swerve_drive_bringup')
    ros_gz_sim_share = FindPackageShare('ros_gz_sim')

    # Controller config
    controller_config = PathJoinSubstitution([pkg_share, 'config', 'swerve_controllers.yaml'])

    # Path to URDF xacro (with use_gazebo:=true)
    urdf_path = PathJoinSubstitution([pkg_share, 'urdf', 'swerve_robot.urdf.xacro'])
    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ', urdf_path,
        ' use_gazebo:=true',
        ' controller_yaml:=', controller_config,
    ])
    robot_description = {'robot_description': robot_description_content}

    # 0. Zenoh Router (zenohd)
    zenoh_router = ExecuteProcess(
        cmd=['zenohd', '-c', '/workspace/zenoh/zenoh_router.json5'],
        output='screen',
    )

    # 1. Gazebo Sim (Headless server-only or GUI)
    gazebo_sim_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([ros_gz_sim_share, 'launch', 'gz_sim.launch.py'])
        ),
        launch_arguments={'gz_args': '-s -r -v 3 empty.sdf'}.items(),
        condition=IfCondition(headless),
    )

    gazebo_sim_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([ros_gz_sim_share, 'launch', 'gz_sim.launch.py'])
        ),
        launch_arguments={'gz_args': '-r -v 3 empty.sdf'}.items(),
        condition=UnlessCondition(headless),
    )

    # 2. robot_state_publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description, {'use_sim_time': True}],
    )

    # 3. Spawn robot in Gazebo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-world', 'empty',
            '-name', 'swerve_robot',
            '-topic', 'robot_description',
            '-z', '0.1',
        ],
    )

    # 4. Bridge /clock from Gazebo to ROS 2
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    # 5. static transform (map -> odom)
    static_map_to_odom_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--frame-id', 'map', '--child-frame-id', 'odom'],
    )

    # 6. Spawners (joint_state_broadcaster & swerve_drive_controller)
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )

    swerve_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'swerve_drive_controller',
            '--param-file', controller_config,
            '--controller-manager', '/controller_manager',
        ],
    )

    delay_swerve_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[swerve_drive_controller_spawner],
        )
    )

    # 7. foxglove_bridge
    foxglove_bridge_node = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        parameters=[{
            'port': 8765,
            'address': '0.0.0.0',
            'send_buffer_limit': 100000000,
            'use_sim_time': True,
        }],
    )

    return LaunchDescription([
        zenoh_router,
        headless_arg,
        gazebo_sim_headless,
        gazebo_sim_gui,
        clock_bridge,
        static_map_to_odom_node,
        robot_state_publisher_node,
        spawn_robot,
        joint_state_broadcaster_spawner,
        delay_swerve_controller_spawner,
        foxglove_bridge_node,
    ])
