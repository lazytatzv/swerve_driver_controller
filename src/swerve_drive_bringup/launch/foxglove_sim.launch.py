import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare('swerve_drive_bringup')

    # Path to URDF xacro
    urdf_path = PathJoinSubstitution([pkg_share, 'urdf', 'swerve_robot.urdf.xacro'])
    # Process xacro into robot_description
    robot_description_content = Command([FindExecutable(name='xacro'), ' ', urdf_path])
    robot_description = {'robot_description': robot_description_content}

    # Controller config
    controller_config = PathJoinSubstitution([pkg_share, 'config', 'swerve_controllers.yaml'])

    # 1. robot_state_publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description],
    )

    # 2. controller_manager (ros2_control_node)
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controller_config],
        output='both',
        remappings=[
            ('/swerve_drive_controller/cmd_vel', '/cmd_vel'),
        ],
    )

    # 3. joint_state_broadcaster spawner
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )

    # 4. swerve_drive_controller spawner (start after joint_state_broadcaster)
    swerve_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['swerve_drive_controller', '--controller-manager', '/controller_manager'],
    )

    # 5. foxglove_bridge node
    foxglove_bridge_node = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        parameters=[{
            'port': 8765,
            'address': '0.0.0.0',
            'send_buffer_limit': 100000000,
        }],
    )

    # Delay swerve_drive_controller until joint_state_broadcaster starts
    delay_swerve_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[swerve_drive_controller_spawner],
        )
    )

    return LaunchDescription([
        robot_state_publisher_node,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        delay_swerve_controller_spawner,
        foxglove_bridge_node,
    ])
