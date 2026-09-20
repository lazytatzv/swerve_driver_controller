import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    use_joy_arg = DeclareLaunchArgument(
        'use_joy',
        default_value='false',
        description='Enable joystick teleoperation nodes',
    )
    pkg_share = FindPackageShare('swerve_drive_bringup')

    # Path to URDF xacro
    urdf_path = PathJoinSubstitution([pkg_share, 'urdf', 'swerve_robot.urdf.xacro'])
    robot_description_content = Command([FindExecutable(name='xacro'), ' ', urdf_path])
    robot_description = {'robot_description': robot_description_content}

    # Controller config
    controller_config = PathJoinSubstitution([pkg_share, 'config', 'swerve_controllers.yaml'])
    rviz_config = PathJoinSubstitution([pkg_share, 'config', 'view_robot.rviz'])

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
            ('/swerve_drive_controller/odom', '/odom'),
        ],
    )

    # 2.1 static transform (map -> odom)
    static_map_to_odom_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--frame-id', 'map', '--child-frame-id', 'odom'],
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
        arguments=[
            'swerve_drive_controller',
            '--param-file', controller_config,
            '--controller-manager', '/controller_manager',
        ],
    )

    # 5. RViz2 node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
    )

    # 6. Joystick teleop nodes (PS5 controller, optional)
    joy_config = PathJoinSubstitution([pkg_share, 'config', 'teleop_ps5.yaml'])
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'dev': '/dev/input/js0',
            'deadzone': 0.05,
            'autorepeat_rate': 0.0,
        }],
        condition=IfCondition(LaunchConfiguration('use_joy')),
    )
    teleop_twist_joy_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[joy_config],
        condition=IfCondition(LaunchConfiguration('use_joy')),
    )

    # Delay swerve_drive_controller until joint_state_broadcaster starts
    delay_swerve_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[swerve_drive_controller_spawner],
        )
    )

    return LaunchDescription([
        use_joy_arg,
        static_map_to_odom_node,
        robot_state_publisher_node,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        delay_swerve_controller_spawner,
        rviz_node,
        joy_node,
        teleop_twist_joy_node,
    ])
