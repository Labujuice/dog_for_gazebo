import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_args = [
        DeclareLaunchArgument(
            'joy_dev',
            default_value='/dev/input/js0',
            description='Joystick device path'
        ),
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Whether to launch Gazebo GUI'
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='false',
            description='Whether to launch RViz'
        ),
        DeclareLaunchArgument(
            'robot_name',
            default_value='robot_1',
            description='Namespace for the Go2 robot'
        ),
        DeclareLaunchArgument(
            'world',
            default_value='clean_world.sdf',
            description='World SDF file name'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock'
        ),
    ]

    joy_dev = LaunchConfiguration('joy_dev')
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    robot_name = LaunchConfiguration('robot_name')
    world = LaunchConfiguration('world')
    use_sim_time = LaunchConfiguration('use_sim_time')

    go2_sim_pkg = FindPackageShare('go2_sim')
    config_filepath = PathJoinSubstitution([
        go2_sim_pkg, 'config', 'ps4_go2.config.yaml'
    ])

    # 1. Joy Node (reads PS4 DualShock 4 from /dev/input/js0)
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'device_id': 0,
            'device_name': '',
            'deadzone': 0.05,
            'autorepeat_rate': 20.0,
        }],
        output='screen',
    )

    # 2. Teleop Twist Joy Node (maps sticks to /cmd_vel)
    teleop_twist_joy_node = Node(
        package='teleop_twist_joy',
        executable='teleop_twist_joy_node',
        name='teleop_twist_joy_node',
        parameters=[config_filepath, {'use_sim_time': use_sim_time}],
        remappings=[('cmd_vel', '/cmd_vel')],
        output='screen',
    )

    # 3. Clean Simulation + Quad-SDK Control Stack + Teleop Mapper
    clean_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([go2_sim_pkg, 'launch', 'go2_clean.launch.py'])
        ),
        launch_arguments={
            'gui': gui,
            'rviz': rviz,
            'robot_name': robot_name,
            'world': world,
            'use_sim_time': use_sim_time,
        }.items(),
    )

    return LaunchDescription(declared_args + [
        joy_node,
        teleop_twist_joy_node,
        clean_sim_launch,
    ])
