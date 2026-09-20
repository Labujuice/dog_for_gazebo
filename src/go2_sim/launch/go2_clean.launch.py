import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_args = [
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
        DeclareLaunchArgument(
            'init_pose',
            default_value='-x 0.0 -y 0.0 -z 0.4',
            description='Initial spawn pose for Go2 (Z >= 0.35m to prevent ground clipping)'
        ),
    ]

    robot_name = LaunchConfiguration('robot_name')
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    world = LaunchConfiguration('world')
    use_sim_time = LaunchConfiguration('use_sim_time')
    init_pose = LaunchConfiguration('init_pose')

    quad_utils_pkg = FindPackageShare('quad_utils')

    # 1. Gazebo + Robot Bringup (gz_sim, clock bridge, ros2_control, robot_driver)
    # Construct robot_configs JSON for Go2
    # Note: quad_gazebo.py parses robot_configs and includes robot_bringup.py
    robot_configs_arg = [
        '[{"name": "robot_1", "type": "go2", "controller": "inverse_dynamics", "init_pose": "-x 0.0 -y 0.0 -z 0.4"}]'
    ]

    quad_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([quad_utils_pkg, 'launch', 'quad_gazebo.py'])
        ),
        launch_arguments={
            'world': world,
            'gui': gui,
            'rviz': rviz,
            'use_sim_time': use_sim_time,
            'robot_configs': robot_configs_arg,
        }.items(),
    )

    # 2. Planning Stack: Local Planner (NMPC) + Body Force Estimator
    planning_launch = GroupAction([
        PushRosNamespace(robot_name),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([quad_utils_pkg, 'launch', 'planning.py'])
            ),
            launch_arguments={
                'robot_type': 'go2',
                'namespace': robot_name,
                'reference': 'twist',
                'controller_mode': 'mpc',
                'twist_input': 'none',
                'use_sim_time': use_sim_time,
                'logging': 'false',
            }.items(),
        )
    ])

    # 3. Teleop Mapper Node (FSM manager, posture adjuster, joy/twist mapper, topic relays)
    teleop_mapper_node = Node(
        package='go2_sim',
        executable='teleop_mapper',
        name='teleop_mapper',
        output='screen',
        parameters=[{
            'robot_namespace': robot_name,
            'use_sim_time': use_sim_time,
            'max_vx': 0.6,
            'max_vy': 0.4,
            'max_wz': 1.0,
            'turbo_scale': 1.5,
            'auto_recovery': True,
        }],
    )

    return LaunchDescription(declared_args + [
        quad_gazebo_launch,
        planning_launch,
        teleop_mapper_node,
    ])
