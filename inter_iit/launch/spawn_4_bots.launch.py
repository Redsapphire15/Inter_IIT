"""This is all-in-one launch script intended for use by nav2 developers."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('inter_iit')
    launch_dir = os.path.join(bringup_dir, 'launch')

    # Paths
    urdf_file = os.path.join(bringup_dir, 'urdf', 'turtlebot3_waffle.urdf')
    sdf_file = os.path.join(bringup_dir, 'worlds', 'waffle.model')

    with open(urdf_file, 'r') as infp:
        robot_description = infp.read()

    # Declare arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use simulation clock'
    )
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz', default_value='true', description='Start RVIZ'
    )
    declare_world_cmd = DeclareLaunchArgument(
        'world', default_value=os.path.join(bringup_dir,'worlds', 'world_easy.world'),
        description='World file'
    )

    # Robot spawn configurations (add more as needed)
    robot_positions = [
        {'name': 'robot1', 'x': '0.5', 'y': '2.5', 'z': '0.01'},
        {'name': 'robot2', 'x': '8.5', 'y': '1.5', 'z': '0.01'},
        {'name': 'robot3', 'x': '8.5', 'y': '8.5', 'z': '0.01'},
        {'name': 'robot4', 'x': '0.5', 'y': '8.0', 'z': '0.01'}
    ]

    spawn_robots = []

    for robot in robot_positions:
        namespace = robot['name']

        # Group actions to separate namespace
        spawn_robot = GroupAction([
            PushRosNamespace(namespace),

            # Robot State Publisher
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[{'use_sim_time': True, 'robot_description': robot_description}]
            ),

            # Spawn robot in Gazebo
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                name=f'spawn_{namespace}',
                output='screen',
                arguments=[
                    '-entity', namespace,
                    '-file', sdf_file,
                    '-robot_namespace', namespace,
                    '-x', robot['x'], '-y', robot['y'], '-z', robot['z']
                ]
            ),
            
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name=f'static_tf_pub_{namespace}',
                output='screen',
                arguments=[
                    robot['x'], robot['y'], robot['z'], '0', '0', '0',  # Transform (x, y, z, roll, pitch, yaw)
                    'map',                         # Parent frame
                    f'base_footprint'  # Child frame
                ]
            )
            
        ])

        spawn_robots.append(spawn_robot)

    # Start Gazebo
    start_gazebo_server_cmd = ExecuteProcess(
        cmd=['gzserver', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', LaunchConfiguration('world')],
        cwd=[launch_dir],
        output='screen'
    )

    start_gazebo_client_cmd = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        cmd=['gzclient'],
        cwd=[launch_dir],
        output='screen'
    )

    # RVIZ
    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'rviz_launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )

    # Create launch description
    ld = LaunchDescription()

    # Declare arguments
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_world_cmd)

    # Add Gazebo and RVIZ
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)
    ld.add_action(rviz_cmd)

    # Add robot spawns
    for spawn_robot in spawn_robots:
        ld.add_action(spawn_robot)

    return ld

