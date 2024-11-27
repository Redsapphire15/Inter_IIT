import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Declare the namespace argument
    declare_namespace = DeclareLaunchArgument(
        'ns', default_value='robot1',
        description='Namespace for the robot'
    )

    # Declare the URDF file path
    urdf_file_path = os.path.join(
        '/home/kavin/ros_ws/src/inter_iit',
        'urdf',
        'robot_model.urdf'
    )

    # Load URDF
    with open(urdf_file_path, 'r') as urdf_file:
        robot_description = urdf_file.read()

    # Get the namespace
    namespace = LaunchConfiguration('ns')

    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=namespace,
        parameters=[{'robot_description': robot_description}]
    )

    # Spawn Entity Node for Gazebo
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        namespace=namespace,
        output='screen',
        arguments=[
            '-entity', namespace,
            '-x', '0', '-y', '0', '-z', '0.01',
            '-robot_namespace', namespace,
            '-file', urdf_file_path
        ]
    )

    # Create LaunchDescription
    ld = LaunchDescription()

    # Add actions
    ld.add_action(declare_namespace)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(spawn_entity_node)

    return ld

