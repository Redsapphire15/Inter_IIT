import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get the package directory
    package_dir = get_package_share_directory('inter_iit')  # Replace 'inter_iit' with your package name

    # Path to the world file
    default_world_path = os.path.join(package_dir, 'worlds', 'new_world_2.world')  # Replace 'default.world' as needed

    # Declare the argument for the world file
    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=default_world_path,
        description='Path to the world file to be loaded'
    )

    # Start Gazebo server
    start_gazebo_server_cmd = ExecuteProcess(
        cmd=['gzserver', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', LaunchConfiguration('world')],
        output='screen'
    )

    # Optionally start Gazebo client
    start_gazebo_client_cmd = ExecuteProcess(
        cmd=['gzclient'],
        output='screen'
    )

    # Create the launch description
    ld = LaunchDescription()

    # Add the world argument
    ld.add_action(declare_world_cmd)

    # Add commands to launch Gazebo
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)

    return ld

