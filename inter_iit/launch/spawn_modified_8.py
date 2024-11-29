import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace


def generate_launch_description():
    # Get the package directory
    bringup_dir = get_package_share_directory('inter_iit')
    launch_dir = os.path.join(bringup_dir, 'launch')

    # Paths for URDFs and SDFs
    urdf_paths = [
        os.path.join(bringup_dir, 'urdf', 'robot1.urdf'),
        os.path.join(bringup_dir, 'urdf', 'robot2.urdf'),
        os.path.join(bringup_dir, 'urdf', 'robot3.urdf'),
        os.path.join(bringup_dir, 'urdf', 'robot4.urdf'),
        os.path.join(bringup_dir, 'urdf', 'box_1.urdf'),
        os.path.join(bringup_dir, 'urdf', 'box_2.urdf'),
        os.path.join(bringup_dir, 'urdf', 'box_3.urdf')
    ]
    
    sdf_paths = [
        os.path.join(bringup_dir, 'models', 'mybot1.sdf'),
        os.path.join(bringup_dir, 'models', 'mybot2.sdf'),
        os.path.join(bringup_dir, 'models', 'mybot3.sdf'),
        os.path.join(bringup_dir, 'models', 'mybot4.sdf'),
        os.path.join(bringup_dir, 'models', 'model_box.sdf'),
        os.path.join(bringup_dir, 'models', 'model_box.sdf'),
        os.path.join(bringup_dir, 'models', 'model_box.sdf')
    ]
    	
    print("1")
    #sdf_path_1 = os.path.join(bringup_dir, 'models', 'waffle.model')
    print("helo")
    #sdf_path_2 = os.path.join(bringup_dir, 'models', 'model_box.sdf')
    print("hello")
    # Load robot descriptions
    robot_descriptions = []
    for urdf_path in urdf_paths:
        with open(urdf_path, 'r') as urdf_file:
            robot_descriptions.append(urdf_file.read())
    print("helo")
    yaml_file = os.path.join(bringup_dir, 'config', 'robots.yaml')

    with open(yaml_file, 'r') as file:
        robots_config = yaml.safe_load(file)

    # Dynamically spawn all robots based on YAML configuration
    spawn_robots = []
    def spawn_robot_group(robot_name, robot_type_index, x, y, z, m):
        """Creates a group action to spawn a robot."""
        namespace = robot_name
        robot_description = robot_descriptions[robot_type_index]
        #sdf_file = sdf_path_1 if m == 1 else sdf_path_2

        if m == 1:
            return GroupAction([
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
                        '-file', sdf_paths[robot_type_index],
                        '-robot_namespace', namespace,
                        '-x', str(x), '-y', str(y), '-z', str(z)
                    ]
                ),

                Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    name=f'static_tf_pub_{namespace}',
                    output='screen',
                    arguments=[
                        str(x), str(y), str(z), '0', '0', '0',  # Transform (x, y, z, roll, pitch, yaw)
                        'map',                         # Parent frame
                        f'{namespace}/odom'  # Child frame
                    ]
                ),
                Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    name=f'static_tf_pub1_{namespace}',
                    output='screen',
                    arguments=[
                        str(x), str(y), str(z), '0', '0', '0',  # Transform (x, y, z, roll, pitch, yaw)
                        f'{namespace}/odom',                         # Parent frame
                        f'{namespace}/base_footprint'  # Child frame
                    ]
                )
            ])
        else:
            return GroupAction([
                PushRosNamespace(namespace),
                 Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    name=f'spawn_{namespace}',
                    output='screen',
                    arguments=[
                        '-entity', namespace,
                        '-file', sdf_paths[robot_type_index],
                        '-robot_namespace', namespace,
                        '-x', str(x), '-y', str(y), '-z', str(z)
                    ]
                ),

                Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    name=f'static_tf_pub_{namespace}',
                    output='screen',
                    arguments=[
                        str(x), str(y), str(z), '0', '0', '0',  # Transform (x, y, z, roll, pitch, yaw)
                        'map',                         # Parent frame
                        f'{namespace}/odom'  # Child frame
                    ]
                ),
                Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    name=f'static_tf_pub1_{namespace}',
                    output='screen',
                    arguments=[
                        str(x), str(y), str(z), '0', '0', '0',  # Transform (x, y, z, roll, pitch, yaw)
                        f'{namespace}/odom',                         # Parent frame
                        f'{namespace}/base_footprint'  # Child frame
                    ]
                )
            ])


    # Dynamically spawn all robots based on YAML configuration
    for robot_type, robot_list in robots_config.items():
        print(robot_type)
        for i, robot in enumerate(robot_list):
            if robot_type == 'robot1':  # Handle robot1 type
                spawn_robots.append(spawn_robot_group(robot['name'], i, robot['x'], robot['y'], robot['z'], 1))
            elif robot_type == 'robot2':  # Handle robot2 type
                spawn_robots.append(spawn_robot_group(robot['name'], i+4, robot['x'], robot['y'], robot['z'], 2))

    print("ddjkajdnajdnjka")
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
    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'rviz_launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )

    # Create launch description
    ld = LaunchDescription()

    # Declare arguments
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation clock'))
    ld.add_action(DeclareLaunchArgument('use_rviz', default_value='true', description='Start RVIZ'))
    ld.add_action(DeclareLaunchArgument('world', default_value=os.path.join(bringup_dir, 'worlds', 'world_easy.world'), description='World file'))

    # Add Gazebo and RVIZ
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)
    ld.add_action(rviz_cmd)
    print("hdiei")
    # Add robot spawns
    for spawn_robot in spawn_robots:
        print(spawn_robot)
        ld.add_action(spawn_robot)

    return ld

