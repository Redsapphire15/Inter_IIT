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

    # Paths for robot URDFs and SDFs
    robot_urdfs = [
        os.path.join(bringup_dir, 'urdf', 'robot1.urdf'),
        os.path.join(bringup_dir, 'urdf', 'robot2.urdf'),
        os.path.join(bringup_dir, 'urdf', 'robot3.urdf'),
        os.path.join(bringup_dir, 'urdf', 'robot4.urdf'),
        os.path.join(bringup_dir, 'urdf', 'box1.urdf'),
        os.path.join(bringup_dir, 'urdf', 'box2.urdf'),
        os.path.join(bringup_dir, 'urdf', 'box3.urdf')
    ]

    sdf_file = os.path.join(bringup_dir, 'models', 'waffle.model')
    sdf_file_2 = os.path.join(bringup_dir, 'models', 'model_box.sdf')
    # Load robot descriptions
    robot_descriptions = []
    for urdf_path in robot_urdfs:
        with open(urdf_path, 'r') as urdf_file:
            robot_descriptions.append(urdf_file.read())

    # Load robot spawn configurations from YAML
    yaml_file = os.path.join(bringup_dir, 'config', 'robots.yaml')
    with open(yaml_file, 'r') as file:
        robots_config = yaml.safe_load(file)

    # Create spawn actions
    spawn_robots = []

    def spawn_robot_group(robot_name, robot_type_index, x, y, z, m):
        """Creates a group action to spawn a robot."""
        namespace = robot_name
        robot_description = robot_descriptions[robot_type_index]
	if m == 0:
		sdf_file = os.path.join(bringup_dir, 'models', 'waffle.model')
	else:
		sdf_file = os.path.join(bringup_dir, 'models', 'model_box.sdf')


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
                    '-file', sdf_file,
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
                    f'{namespace}/base_footprint'  # Child frame
                ]
            )
        ])

    # Spawn each robot type based on the YAML configuration
    for i, robot_type in enumerate(robots_config.keys()):
        for j, robot in enumerate(robots_config[robot_type]):
            m = 0
            robot_name = f'{robot_type}_{j+1}' 
            if j+1> 4:
            	m = 1
            spawn_robots.append(spawn_robot_group(robot_name, i, robot['x'], robot['y'], robot['z']))

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
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation clock'))
    ld.add_action(DeclareLaunchArgument('use_rviz', default_value='true', description='Start RVIZ'))
    ld.add_action(DeclareLaunchArgument('world', default_value=os.path.join(bringup_dir, 'worlds', 'world_easy.world'), description='World file'))

    # Add Gazebo and RVIZ
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)
    ld.add_action(rviz_cmd)

    # Add robot spawns
    for spawn_robot in spawn_robots:
        ld.add_action(spawn_robot)

    return ld

