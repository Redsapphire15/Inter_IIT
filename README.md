# Inter_IIT
Contains the files for Inter-IIT 2025 BharatForge

Follow the below instructions to spawn 4 turtlebots in a warehouse

$ sudo apt install ros-humble-navigation2

$ sudo apt install ros-humble-nav2-bringup

$ sudo apt install ros-humble-turtlebot3-gazebo


$ source /opt/ros/humble/setup.bash

$ export TURTLEBOT3_MODEL=waffle  # Iron and older only with Gazebo Classic

In the spawn_4_bots.py, change the line,
declare_world_cmd = DeclareLaunchArgument(
        'world', default_value=os.path.join('/home/kavin', 'world2.world'),
        description='World file'
    )
**from '/home/kavin' to '/home/{your username}/{wherever your world file is}'**

Extract the models.zip folder and copy the entire folder and paste it in the /home/{$USER}/.gazebo folder

**NOTE: /.gazebo is a hidden folder. In order to make it viewable, access via terminal or press Ctrl+H in home directory**

Once you've done this,
$ sudo mv spawn_4_bots.py /opt/ros/humble/share/nav2_bringup/launch/spawn_4_bots.py
$ ros2 launch nav2_bringup spawn_4_bots.py
