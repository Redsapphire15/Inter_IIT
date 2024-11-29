# Inter_IIT
I hope everyone has finished the already given instructions. 
Extract the models.zip folder and copy the entire folder and paste it in the /home/{$USER}/.gazebo folder

**NOTE: /.gazebo is a hidden folder. In order to make it viewable, access via terminal or press Ctrl+H in home directory**
**NOTE: inter_iit is the package. NOT THE ENTIRE THING**
Once you've done this,

$ ros2 launch inter_iit spawn_modified_8.launch.py

In another terminal

$ cd ~/inter_iit/scripts
$ python3 odom_to_frame_new.py
