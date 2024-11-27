import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import random
import yaml
import os
import time
from ament_index_python.packages import get_package_share_directory

class ObstacleMover(Node):
    def __init__(self):
        super().__init__('obstacle_mover')
        bringup_dir = get_package_share_directory('inter_iit')
        # Create publishers for each obstacle's cmd_vel
        self.cmd_vel_publishers = []  # Renamed to avoid potential conflicts
        yaml_file = os.path.join(bringup_dir, 'config', 'robots.yaml')
        with open(yaml_file, 'r') as file:
            print("went inside this")
            robots_config = yaml.safe_load(file)

        for i in range(len(robots_config['robot2'])):  # Assuming there are 4 obstacles: obs0, obs1, obs2, obs3
            publisher = self.create_publisher(Twist, f'robot2_{i+1}/cmd_vel', 10)
            self.cmd_vel_publishers.append(publisher)

    def move_obstacles(self):
        while rclpy.ok():
            for publisher in self.cmd_vel_publishers:
                # Create a new Twist message
                msg = Twist()
                
                # Random linear x velocity between 0 and 2.0
                msg.linear.x = random.uniform(0.0, 2.0)
                
                # Random angular z velocity with mu=0 and sigma=pi/4
                msg.angular.z = random.gauss(0, 3.14 / 4)  # pi/4 is approximately 0.7854
                
                # Publish the command
                publisher.publish(msg)
                
                # Log the command for debugging
                self.get_logger().info(f'Published cmd_vel linear.x={msg.linear.x:.2f}, angular.z={msg.angular.z:.2f}')
            
            # Sleep for a short duration before the next command
            time.sleep(1)  # Adjust as needed for frequency

def main(args=None):
    rclpy.init(args=args)
    obstacle_mover = ObstacleMover()
    
    try:
        obstacle_mover.move_obstacles()
    except KeyboardInterrupt:
        pass
    finally:
        obstacle_mover.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
