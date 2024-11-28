import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import random
import time
import yaml
import os
from ament_index_python.packages import get_package_share_directory

class ObstacleMover(Node):
    def __init__(self):
        super().__init__('obstacle_mover')
        bringup_dir = get_package_share_directory('inter_iit')
        
        # Initialize lists for publishers and subscribers
        self.cmd_vel_publishers = []
        self.odom_subscribers = []  # Initialize odom_subscribers here
        self.obstacle_positions = {}  # Initialize dictionary for positions
        self.obstacle_timestamps = {}  # Initialize dictionary for timestamps
        self.initial_velocities = [self.random_velocity() for _ in range(4)]  # Initialize dictionary for initial velocities
        
        yaml_file = os.path.join(bringup_dir, 'config', 'robots.yaml')
        with open(yaml_file, 'r') as file:
            print("went inside this")
            robots_config = yaml.safe_load(file)
        


        for i in range(len(robots_config['robot2'])):
            publisher = self.create_publisher(Twist, f'robot2_{i}/cmd_vel', 10)
            self.cmd_vel_publishers.append(publisher)
            
            # Subscribe to the odometry topic of each obstacle
            odom_subscriber = self.create_subscription(Odometry, f'robot2_{i}/odom', lambda msg, i=i: self.odom_callback(msg, i), 10)
            self.odom_subscribers.append(odom_subscriber)

            # Initialize position and timestamp
            self.obstacle_positions[i] = (0.0, 0.0)  # Initial position (x, y)
            self.obstacle_timestamps[i] = time.time()

            # Publish initial velocity
            self.publish_velocity(i, self.initial_velocities[i])

    def random_velocity(self):
        """Generate a random velocity for obstacles."""
        velocity = Twist()
        velocity.linear.x = random.uniform(-1.5, 1.5)
        velocity.linear.y = 0.0
        velocity.linear.z = 0.0
        velocity.angular.x = 0.0
        velocity.angular.y = 0.0
        velocity.angular.z = random.uniform(-3.1415 / 4, 3.1415 / 4)
        return velocity

    def odom_callback(self, msg, index):
        """Callback function to handle odometry messages."""
        current_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        previous_position = self.obstacle_positions[index]
        
        # Check if the obstacle has moved within tolerance
        if abs(current_position[0] - previous_position[0]) < 0.1 and abs(current_position[1] - previous_position[1]) < 0.1:
            if time.time() - self.obstacle_timestamps[index] > 5:
                # If stationary for 10 seconds, reverse direction
                reverse_velocity = Twist()
                if self.initial_velocities[index].linear.x >= 0:
                    reverse_velocity.linear.x = -random.uniform(0.5, 1.5)  # Negative value
                else:
                    reverse_velocity.linear.x = random.uniform(0.5, 1.5)   # Positive value
                self.initial_velocities[index].linear.x = reverse_velocity.linear.x
                reverse_velocity.angular.z = random.uniform(-3.141592 / 4, 3.141592 / 4)
                self.initial_velocities[index].angular.z = reverse_velocity.angular.z

                self.publish_velocity(index, reverse_velocity)
                self.get_logger().info(f'Obstacle {index} stationary; reversing velocity.')
                
                # Reset timestamp after reversing
                self.obstacle_timestamps[index] = time.time()
        else:
            # Update position and reset timestamp if moved
            self.obstacle_positions[index] = current_position
            self.obstacle_timestamps[index] = time.time()

    def publish_velocity(self, index, velocity):
        """Publish the given velocity to the specified obstacle."""
        self.cmd_vel_publishers[index].publish(velocity)
        self.get_logger().info(f'Published cmd_vel for obs{index}: linear.x={velocity.linear.x:.2f}, angular.z={velocity.angular.z:.2f}')

def main(args=None):
    rclpy.init(args=args)
    obstacle_mover = ObstacleMover()
    
    try:
        rclpy.spin(obstacle_mover)  # Keep the node running and processing callbacks
    except KeyboardInterrupt:
        pass
    finally:
        obstacle_mover.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
