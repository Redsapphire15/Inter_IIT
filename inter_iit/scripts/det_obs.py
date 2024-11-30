#!/usr/bin/python3

import rclpy
import yaml
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration
import torch
import numpy as np
from sklearn.cluster import DBSCAN
from tf_transformations import euler_from_quaternion
from ament_index_python.packages import get_package_share_directory
import os

class ObstacleDetector(Node):
    def __init__(self):
        super().__init__('obstacle_detector')

        # Initialize dictionaries to store data for multiple robots
        self.laser_data = {}
        self.odom_data = {}

        # Global database for obstacles
        self.global_obstacles = []

        # DBSCAN parameters for clustering points into obstacles
        self.dbscan_eps = 0.5  # Maximum distance between two samples for one to be considered as in the neighborhood of the other
        self.dbscan_min_samples = 5  # Minimum number of samples in a neighborhood for a point to be considered as a core point

        # Initialize subscribers and publishers
        self.marker_publisher = self.create_publisher(MarkerArray, 'obstacle_markers', 10)

        # Load robot configurations from the YAML file
        self.load_robot_configurations()

        # Create a timer to update markers regularly
        self.create_timer(1.0, self.update_and_publish_markers)

    def load_robot_configurations(self):
        bringup_dir = get_package_share_directory('inter_iit')
        yaml_file = os.path.join(bringup_dir, 'config', 'robots.yaml')
        with open(yaml_file, 'r') as file:
            robots_config = yaml.safe_load(file)

        for robot_type, robot_list in robots_config.items():
            for i, robot in enumerate(robot_list):
                robot_name = robot['name']
                scan_topic = f'/{robot_name}/{robot_name}/scan'
                odom_topic = f'/{robot_name}/odom'
                self.create_subscription(LaserScan, scan_topic, lambda msg, topic=robot_name: self.laser_callback(msg, topic), 10)
                self.create_subscription(Odometry, odom_topic, lambda msg, topic=robot_name: self.odom_callback(msg, topic), 10)

    def odom_callback(self, msg, topic):
        # Extract the robot name from the topic
        robot_name = topic  # The robot name is passed directly
        self.odom_data[robot_name] = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'theta': euler_from_quaternion([msg.pose.pose.orientation.x,
                                            msg.pose.pose.orientation.y,
                                            msg.pose.pose.orientation.z,
                                            msg.pose.pose.orientation.w])[2]
        }

    def laser_callback(self, msg, topic):
        # Extract the robot name from the topic
        robot_name = topic  # The robot name is passed directly
        self.laser_data[robot_name] = {
            'ranges': torch.tensor(msg.ranges),
            'angle_min': msg.angle_min,
            'angle_increment': msg.angle_increment
        }

        # Process data for the current robot
        self.process_data(robot_name)

    def process_data(self, robot_name):
        if robot_name not in self.laser_data or robot_name not in self.odom_data:
            return  # Ensure both laser and odometry data are available

        # Extract laser scan and odometry data
        laser_data = self.laser_data[robot_name]
        odom_data = self.odom_data[robot_name]

        ranges = laser_data['ranges']
        angle_min = laser_data['angle_min']
        angle_increment = laser_data['angle_increment']

        # Filter out infinite values
        valid_indices = ~torch.isinf(ranges)
        ranges = ranges[valid_indices]

        # Calculate the angles of all valid LIDAR points
        angles = angle_min + np.arange(len(laser_data['ranges'])) * angle_increment
        angles = torch.tensor(angles, dtype=torch.float32)
        angles = angles[valid_indices]  # Apply valid indices to angles

        # Convert LIDAR points to Cartesian coordinates in the LIDAR frame
        x_lidar = ranges * torch.cos(angles)
        y_lidar = ranges * torch.sin(angles)

        # Transform points to the global frame
        cos_theta = torch.cos(torch.tensor(odom_data['theta'], dtype=torch.float32))
        sin_theta = torch.sin(torch.tensor(odom_data['theta'], dtype=torch.float32))
        x_global = odom_data['x'] + x_lidar * cos_theta - y_lidar * sin_theta
        y_global = odom_data['y'] + x_lidar * sin_theta + y_lidar * cos_theta

        # Append new points to the global obstacle list without duplicates
        new_obstacles = np.vstack((x_global.cpu().numpy(), y_global.cpu().numpy())).T
        for point in new_obstacles:
            if not any((point == existing_point).all() for existing_point in self.global_obstacles):
                self.global_obstacles.append(point)

    def update_and_publish_markers(self):
        # Apply clustering to merge close obstacles and remove duplicates
        self.update_global_obstacles()
        
        # Publish markers for visualization in RViz
        self.publish_markers()

    def update_global_obstacles(self):
        all_obstacles = np.array(self.global_obstacles)
        if len(all_obstacles) > 0:
            # Remove NaN values
            all_obstacles = all_obstacles[~np.isnan(all_obstacles).any(axis=1)]
            
            clustering = DBSCAN(eps=self.dbscan_eps, min_samples=1).fit(all_obstacles)
            unique_labels = set(clustering.labels_)
            merged_obstacles = []

            for label in unique_labels:
                if label != -1:
                    class_member_mask = (clustering.labels_ == label)
                    xy = all_obstacles[class_member_mask]
                    obstacle_center = np.mean(xy, axis=0)
                    
                    # Calculate the length of the obstacle
                    distances = np.sqrt(np.sum(np.diff(xy, axis=0) ** 2, axis=1))
                    total_length = np.sum(distances)

                    # Determine the number of segments
                    num_segments = int(total_length // self.dbscan_eps)
                    remainder = total_length % self.dbscan_eps
                    
                    # If there's a remainder, we have an additional segment
                    if remainder > 0:
                        num_segments += 1

                    # Place markers at the centers of each segment
                    segment_lengths = np.cumsum(distances)
                    start_index = 0
                    for i in range(num_segments):
                        if i < num_segments - 1 or remainder == 0:
                            end_index = np.searchsorted(segment_lengths, (i + 1) * self.dbscan_eps)
                        else:
                            end_index = len(segment_lengths)  # Include the remainder segment.

                        if start_index < end_index:  # Ensure valid segment indices
                            segment = xy[start_index:end_index]
                            if len(segment) > 0:
                                segment_center = np.mean(segment, axis=0)
                                merged_obstacles.append(segment_center)
                        
                        start_index = end_index

            self.global_obstacles = merged_obstacles

    def publish_markers(self):
        marker_array = MarkerArray()
        for i, (x, y) in enumerate(self.global_obstacles):
            marker = Marker()
            marker.header.frame_id = "odom"  # Use a common frame like map or odom
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = f"obstacles"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = float(x)
            marker.pose.position.y = float(y)
            marker.pose.position.z = 0.0
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.3  # Adjusted scale for better visibility
            marker.scale.y = 0.3
            marker.scale.z = 0.3
            marker.color.a = 1.0  # Fully opaque
            marker.color.r = 0.0  # Adjust color for better visibility
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.lifetime = Duration(sec=0, nanosec=0)  # Set lifetime to 0 for indefinite duration
            marker_array.markers.append(marker)
        self.marker_publisher.publish(marker_array)
        self.get_logger().info("Obstacle markers updated")

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
