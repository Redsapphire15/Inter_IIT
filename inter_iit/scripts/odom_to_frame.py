import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

class OdometryToTFPublisher(Node):
    def __init__(self):
        super().__init__('odom_to_tf_publisher')
        
        # Create a TransformBroadcaster instance
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Subscribe to the /odom topic
        self.odom_subscriber = self.create_subscription(
            Odometry,
            'robot1_1/odom',  # This is where your robot's odometry data will come from
            self.odom_callback,
            10  # Queue size
        )

        # Initialize position and orientation
        self.position = None
        self.orientation = None

    def odom_callback(self, msg):
        # Update position and orientation based on Odometry message
        self.position = msg.pose.pose.position
        self.orientation = msg.pose.pose.orientation
        print(f'x:{self.position.x}, y:{self.position.y}, z:{self.position.z}')

    def publish_transform(self):
        print("hey I ran")
        if self.position is None or self.orientation is None:
            self.get_logger().warn("Waiting for odometry data...")
            return
        transform = TransformStamped()

        # Set the timestamp and parent/child frames
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'odom'  # Parent frame
        transform.child_frame_id = 'robot1_1/base_footprint'  # Child frame (robot's base)

        # Modify the position dynamically (+1 to x, +0.5 to y)
        transform.transform.translation.x = self.position.x # Add 1 to x
        transform.transform.translation.y = self.position.y  # Add 0.5 to y
        transform.transform.translation.z = self.position.z  # Keep z the same

        # Set rotation (yaw, pitch, roll)
        transform.transform.rotation.x = self.orientation.x
        transform.transform.rotation.y = self.orientation.y
        transform.transform.rotation.z = self.orientation.z
        transform.transform.rotation.w = self.orientation.w

        # Publish the transform
        self.tf_broadcaster.sendTransform(transform)
        print("Hei")

def main(args=None):
    rclpy.init(args=args)
    node = OdometryToTFPublisher()

    # Run the loop to update and publish transform
    rate = node.create_rate(10)  # Loop at 10 Hz (10 times per second)
    
    try:
        while True:
            node.publish_transform()  # Update the transform based on the latest odom data
            rclpy.spin_once(node)  # Ensure callbacks are handled
            rate.sleep()  # Maintain the loop rate
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
