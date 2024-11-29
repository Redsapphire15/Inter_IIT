import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class OdometryToTF(Node):
    def __init__(self):
        super().__init__('odometry_to_tf')
        
        # Create a tf2 broadcaster
        self.broadcaster = TransformBroadcaster(self)
        
        # Create a subscriber to robot1/odom
        self.odom1_subscriber = self.create_subscription(
            Odometry,
            '/robot1_1/odom',
            self.odom1_callback,
            10
        )

        self.odom1_subscriber = self.create_subscription(
            Odometry,
            '/robot1_2/odom',
            self.odom2_callback,
            10
        )

        self.odom1_subscriber = self.create_subscription(
            Odometry,
            '/robot1_3/odom',
            self.odom3_callback,
            10
        )

        self.odom1_subscriber = self.create_subscription(
            Odometry,
            '/robot1_4/odom',
            self.odom4_callback,
            10
        )
        
        # Initialize the transform message
        self.transform = TransformStamped()

    def odom1_callback(self, msg):
        print("Hi")
        # Extract odometry position and orientation data
        odom_frame = 'odom'
        base_link_frame = 'robot1_1/base_footprint'

        self.transform.header.stamp = self.get_clock().now().to_msg()
        self.transform.header.frame_id = odom_frame
        self.transform.child_frame_id = base_link_frame

        self.transform.transform.translation.x = msg.pose.pose.position.x
        self.transform.transform.translation.y = msg.pose.pose.position.y
        self.transform.transform.translation.z = msg.pose.pose.position.z

        self.transform.transform.rotation = msg.pose.pose.orientation

        # Broadcast the transform
        self.broadcaster.sendTransform(self.transform)

    def odom2_callback(self, msg):
        print("Hi")
        # Extract odometry position and orientation data
        odom_frame = 'odom'
        base_link_frame = 'robot1_2/base_footprint'

        self.transform.header.stamp = self.get_clock().now().to_msg()
        self.transform.header.frame_id = odom_frame
        self.transform.child_frame_id = base_link_frame

        self.transform.transform.translation.x = msg.pose.pose.position.x
        self.transform.transform.translation.y = msg.pose.pose.position.y
        self.transform.transform.translation.z = msg.pose.pose.position.z

        self.transform.transform.rotation = msg.pose.pose.orientation

        # Broadcast the transform
        self.broadcaster.sendTransform(self.transform)

    def odom3_callback(self, msg):
        print("Hi")
        # Extract odometry position and orientation data
        odom_frame = 'odom'
        base_link_frame = 'robot1_3/base_footprint'

        self.transform.header.stamp = self.get_clock().now().to_msg()
        self.transform.header.frame_id = odom_frame
        self.transform.child_frame_id = base_link_frame

        self.transform.transform.translation.x = msg.pose.pose.position.x
        self.transform.transform.translation.y = msg.pose.pose.position.y
        self.transform.transform.translation.z = msg.pose.pose.position.z

        self.transform.transform.rotation = msg.pose.pose.orientation

        # Broadcast the transform
        self.broadcaster.sendTransform(self.transform)

    def odom4_callback(self, msg):
        print("Hi")
        # Extract odometry position and orientation data
        odom_frame = 'odom'
        base_link_frame = 'robot1_4/base_footprint'

        self.transform.header.stamp = self.get_clock().now().to_msg()
        self.transform.header.frame_id = odom_frame
        self.transform.child_frame_id = base_link_frame

        self.transform.transform.translation.x = msg.pose.pose.position.x
        self.transform.transform.translation.y = msg.pose.pose.position.y
        self.transform.transform.translation.z = msg.pose.pose.position.z

        self.transform.transform.rotation = msg.pose.pose.orientation

        # Broadcast the transform
        self.broadcaster.sendTransform(self.transform)


def main(args=None):
    print("Hi")
    rclpy.init(args=args)

    odometry_to_tf_node = OdometryToTF()

    # Spin the node to keep it running and processing callbacks
    rclpy.spin(odometry_to_tf_node)
    print("Hi2")
    # Destroy the node explicitly when done
    odometry_to_tf_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
