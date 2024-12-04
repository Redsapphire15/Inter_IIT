import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from rclpy.parameter import Parameter





class OdometryToTF(Node):
    def __init__(self):
        super().__init__('odometry_to_tf')

        # Create a tf2 broadcaster
        self.broadcaster = TransformBroadcaster(self)
        self.set_parameters([Parameter('use_sim_time', value=True)])
        # Create subscribers to odometry topics
        self.odom_data = {
            'robot1_1': None,
            'robot1_2': None,
            'robot1_3': None,
            'robot1_4': None,
        }

        self.create_subscription(Odometry, '/robot1_1/odom', self.odom_callback('robot1_1'), 10)
        self.create_subscription(Odometry, '/robot1_2/odom', self.odom_callback('robot1_2'), 10)
        self.create_subscription(Odometry, '/robot1_3/odom', self.odom_callback('robot1_3'), 10)
        self.create_subscription(Odometry, '/robot1_4/odom', self.odom_callback('robot1_4'), 10)

        # Timer to publish transforms at 100 Hz
        self.timer = self.create_timer(0.01, self.publish_transforms)

    def odom_callback(self, robot_name):
        def callback(msg):
            self.odom_data[robot_name] = msg
        return callback

    def publish_transforms(self):
        print("Ji")
        for robot_name, odom_msg in self.odom_data.items():
            if odom_msg:
                # Set transform data
                odom_frame = 'odom'
                base_link_frame = f'{robot_name}/base_footprint'

                transform = TransformStamped()
                # transform.header.stamp = self.get_clock().now().to_msg()
                transform.header.stamp = odom_msg.header.stamp
                transform.header.frame_id = odom_frame
                transform.child_frame_id = base_link_frame

                transform.transform.translation.x = odom_msg.pose.pose.position.x
                transform.transform.translation.y = odom_msg.pose.pose.position.y
                transform.transform.translation.z = odom_msg.pose.pose.position.z
                transform.transform.rotation = odom_msg.pose.pose.orientation

                # Broadcast the transform
                self.broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)

    odometry_to_tf_node = OdometryToTF()

    # Spin the node to keep it running and processing callbacks
    rclpy.spin(odometry_to_tf_node)

    # Destroy the node explicitly when done
    odometry_to_tf_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

