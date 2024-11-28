import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import tf_transformations

class StaticTransformPublisher(Node):
    def __init__(self):
        super().__init__('static_transform_publisher')

        # Parameters for the transform
        self.declare_parameter('parent_frame', 'map')
        self.declare_parameter('child_frame', 'base_link')
        self.declare_parameter('translation', [0.0, 0.0, 0.0])  # [x, y, z]
        self.declare_parameter('rotation', [0.0, 0.0, 0.0, 1.0])  # [qx, qy, qz, qw]

        # Get parameters
        parent_frame = self.get_parameter('parent_frame').value
        child_frame = self.get_parameter('child_frame').value
        translation = self.get_parameter('translation').value
        rotation = self.get_parameter('rotation').value

        # Create a StaticTransformBroadcaster
        self.broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        # Create the transform message
        static_transform_stamped = TransformStamped()
        static_transform_stamped.header.stamp = self.get_clock().now().to_msg()
        static_transform_stamped.header.frame_id = parent_frame
        static_transform_stamped.child_frame_id = child_frame
        static_transform_stamped.transform.translation.x = translation[0]
        static_transform_stamped.transform.translation.y = translation[1]
        static_transform_stamped.transform.translation.z = translation[2]
        static_transform_stamped.transform.rotation.x = rotation[0]
        static_transform_stamped.transform.rotation.y = rotation[1]
        static_transform_stamped.transform.rotation.z = rotation[2]
        static_transform_stamped.transform.rotation.w = rotation[3]

        # Broadcast the transform
        self.broadcaster.sendTransform(static_transform_stamped)
        self.get_logger().info(f"Static transform published from {parent_frame} to {child_frame}")

def main(args=None):
    rclpy.init(args=args)
    node = StaticTransformPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

