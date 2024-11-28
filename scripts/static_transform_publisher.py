import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import tf_transformations
from tf2_ros import StaticTransformBroadcaster

class MultiStaticTransformPublisher(Node):
    def __init__(self):
        super().__init__('multi_static_transform_publisher')

        # List of transformations for the robots
        # Define each robot's static transform here
        transforms = [
            {
                'parent_frame': 'map',
                'child_frame': 'robot1_base_link',
                'translation': [0.0, 0.0, 0.0],  # [x, y, z]
                'rotation': [0.0, 0.0, 0.0, 1.0],  # Quaternion [qx, qy, qz, qw]
            },
            {
                'parent_frame': 'map',
                'child_frame': 'robot2_base_link',
                'translation': [1.0, 0.0, 0.0],
                'rotation': [0.0, 0.0, 0.0, 1.0],
            },
            {
                'parent_frame': 'map',
                'child_frame': 'robot3_base_link',
                'translation': [0.0, 1.0, 0.0],
                'rotation': [0.0, 0.0, 0.0, 1.0],
            },
            {
                'parent_frame': 'map',
                'child_frame': 'robot4_base_link',
                'translation': [1.0, 1.0, 0.0],
                'rotation': [0.0, 0.0, 0.0, 1.0],
            }
        ]

        # Create the StaticTransformBroadcaster
        self.broadcaster = StaticTransformBroadcaster(self)

        # Publish all transforms
        for transform in transforms:
            static_transform_stamped = TransformStamped()
            static_transform_stamped.header.stamp = self.get_clock().now().to_msg()
            static_transform_stamped.header.frame_id = transform['parent_frame']
            static_transform_stamped.child_frame_id = transform['child_frame']
            static_transform_stamped.transform.translation.x = transform['translation'][0]
            static_transform_stamped.transform.translation.y = transform['translation'][1]
            static_transform_stamped.transform.translation.z = transform['translation'][2]
            static_transform_stamped.transform.rotation.x = transform['rotation'][0]
            static_transform_stamped.transform.rotation.y = transform['rotation'][1]
            static_transform_stamped.transform.rotation.z = transform['rotation'][2]
            static_transform_stamped.transform.rotation.w = transform['rotation'][3]

            # Broadcast the transform
            self.broadcaster.sendTransform(static_transform_stamped)
            self.get_logger().info(f"Static transform published from {transform['parent_frame']} to {transform['child_frame']}")

def main(args=None):
    rclpy.init(args=args)
    node = MultiStaticTransformPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
