import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid


class MapSubscriber(Node):
    def __init__(self):
        super().__init__('map_subscriber')

        # Subscribe to the /map topic
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )
        self.subscription  # prevent unused variable warning
        self.get_logger().info('Map Subscriber Node Initialized.')

    def map_callback(self, msg):
        """
        Callback function to process incoming map data.

        Args:
            msg (OccupancyGrid): The message published to the /map topic.
        """
        # Header
        self.get_logger().info(f"Received Map: Frame ID: {msg.header.frame_id}")
        self.get_logger().info(f"Timestamp: {msg.header.stamp.sec}.{msg.header.stamp.nanosec}")

        # Map metadata
        self.get_logger().info(f"Map Info: Resolution: {msg.info.resolution}, Width: {msg.info.width}, Height: {msg.info.height}")
        self.get_logger().info(f"Origin: {msg.info.origin.position.x}, {msg.info.origin.position.y}, {msg.info.origin.position.z}")

        # Map data (occupancy grid)
        map_data = msg.data
        print("Map_data", len(map_data))


def main(args=None):
    rclpy.init(args=args)

    # Create and spin the subscriber node
    map_subscriber = MapSubscriber()

    try:
        rclpy.spin(map_subscriber)
    except KeyboardInterrupt:
        map_subscriber.get_logger().info('Shutting down Map Subscriber Node.')
    finally:
        map_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

