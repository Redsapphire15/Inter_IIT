import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

def merge_maps(map1, map2, map3, map4):
	merged_map = OccupancyGrid()
	merged_map.header = map1.header
	merged_map.header.frame_id = 'merge_map'

	# Determine the boundaries of the merged map
	maps = [map1, map2]
	if map3:
		maps.append(map3)
	if map4:
		maps.append(map4)

	min_x = min(m.info.origin.position.x for m in maps)
	min_y = min(m.info.origin.position.y for m in maps)
	max_x = max(m.info.origin.position.x + (m.info.width * m.info.resolution) for m in maps)
	max_y = max(m.info.origin.position.y + (m.info.height * m.info.resolution) for m in maps)

	merged_map.info.origin.position.x = min_x
	merged_map.info.origin.position.y = min_y
	merged_map.info.resolution = min(m.info.resolution for m in maps)
	merged_map.info.width = int(np.ceil((max_x - min_x) / merged_map.info.resolution))
	merged_map.info.height = int(np.ceil((max_y - min_y) / merged_map.info.resolution))
	merged_map.data = [-1] * (merged_map.info.width * merged_map.info.height)

	# Merge map data
	for m in maps:
		for y in range(m.info.height):
		    for x in range(m.info.width):
		        i = x + y * m.info.width
		        merged_x = int(np.floor((m.info.origin.position.x + x * m.info.resolution - min_x) / merged_map.info.resolution))
		        merged_y = int(np.floor((m.info.origin.position.y + y * m.info.resolution - min_y) / merged_map.info.resolution))
		        merged_i = merged_x + merged_y * merged_map.info.width
		        if merged_map.data[merged_i] == -1:
		            merged_map.data[merged_i] = m.data[i]
	print("KI")	
	return merged_map

class MergeMapNode(Node):
    def __init__(self):
        super().__init__('merge_map_node')
        qos_profile = QoSProfile(
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            depth=10
        )
        self.publisher = self.create_publisher(OccupancyGrid, '/merge_map', qos_profile)

        # Subscriptions for 4 maps
        self.subscription1 = self.create_subscription(OccupancyGrid, 'map1', self.map1_callback, qos_profile)
        self.subscription2 = self.create_subscription(OccupancyGrid, 'map2', self.map2_callback, qos_profile)
        self.subscription3 = self.create_subscription(OccupancyGrid, 'map3', self.map3_callback, qos_profile)
        self.subscription4 = self.create_subscription(OccupancyGrid, 'map4', self.map4_callback, qos_profile)

        self.map1 = None
        self.map2 = None
        self.map3 = None
        self.map4 = None

    def try_publish_merged_map(self):
        if self.map1 and self.map2 and self.map3 and self.map4:
            merged_map = merge_maps(self.map1, self.map2, self.map3, self.map4)
            self.publisher.publish(merged_map)

    def map1_callback(self, msg):
        self.map1 = msg
        self.try_publish_merged_map()

    def map2_callback(self, msg):
        self.map2 = msg
        self.try_publish_merged_map()

    def map3_callback(self, msg):
        self.map3 = msg
        self.try_publish_merged_map()

    def map4_callback(self, msg):
        self.map4 = msg
        self.try_publish_merged_map()

def main(args=None):
    rclpy.init(args=args)
    merge_map_node = MergeMapNode()
    rclpy.spin(merge_map_node)
    merge_map_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

