'''
    PURPOSE: To map out the global database of bots , obstacles and targets.
    Input(s):
        File inputs:
        - Data.yaml : Contains initial position of bot , center of mass of static obstacles and location of targers
        Topic subscriptions:
        - /robot_i/odom : Odometry Data of Robot i
            - Format: nav_msgs/Odometry
        - /obstacle_markers: Locations of all obstacles
            - Format: visualization_msgs/MarkerArray
        - /map : A map of the environment
            - Format: nav_msgs/OccupancyGrid
    Output:
        - A map of all obstacles , bots and targets , with a dynamic background of an occupancy map
'''

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, OccupancyGrid # Getting Odometry Data
from visualization_msgs.msg import MarkerArray  # Visualizing Obstacles
import pygame                   # Visualization method
from functools import partial   # Creating Multiple subscriptions at once
import yaml        # To read the input Data.yaml file
import numpy as np # To deal with the background operations quickly


class GlobalDatabaseNode(Node): # Creates a node that acts as the interface for the same.
    def __init__(self):
        super().__init__('global_database_node')
        self.data_file = "../config/robots.yaml"    # The data file for the same
        self.data = self.load_data(self.data_file)                          # Load the data

        self.robot_data = {}                            # A dictionary to store robot data
        self.num_robots = len(self.data["robot1"])        # Number of robots.
        self.scale = 100                                # Pixels to m(1000/10 = 100)

        # Initialize robot data
        for i in range(1, self.num_robots + 1):
            scaled_position = (int(self.data["robot1"][i - 1]["x"] * self.scale),(1000 - int(self.data["robot1"][i - 1]["y"] * self.scale)))
            self.robot_data[i] = {"Position": scaled_position}
        
        self.obs = []

        # Scale tasks
        self.tasks = {}
        for i in self.data["target"]:
            self.tasks[i["name"]] = (int(i["x"]*self.scale),1000 - int(i["y"]*self.scale))
        #Initialize the pygame environment
        pygame.init()
        self.screen = pygame.display.set_mode((1400, 1000))
        self.clock = pygame.time.Clock()    # FPS Control
        pygame.display.set_caption("Global Database")
        self.CL = True                      # Control Statement(For closing the environment)

        for i in range(1, self.num_robots + 1):
            pos_topic = f'/robot1_{i}/odom'      # Odometry Topic 
            self.create_subscription(Odometry, pos_topic, partial(self.pos_callback, i), 10)    # Odometry Subscription
        self.create_subscription(MarkerArray, '/obstacle_markers', self.obs_callback, 10)       # Obstacles Subscription
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)                  # Map Subscription

    def load_data(self, filename):  # Method to load the data.yaml file
        try:
            with open(filename, 'r') as file:
                return yaml.safe_load(file)
        except Exception as e:
            self.get_logger().error(f"Failed to load data: {e}")
            return {}

    def pos_callback(self, robot_id, msg):  # Getting position data of each bot via ROS Topics
        position = msg.pose.pose.position
        x = int(position.x * self.scale)
        y = self.screen.get_height() - int(position.y * self.scale)
        self.robot_data[robot_id]["Position"] = (x, y)
        self.update_pygame()

    def obs_callback(self, msg):    # Getting obstacle data via ROS Topics
        self.obs.clear()
        for marker in msg.markers:
            x = int(marker.pose.position.x * self.scale)
            y = self.screen.get_height() - int(marker.pose.position.y * self.scale)
            self.obs.append((x, y))
        self.update_pygame()

    def map_callback(self, msg):    # Getting map data via ROS Topics , assuming output resolution is 0.1 m/cell
        # Extract map information
        self.res = msg.info.resolution
        self.width, self.height = msg.info.width, msg.info.height
        self.map_flat = np.array(msg.data, dtype=np.int8)
        self.map = self.map_flat.reshape(self.height, self.width)

        # Create an empty color map
        self.color_map = np.zeros((self.map.shape[0], self.map.shape[1], 3), dtype=np.uint8)

        # Set color for unknown cells (-1)
        self.color_map[self.map == -1] = [0, 0, 0]  # Black for unknown

        # Create a mask for known cells (values from 0 to 100)
        known_mask = (self.map >= 0) & (self.map <= 100)

        # Map low values (blue) and high values (red)
        red_values = np.clip((self.map[known_mask] / 100.0) * 255, 0, 255).astype(np.uint8)  # Scale red from 0 to 255
        blue_values = np.clip(255 - red_values, 0, 255).astype(np.uint8)  # Scale blue inversely

        # Assign colors based on the mask
        colors = np.zeros((np.sum(known_mask), 3), dtype=np.uint8)
        colors[:, 0] = red_values   # Red channel
        colors[:, 2] = blue_values   # Blue channel

        # Place colors back into the color_map at the correct indices
        self.color_map[known_mask] = colors

        # Create surface with scaling
        self.surf = pygame.Surface((self.width, self.height))
        pygame.surfarray.blit_array(self.surf, self.color_map)
        self.surf = pygame.transform.flip(self.surf, flip_x=False, flip_y=True)
        # Blit the scaled surface onto the screen
        self.screen.blit(self.surf, (0, 0))

        pygame.display.update()

    def draw_cross(self, position, color): # Draw a X to mark the target
        X = int(25 / np.sqrt(2))
        p1 = (position[0] + X, position[1] + X)
        p2 = (position[0] - X, position[1] - X)
        p3 = (position[0] - X, position[1] + X)
        p4 = (position[0] + X, position[1] - X)
        pygame.draw.line(self.screen, color, p1, p2, 5)
        pygame.draw.line(self.screen, color, p3, p4, 5)

    def render_tasks(self):               # Render the tasks list
        task_color = pygame.Color(255, 0, 0)
        for _, location in self.tasks.items():
            position = [int(coord) for coord in location]
            self.draw_cross(position, task_color)

    def render_robot(self, robot_id):     # Draw robot and path taken by the robot
        position = self.robot_data[robot_id]["Position"]
        pygame.draw.circle(self.screen, pygame.Color(0, 255, 0), position, 25)

    def render_obstacles(self):         # Draw obstacles
        for obs in self.obs:
            pygame.draw.circle(self.screen, pygame.Color(255, 0, 255), obs, 5)

    def render_task_table(self):        # Draw the task table
        pygame.draw.rect(self.screen, pygame.Color(200, 200, 200), (1000, 0, 400, 1000))  # Sidebar background
        font = pygame.font.Font(None, 24)
        y_offset = 20

        for task_name, location in self.tasks.items():
            text = f"{task_name}: {location[0],1000 - location[1]}"
            task_surface = font.render(text, True, (0, 0, 0))
            self.screen.blit(task_surface, (1010, y_offset))
            y_offset += 30

    def render_legend(self):    # Draw the legend
        legend_x = 1010
        legend_y = 900
        spacing = 30
        font = pygame.font.Font(None, 24)

        legend_items = [
            ("Robot Position", pygame.Color(0, 255, 0), "circle"),
            ("Obstacles", pygame.Color(255, 0, 255), "circle"),
            ("Targets", pygame.Color(255, 0, 0), "cross"),
        ]

        for i, (label, color, shape) in enumerate(legend_items):
            item_y = legend_y + i * spacing

            if shape == "circle":
                pygame.draw.circle(self.screen, color, (legend_x, item_y + 10), 10)
            elif shape == "line":
                pygame.draw.line(self.screen, color, (legend_x - 10, item_y + 10), (legend_x + 10, item_y + 10), 3)
            elif shape == "cross":
                X = 10
                p1 = (legend_x - X, item_y + 10 - X)
                p2 = (legend_x + X, item_y + 10 + X)
                p3 = (legend_x - X, item_y + 10 + X)
                p4 = (legend_x + X, item_y + 10 - X)
                pygame.draw.line(self.screen, color, p1, p2, 3)
                pygame.draw.line(self.screen, color, p3, p4, 3)

            text_surface = font.render(label, True, (0, 0, 0))
            self.screen.blit(text_surface, (legend_x + 20, item_y+2))

    def update_data(self):  # Update Data
        self.tasks = {}
        for i in self.data["target"]:
            self.tasks[i["name"]] = (int(i["x"]*self.scale),1000 - int(i["y"]*self.scale))

    def run_pygame(self):
        while rclpy.ok() and self.CL:
            rclpy.spin_once(self, timeout_sec=0.1)

            # Render dynamic map background
            self.screen.fill((255, 255, 255))  # Clear screen in case map isn't ready
            if hasattr(self, 'surf'):         # Check if map is loaded
                self.screen.blit(self.surf, (0, 0))  # Render background map

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.CL = False

            self.update_data()
            for i in range(1, self.num_robots + 1):
                self.render_robot(i)
            self.render_obstacles()
            self.render_tasks()
            self.render_task_table()
            self.render_legend()

            pygame.display.flip()

        pygame.quit()

    def update_pygame(self):    # Update loop
        self.screen.fill((255,255,255))
        if hasattr(self, 'surf'):         # Check if map is loaded
            self.screen.blit(self.surf, (0, 0))  # Render background map
        
        for i in range(1, self.num_robots + 1):
            self.render_robot(i)
        self.render_obstacles()
        self.render_tasks()
        self.render_task_table()
        self.render_legend()
        pygame.display.flip()

def main(args=None):    # Main node running
    rclpy.init(args=args)
    node = GlobalDatabaseNode()
    try:
        node.run_pygame()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
