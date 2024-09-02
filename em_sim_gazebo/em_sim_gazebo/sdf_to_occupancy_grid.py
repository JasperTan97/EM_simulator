import xml.etree.ElementTree as ET
import numpy as np
import os
import time
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose

class SDFToOccupancyGrid(Node):
    def __init__(self):
        super().__init__('sdf_to_occupancy_grid_node')

        # Get the package share directory
        pkg_project_gazebo = get_package_share_directory("em_sim_gazebo")

        # Construct the full path to the SDF file
        sdf_file = os.path.join(pkg_project_gazebo, "worlds", "road_network.sdf")

        # Generate the occupancy grid from the SDF file
        self.grid, self.origin_x, self.origin_y, self.resolution = self.sdf_to_occupancy_grid(sdf_file, resolution=0.1)

        # Set up a publisher for the occupancy grid
        self.publisher_ = self.create_publisher(OccupancyGrid, '/map', 10)

        # Publish the map continuously every 0.5 seconds
        self.timer = self.create_timer(0.5, self.publish_occupancy_grid_callback)

        # # NOTE: Can't seem to make transient local work with rviz2
        # qos_profile = QoSProfile(
        #     durability=DurabilityPolicy.TRANSIENT_LOCAL,
        #     reliability=ReliabilityPolicy.RELIABLE,
        #     depth=1
        # )

        # # Set up a publisher for the occupancy grid with the specified QoS
        # self.publisher_ = self.create_publisher(OccupancyGrid, '/map', qos_profile)

        # self.publish_occupancy_grid_callback()

    def sdf_to_occupancy_grid(self, sdf_file, resolution=0.1):
        # Parse the SDF file
        tree = ET.parse(sdf_file)
        root = tree.getroot()

        # Determine the bounds of the map based on the SDF
        # Assuming the SDF is well-structured and roads are modeled as boxes or planes
        min_x, max_x, min_y, max_y = float('inf'), float('-inf'), float('inf'), float('-inf')

        for model in root.findall(".//model"):
            for link in model.findall("link"):
                for collision in link.findall("collision"):
                    for geometry in collision.findall("geometry"):
                        box = geometry.find("box")
                        if box is not None:
                            size = box.find("size").text.split()
                            size_x = float(size[0]) / 2.0
                            size_y = float(size[1]) / 2.0
                            # print(f"      Size: {size_x}, {size_y}")

                        pose = collision.find("pose")
                        if pose is not None:
                            pose_values = list(map(float, pose.text.split()))
                            pos_x = pose_values[0]
                            pos_y = pose_values[1]
                            # print(f"      Pose: {pos_x}, {pos_y}")

                            min_x = min(min_x, pos_x - size_x)
                            max_x = max(max_x, pos_x + size_x)
                            min_y = min(min_y, pos_y - size_y)
                            max_y = max(max_y, pos_y + size_y)

        if min_x == float('inf') or max_x == float('-inf') or min_y == float('inf') or max_y == float('-inf'):
            raise ValueError("Failed to parse the SDF file correctly. Ensure that it contains valid models with pose and size elements.")

        # Calculate grid size
        width = int((max_x - min_x) / resolution)
        height = int((max_y - min_y) / resolution)

        # Create the occupancy grid (initialize with free space)
        grid = np.full((height, width), -1)

        for model in root.findall(".//model"):
            for link in model.findall("link"):
                for collision in link.findall("collision"):
                    for geometry in collision.findall("geometry"):
                        box = geometry.find("box")
                        if box is not None:
                            size = box.find("size").text.split()
                            size_x = float(size[0]) / 2.0
                            size_y = float(size[1]) / 2.0

                        pose = collision.find("pose")
                        if pose is not None:
                            pose_values = list(map(float, pose.text.split()))
                            pos_x = pose_values[0]
                            pos_y = pose_values[1]

                            # Mark the grid as occupied
                            grid_min_x = int((pos_x - size_x - min_x) / resolution)
                            grid_max_x = int((pos_x + size_x - min_x) / resolution)
                            grid_min_y = int((pos_y - size_y - min_y) / resolution)
                            grid_max_y = int((pos_y + size_y - min_y) / resolution)

                            grid[grid_min_y:grid_max_y, grid_min_x:grid_max_x] = 100  # Mark as occupied

        return grid, min_x, min_y, resolution

    def publish_occupancy_grid_callback(self):
        self.publish_occupancy_grid(self.grid, self.origin_x, self.origin_y, self.resolution)

    def publish_occupancy_grid(self, grid, origin_x, origin_y, resolution):
        occupancy_grid = OccupancyGrid()
        occupancy_grid.header.frame_id = 'world'
        occupancy_grid.header.stamp = self.get_clock().now().to_msg()  # Add timestamp
        occupancy_grid.info = MapMetaData()
        occupancy_grid.info.resolution = resolution
        occupancy_grid.info.width = grid.shape[1]
        occupancy_grid.info.height = grid.shape[0]
        occupancy_grid.info.origin = Pose()
        occupancy_grid.info.origin.position.x = origin_x
        occupancy_grid.info.origin.position.y = origin_y
        occupancy_grid.info.origin.position.z = 0.0
        occupancy_grid.info.origin.orientation.w = 1.0
        occupancy_grid.data = grid.flatten().tolist()

        self.publisher_.publish(occupancy_grid)


def main(args=None):
    rclpy.init(args=args)
    node = SDFToOccupancyGrid()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
