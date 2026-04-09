import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import NavSatFix
import math

class FeedDispersionMapper(Node):
    def __init__(self):
        super().__init__('feed_dispersion_mapper')
        
        # --- Grid Parameters ---
        self.resolution = 0.25           # Meters per cell
        self.farm_size = 100.0           # 100x100 meter farm
        self.origin_x = -50.0            # Bottom left X
        self.origin_y = -50.0            # Bottom left Y
        self.dispersion_radius = 1.5     # Meters of feed spread per boat
        
        # Matrix Dimensions
        self.width = int(self.farm_size / self.resolution)
        self.height = int(self.farm_size / self.resolution)
        
        # Initialize the 1D array: 0 = Unfed, 100 = Fed
        self.grid_data = [0] * (self.width * self.height)
        
        # --- ROS 2 Interfaces ---
        # Publisher for RViz2
        self.map_pub = self.create_publisher(OccupancyGrid, '/feed_map', 10)
        
        # Subscribers for Swarm GPS (Hardcoded for 3 agents in Phase 1)
        self.create_subscription(NavSatFix, '/asv_0/gps/fix', self.gps_callback, 10)
        self.create_subscription(NavSatFix, '/asv_1/gps/fix', self.gps_callback, 10)
        self.create_subscription(NavSatFix, '/asv_2/gps/fix', self.gps_callback, 10)
        
        # Publish the map at 2Hz
        self.create_timer(0.5, self.publish_map)
        
        self.get_logger().info("Feed Dispersion Mapper Initialized.")

    def gps_callback(self, msg: NavSatFix):
        # NOTE: In a flat Gazebo planar_move, NavSatFix altitude/lat/lon 
        # is often mapped directly to X, Y if properly configured. 
        # We assume msg.longitude = X and msg.latitude = Y for this proxy.
        x = msg.longitude
        y = msg.latitude
        
        self.apply_feed_dispersion(x, y)

    def apply_feed_dispersion(self, center_x, center_y):
        # Convert physical radius to cell count
        cell_radius = int(self.dispersion_radius / self.resolution)
        
        center_col = int((center_x - self.origin_x) / self.resolution)
        center_row = int((center_y - self.origin_y) / self.resolution)
        
        # Iterate through the bounding box around the boat
        for r in range(center_row - cell_radius, center_row + cell_radius + 1):
            for c in range(center_col - cell_radius, center_col + cell_radius + 1):
                # Bounds check
                if 0 <= r < self.height and 0 <= c < self.width:
                    # Circular distance check
                    if math.sqrt((r - center_row)**2 + (c - center_col)**2) <= cell_radius:
                        idx = (r * self.width) + c
                        self.grid_data[idx] = 100 # Mark as FED

    def publish_map(self):
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        
        msg.info.resolution = self.resolution
        msg.info.width = self.width
        msg.info.height = self.height
        
        # Set the origin to the bottom-left corner
        msg.info.origin.position.x = self.origin_x
        msg.info.origin.position.y = self.origin_y
        msg.info.origin.position.z = 0.0
        
        msg.data = self.grid_data
        self.map_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = FeedDispersionMapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()