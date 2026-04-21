import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from functools import partial
import numpy as np

class FeedDispersionMapper(Node):
    def __init__(self):
        super().__init__('feed_dispersion_mapper')
        
        self.fleet_size = 4
        self.resolution = 0.25
        self.width = int(100.0 / self.resolution)
        self.height = int(100.0 / self.resolution)
        self.origin_x = -50.0
        self.origin_y = -50.0
        
        # Grid initialized to 0 (uncovered water)
        self.grid_data = np.zeros(self.width * self.height, dtype=np.int8)
        
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 10)
        
        for i in range(self.fleet_size):
             ns = f'asv_{i}'
             self.create_subscription(Odometry, f'/{ns}/odom', partial(self.odom_callback, agent_id=ns), 10)
             
        self.create_timer(1.0, self.publish_map)
        self.get_logger().info('Feed Dispersion Mapper node started.')
        
    def odom_callback(self, msg, agent_id):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        grid_x = int((x - self.origin_x) / self.resolution)
        grid_y = int((y - self.origin_y) / self.resolution)
        
        # 1.0m diameter dispersion -> 0.5m radius -> 2 cells
        radius = 2 
        for dx in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                if dx*dx + dy*dy <= radius*radius:
                    cx = grid_x + dx
                    cy = grid_y + dy
                    if 0 <= cx < self.width and 0 <= cy < self.height:
                        idx = cy * self.width + cx
                        self.grid_data[idx] = 100
                        
    def publish_map(self):
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.info.resolution = self.resolution
        msg.info.width = self.width
        msg.info.height = self.height
        msg.info.origin.position.x = self.origin_x
        msg.info.origin.position.y = self.origin_y
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        msg.data = self.grid_data.tolist()
        self.map_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = FeedDispersionMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
