import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PolygonStamped, Point32
import numpy as np
from scipy.spatial import Voronoi
from functools import partial

class MatsyotaCommander(Node):
    def __init__(self):
        super().__init__('matsyota_commander')
        
        self.fleet_size = 4
        self.timeout_threshold = 2.0  # Seconds before an agent is declared DEAD
        
        # State Tracking
        self.agent_positions = {}  # { 'asv_0': (x, y) }
        self.last_seen = {}        # { 'asv_0': timestamp_in_seconds }
        self.publishers_dict = {}  # { 'asv_0': Publisher }

        # Farm Boundaries (100x100m)
        self.farm_min = -50.0
        self.farm_max = 50.0

        # Dynamically generate Subscribers and Publishers for the Fleet
        for i in range(self.fleet_size):
            ns = f'asv_{i}'
            
            # Use partial to pass the namespace ID into the generic callback
            self.create_subscription(
                NavSatFix, 
                f'/{ns}/gps/fix', 
                partial(self.gps_callback, agent_id=ns), 
                10
            )
            
            self.publishers_dict[ns] = self.create_publisher(
                PolygonStamped, 
                f'/{ns}/assigned_territory', 
                10
            )

        # The Commander computes the Voronoi lattice at a low frequency (1Hz)
        self.create_timer(1.0, self.commander_loop)
        self.get_logger().info("Matsyota Commander Initialized. Watchdog Active.")

    def gps_callback(self, msg: NavSatFix, agent_id: str):
        """Implicit Heartbeat: Receiving GPS updates the watchdog timer."""
        # Mapping Gazebo planar_move NavSatFix proxy back to X, Y
        self.agent_positions[agent_id] = (msg.longitude, msg.latitude)
        
        # Record the exact ROS 2 time this message was received
        now = self.get_clock().now().nanoseconds / 1e9
        self.last_seen[agent_id] = now

    def commander_loop(self):
        """The core SCADA loop: Check health, run math, dispatch orders."""
        now = self.get_clock().now().nanoseconds / 1e9
        
        active_agents = []
        active_positions = []

        # 1. Watchdog Health Check
        for agent_id in list(self.agent_positions.keys()):
            time_since_last_msg = now - self.last_seen[agent_id]
            
            if time_since_last_msg > self.timeout_threshold:
                self.get_logger().warn(f"WATCHDOG: {agent_id} is DEAD (Timeout: {time_since_last_msg:.2f}s). Dropping from lattice.")
            else:
                active_agents.append(agent_id)
                active_positions.append(self.agent_positions[agent_id])

        if len(active_agents) == 0:
            self.get_logger().error("ALL AGENTS DEAD. Swarm offline.")
            return

        if len(active_agents) < 3:
            # Voronoi requires at least 3 points to form closed polygons.
            # If 2 or fewer agents are alive, fallback logic is needed (e.g., split the square in half).
            self.get_logger().warn("Insufficient agents for Voronoi. Halting dispatch.")
            return

        # 2. Compute Voronoi on strictly ALIVE agents
        try:
            # We add 4 distant "dummy" points to force the outer edges of the Voronoi 
            # diagram to close, so we can intersect them with the farm boundary later.
            dummy_points = [
                [-1000, -1000], [1000, -1000], [1000, 1000], [-1000, 1000]
            ]
            points_for_voronoi = np.vstack((active_positions, dummy_points))
            
            vor = Voronoi(points_for_voronoi)
            
            # Extract and publish the regions
            for idx, agent_id in enumerate(active_agents):
                region_index = vor.point_region[idx]
                region = vor.regions[region_index]
                
                # Check for infinite regions (should be bounded by dummy points)
                if -1 not in region and len(region) > 0:
                    polygon = [vor.vertices[i] for i in region]
                    self.dispatch_territory(agent_id, polygon)
                    
        except Exception as e:
            self.get_logger().error(f"Math Failure during Voronoi partition: {e}")

    def dispatch_territory(self, agent_id, vertices):
        """Packages the vertices into a ROS 2 Polygon and dispatches to the agent."""
        msg = PolygonStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        
        for v in vertices:
            # In production, use Shapely here to clip `v` to self.farm_min/max
            point = Point32()
            point.x = float(v[0])
            point.y = float(v[1])
            point.z = 0.0
            msg.polygon.points.append(point)
            
        self.publishers_dict[agent_id].publish(msg)
        self.get_logger().info(f"Dispatched new boundary to {agent_id}.")

def main(args=None):
    rclpy.init(args=args)
    node = MatsyotaCommander()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()