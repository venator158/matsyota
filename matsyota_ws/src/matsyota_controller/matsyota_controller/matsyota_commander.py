import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PolygonStamped, Point32, Point
from std_msgs.msg import String
import numpy as np
from scipy.spatial import Voronoi
from shapely.geometry import Polygon, box
from functools import partial

class MatsyotaCommander(Node):
    def __init__(self):
        super().__init__('matsyota_commander')
        
        self.fleet_size = 4
        self.timeout_threshold = 2.0 
        
        # --- State Tracking ---
        self.agent_positions = {}  
        self.last_seen = {}        
        self.publishers_dict = {}  
        self.assigned_territories = {} # Memory of current active tasks
        
        # --- Legacy Task Queue (The "Debt" Registry) ---
        # Stores tuples: (Shapely Polygon, Point32 last_known_pos)
        self.legacy_tasks = []

        # --- Farm Boundaries ---
        self.farm_boundary = box(-50.0, -50.0, 50.0, 50.0)

        for i in range(self.fleet_size):
            ns = f'asv_{i}'
            self.create_subscription(NavSatFix, f'/{ns}/gps/fix', partial(self.gps_callback, agent_id=ns), 10)
            
            # Completion Listener
            self.create_subscription(String, f'/{ns}/task_status', partial(self.status_callback, agent_id=ns), 10)
            
            self.publishers_dict[ns] = self.create_publisher(PolygonStamped, f'/{ns}/assigned_territory', 10)

        # SCADA loop at 1Hz
        self.create_timer(1.0, self.commander_loop)
        self.get_logger().info("Matsyota Commander: Inheritance Mode Active.")

    def gps_callback(self, msg: NavSatFix, agent_id: str):
        self.agent_positions[agent_id] = (msg.longitude, msg.latitude)
        self.last_seen[agent_id] = self.get_clock().now().nanoseconds / 1e9

    def status_callback(self, msg: String, agent_id: str):
        """When an agent finishes, it checks if it needs to steal a legacy task."""
        if msg.data == "COMPLETE":
            self.get_logger().info(f"Agent {agent_id} completed its primary task.")
            
            # Remove from current active assignments
            if agent_id in self.assigned_territories:
                del self.assigned_territories[agent_id]

            # CHECK DEBT REGISTRY: Is there a fallen comrade to assist?
            if self.legacy_tasks:
                legacy_poly, restart_pt = self.legacy_tasks.pop(0)
                self.get_logger().warn(f"Reassigning legacy task to {agent_id}!")
                self.dispatch_territory(agent_id, legacy_poly, restart_pt)

    def commander_loop(self):
        now = self.get_clock().now().nanoseconds / 1e9
        active_agents = []
        active_positions = []

        # 1. Watchdog & Inheritance Trigger
        for agent_id in list(self.agent_positions.keys()):
            time_since_last_msg = now - self.last_seen.get(agent_id, 0)
            
            if time_since_last_msg > self.timeout_threshold:
                if agent_id in self.assigned_territories:
                    self.get_logger().error(f"FATAL: {agent_id} failed mid-task. Archiving territory.")
                    
                    # Capture inheritance: (The Polygon, Last Known Position)
                    last_pos = self.agent_positions[agent_id]
                    restart_pt = Point32(x=float(last_pos[0]), y=float(last_pos[1]), z=0.0)
                    
                    self.legacy_tasks.append((self.assigned_territories[agent_id], restart_pt))
                    del self.assigned_territories[agent_id]
                continue
            
            # Only include agents in Voronoi if they aren't already busy
            if agent_id not in self.assigned_territories:
                active_agents.append(agent_id)
                active_positions.append(self.agent_positions[agent_id])

        # 2. Initial Partitioning (Only runs if unassigned agents exist)
        if not active_agents:
            return

        if len(active_agents) < 3:
            # Simple split for 1 or 2 agents (omitted for brevity, assume 3+ for demo)
            return

        try:
            dummy_points = [[-1000, -1000], [1000, -1000], [1000, 1000], [-1000, 1000]]
            points_for_voronoi = np.vstack((active_positions, dummy_points))
            vor = Voronoi(points_for_voronoi)
            
            for idx, agent_id in enumerate(active_agents):
                region_idx = vor.point_region[idx]
                region = vor.regions[region_idx]
                
                if -1 not in region and len(region) > 0:
                    raw_poly = Polygon([vor.vertices[i] for i in region])
                    # CLIP TO FARM BOUNDARY (Solves infinite bounds)
                    final_poly = raw_poly.intersection(self.farm_boundary)
                    
                    if not final_poly.is_empty:
                        # For initial dispatch, restart_pt is None (start from current pos)
                        self.dispatch_territory(agent_id, final_poly, None)
                        self.assigned_territories[agent_id] = final_poly
                    
        except Exception as e:
            self.get_logger().error(f"Voronoi Failure: {e}")

    def dispatch_territory(self, agent_id, shapely_poly, restart_pt):
        """Converts Shapely poly to ROS2 msg and adds a 'restart_point' if it's a legacy task."""
        msg = PolygonStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        
        # We pack the vertices into the Polygon msg
        for x, y in shapely_poly.exterior.coords:
            msg.polygon.points.append(Point32(x=float(x), y=float(y), z=0.0))
        
        # CRITICAL: We use the Z-axis of the first point to flag a 'Legacy' restart
        # or we could use a custom message. For now, we'll log it.
        if restart_pt:
            self.get_logger().info(f"Task for {agent_id} includes RESTART at {restart_pt.x, restart_pt.y}")
            # Note: In a production scenario, you'd add a 'Point' field to a custom message.
            # Here, the agent will assume first polygon point is start unless specified.
            
        self.publishers_dict[agent_id].publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MatsyotaCommander()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()