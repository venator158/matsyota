import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PolygonStamped, Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import String
from shapely.geometry import Polygon, LineString, MultiLineString
import math
import numpy as np

class ASVDriver(Node):
    def __init__(self):
        super().__init__('asv_driver')
        
        self.declare_parameter('namespace', 'asv_0')
        self.ns = self.get_parameter('namespace').get_parameter_value().string_value
        
        # --- State Variables ---
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.waypoints = []        
        self.current_wp_index = 0
        self.sweep_spacing = 2.5   # Meters between lawnmower lines
        self.task_active = False

        # --- ROS 2 Interfaces ---
        self.create_subscription(PolygonStamped, f'/{self.ns}/assigned_territory', self.territory_callback, 10)
        self.create_subscription(Odometry, f'/{self.ns}/odom', self.odom_callback, 50)
        
        self.cmd_pub = self.create_publisher(Twist, f'/{self.ns}/cmd_vel', 50)
        self.path_pub = self.create_publisher(Path, f'/{self.ns}/local_plan', 10)
        self.status_pub = self.create_publisher(String, f'/{self.ns}/task_status', 10)
        
        self.create_timer(0.02, self.control_loop)
        self.get_logger().info(f"{self.ns} Driver Online.")

    def territory_callback(self, msg: PolygonStamped):
        """Processes assigned polygon and generates deterministic sweep."""
        vertices = [(p.x, p.y) for p in msg.polygon.points]
        if len(vertices) < 3: return

        # Deterministic Boustrophedon Generation
        poly = Polygon(vertices)
        new_waypoints = self.generate_boustrophedon_path(poly)
        
        # INHERITANCE LOGIC: 
        # Check if the Commander passed a specific restart point. 
        # For this implementation, we check the distance to the start.
        # If the nearest waypoint is far, we drive there first.
        self.waypoints = new_waypoints
        self.current_wp_index = 0
        self.task_active = True
        
        self.publish_rviz_path()
        self.get_logger().info(f"{self.ns}: Path generated with {len(self.waypoints)} waypoints.")

    def generate_boustrophedon_path(self, poly):
        """Slices the polygon with parallel lines to create a lawnmower path."""
        minx, miny, maxx, maxy = poly.bounds
        waypoints = []
        
        # Create horizontal scan lines
        y_coords = np.arange(miny + self.sweep_spacing/2, maxy, self.sweep_spacing)
        
        for i, y in enumerate(y_coords):
            # Create a horizontal line across the bounding box
            line = LineString([(minx - 1, y), (maxx + 1, y)])
            intersection = line.intersection(poly)
            
            if intersection.is_empty:
                continue
                
            # Handle MultiLineString if the polygon is concave/complex
            segments = []
            if intersection.geom_type == 'LineString':
                segments.append(list(intersection.coords))
            elif intersection.geom_type == 'MultiLineString':
                for ls in intersection.geoms:
                    segments.append(list(ls.coords))
            
            # For each segment, sort for 'S-curve' Boustrophedon
            for seg in segments:
                # Reverse every other line to create back-and-forth motion
                if i % 2 != 0:
                    seg.reverse()
                waypoints.extend(seg)
                
        return waypoints

    def publish_rviz_path(self):
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "map"
        for (x, y) in self.waypoints:
            p = PoseStamped()
            p.pose.position.x, p.pose.position.y = x, y
            path_msg.poses.append(p)
        self.path_pub.publish(path_msg)

    def odom_callback(self, msg: Odometry):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.current_yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y**2 + q.z**2))

    def control_loop(self):
        if not self.task_active or self.current_wp_index >= len(self.waypoints):
            if self.task_active:
                # TASK COMPLETE HANDSHAKE
                self.status_pub.publish(String(data="COMPLETE"))
                self.task_active = False
                self.get_logger().info(f"{self.ns} reporting mission complete.")
            self.cmd_pub.publish(Twist())
            return
            
        tx, ty = self.waypoints[self.current_wp_index]
        dx, dy = tx - self.current_x, ty - self.current_y
        dist = math.sqrt(dx**2 + dy**2)
        
        if dist < 0.6: # Waypoint reached
            self.current_wp_index += 1
            return
            
        target_yaw = math.atan2(dy, dx)
        y_err = math.atan2(math.sin(target_yaw - self.current_yaw), math.cos(target_yaw - self.current_yaw))
        
        cmd = Twist()
        # Scale speed based on orientation (don't drive fast while turning)
        cmd.linear.x = 1.0 * math.cos(y_err) if abs(y_err) < 1.0 else 0.0
        cmd.angular.z = 2.5 * y_err
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = ASVDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()