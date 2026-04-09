import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PolygonStamped, Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
import math
import numpy as np

class ASVDriver(Node):
    def __init__(self):
        super().__init__('asv_driver')
        
        # We need to know which boat we are to handle namespaces correctly
        # In ROS 2, this is usually passed via launch parameters or inferred
        self.declare_parameter('namespace', 'asv_0')
        self.ns = self.get_parameter('namespace').get_parameter_value().string_value
        
        # --- State Variables ---
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.waypoints = []        # List of (x, y) tuples generated locally
        self.current_wp_index = 0
        self.sweep_spacing = 2.0   # Meters between lawnmower lines

        # --- ROS 2 Interfaces ---
        # 1. Listen for the Commander's territory assignment
        self.create_subscription(PolygonStamped, f'/{self.ns}/assigned_territory', self.territory_callback, 10)
        
        # 2. Listen to local high-speed odometry for PD control
        self.create_subscription(Odometry, f'/{self.ns}/odom', self.odom_callback, 50)
        
        # 3. Publish motor commands
        self.cmd_pub = self.create_publisher(Twist, f'/{self.ns}/cmd_vel', 50)
        
        # 4. NEW: Publish local planned path for RViz2 visualization
        self.path_pub = self.create_publisher(Path, f'/{self.ns}/local_plan', 10)
        
        # Control Loop Timer (50Hz)
        self.create_timer(0.02, self.control_loop)
        
        self.get_logger().info(f"{self.ns} Driver Initialized. Awaiting territory...")

    def territory_callback(self, msg: PolygonStamped):
        """Triggered when the Commander assigns a new Voronoi cell."""
        self.get_logger().info(f"{self.ns} received new territory polygon. Generating path...")
        
        # Extract vertices
        vertices = [(p.x, p.y) for p in msg.polygon.points]
        
        if len(vertices) < 3:
            return # Invalid polygon
            
        # Generate the Boustrophedon (Lawnmower) waypoints locally
        self.waypoints = self.generate_boustrophedon_path(vertices)
        self.current_wp_index = 0
        
        # Broadcast the generated path to RViz2
        self.publish_rviz_path()

    def generate_boustrophedon_path(self, vertices):
        """
        Placeholder for the Boustrophedon logic. 
        In production, use shapely to slice the polygon with parallel lines 
        spaced by self.sweep_spacing, yielding a sweeping back-and-forth path.
        """
        # For now, just trace the perimeter to prove the architecture works
        path = vertices.copy()
        path.append(vertices[0]) # Close the loop
        return path

    def publish_rviz_path(self):
        """Publishes the generated path so evaluators can see the agent's intent."""
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "map" # Global frame for visualization
        
        for (x, y) in self.waypoints:
            pose = PoseStamped()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            path_msg.poses.append(pose)
            
        self.path_pub.publish(path_msg)

    def odom_callback(self, msg: Odometry):
        """High-speed local state update."""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        # Extract Yaw from Quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def control_loop(self):
        """PD Controller translating the local path into motor commands."""
        if not self.waypoints or self.current_wp_index >= len(self.waypoints):
            # No path or path complete -> Stop motors
            self.cmd_pub.publish(Twist())
            return
            
        target_x, target_y = self.waypoints[self.current_wp_index]
        
        # Calculate error
        dx = target_x - self.current_x
        dy = target_y - self.current_y
        distance = math.sqrt(dx**2 + dy**2)
        
        # Waypoint threshold (Move to next waypoint if within 0.5m)
        if distance < 0.5:
            self.current_wp_index += 1
            return
            
        target_yaw = math.atan2(dy, dx)
        yaw_error = target_yaw - self.current_yaw
        
        # Normalize yaw error to [-pi, pi]
        yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error))
        
        # PD Gains
        Kp_linear = 0.5
        Kp_angular = 2.0
        
        # Command Output
        cmd = Twist()
        # Slow down linearly if we are facing the wrong way
        cmd.linear.x = max(0.0, Kp_linear * distance * math.cos(yaw_error))
        cmd.angular.z = Kp_angular * yaw_error
        
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = ASVDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()