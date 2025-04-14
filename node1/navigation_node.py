#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PointStamped
import math
import tf2_ros
import tf2_geometry_msgs

class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')
        
        # Tuning parameters (adjustable via launch file)
        self.declare_parameter('drive_speed', 0.2)            # Forward speed (m/s)
        self.declare_parameter('rotate_speed', 0.3)            # Base turning speed (rad/s) for avoidance
        self.declare_parameter('obstacle_threshold', 0.2)      # Front obstacle detection threshold (m)
        self.declare_parameter('wall_follow_distance', 1.0)    # Desired distance from the wall on the right (m)
        self.declare_parameter('kp', 1.0)                      # Proportional gain for wall following
        self.declare_parameter('turning_gain', 1.5)            # Extra gain for faster turning corrections
        self.declare_parameter('sector_width', 0.5236)         # Half-width (in radians) for selecting sensor beams (~30°)
        # Fallback defaults in case TF lookup fails:
        self.declare_parameter('default_front_angle', math.pi)      # Default: front in sensor frame at π radians (180°)
        self.declare_parameter('default_right_angle', math.pi/2)      # Default: right in sensor frame at π/2 radians (90°)
        
        # Retrieve parameters:
        self.drive_speed = self.get_parameter('drive_speed').value
        self.rotate_speed = self.get_parameter('rotate_speed').value
        self.obstacle_threshold = self.get_parameter('obstacle_threshold').value
        self.wall_follow_distance = self.get_parameter('wall_follow_distance').value
        self.kp = self.get_parameter('kp').value
        self.turning_gain = self.get_parameter('turning_gain').value
        self.sector_width = self.get_parameter('sector_width').value
        self.default_front_angle = self.get_parameter('default_front_angle').value
        self.default_right_angle = self.get_parameter('default_right_angle').value

        # Create TF buffer and listener for sensor frame transforms
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Navigation state: wait until triggered via /trigger_start
        self.start_navigation = False
        
        # Variables to store computed distances from sensor sectors
        self.front_distance = float('inf')
        self.right_distance = float('inf')
        
        # Subscribers:
        self.create_subscription(Empty, '/trigger_start', self.trigger_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        # Publishers:
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/snc_status', 10)
        
        # Control loop timer (10 Hz)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info("NavigationNode initialized. Waiting for trigger on /trigger_start.")

    def trigger_callback(self, msg: Empty):
        self.start_navigation = True
        self.get_logger().info("Trigger received. Starting navigation.")
        status = String()
        status.data = "Navigation Started"
        self.status_pub.publish(status)

    def compute_desired_angles(self, sensor_frame):
        """
        Uses TF to convert two reference vectors from base_link to the sensor_frame:
           - [1, 0] for forward.
           - [0, -1] for right.
        Returns (front_angle, right_angle) in the sensor frame (radians).
        """
        from geometry_msgs.msg import PointStamped
        try:
            now = self.get_clock().now().to_msg()
            transform = self.tf_buffer.lookup_transform(sensor_frame, 'base_link', now, timeout=rclpy.duration.Duration(seconds=0.1))
            
            point_base = PointStamped()
            point_base.header.frame_id = 'base_link'
            point_base.header.stamp = now
            
            # Forward vector (1, 0, 0) in base_link
            point_base.point.x = 1.0
            point_base.point.y = 0.0
            point_base.point.z = 0.0
            point_sensor = tf2_geometry_msgs.do_transform_point(point_base, transform)
            front_angle = math.atan2(point_sensor.point.y, point_sensor.point.x)
            
            # Right vector (0, -1, 0) in base_link
            point_base.point.x = 0.0
            point_base.point.y = -1.0
            point_sensor = tf2_geometry_msgs.do_transform_point(point_base, transform)
            right_angle = math.atan2(point_sensor.point.y, point_sensor.point.x)
            
            return front_angle, right_angle
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            return self.default_front_angle, self.default_right_angle

    def scan_callback(self, msg: LaserScan):
        sensor_frame = msg.header.frame_id
        front_angle, right_angle = self.compute_desired_angles(sensor_frame)
        
        # Create an array of angles for each beam in the scan.
        num_beams = len(msg.ranges)
        angles = [msg.angle_min + i * msg.angle_increment for i in range(num_beams)]
        
        # Select beams that fall within the desired front window.
        front_indices = [i for i, a in enumerate(angles)
                         if (front_angle - self.sector_width) <= a <= (front_angle + self.sector_width)]
        if front_indices:
            front_values = [msg.ranges[i] for i in front_indices if msg.ranges[i] > 0.0 and math.isfinite(msg.ranges[i])]
            self.front_distance = min(front_values) if front_values else float('inf')
        else:
            self.front_distance = float('inf')
        
        # Similarly, select beams for the right.
        right_indices = [i for i, a in enumerate(angles)
                         if (right_angle - self.sector_width) <= a <= (right_angle + self.sector_width)]
        if right_indices:
            right_values = [msg.ranges[i] for i in right_indices if msg.ranges[i] > 0.0 and math.isfinite(msg.ranges[i])]
            self.right_distance = min(right_values) if right_values else float('inf')
        else:
            self.right_distance = float('inf')
        
        # Optional debug logging:
        self.get_logger().debug(f"Computed angles: front = {front_angle:.2f}, right = {right_angle:.2f}")

    def timer_callback(self):
        twist = Twist()
        if not self.start_navigation:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_pub.publish(twist)
            return
        
        # Log the sensor readings:
        self.get_logger().info(f"Front distance: {self.front_distance:.2f} m, Right distance: {self.right_distance:.2f} m")
        
        # If an obstacle is detected in the front, stop and turn.
        if self.front_distance < self.obstacle_threshold:
            twist.linear.x = 0.0
            twist.angular.z = self.rotate_speed  # Turning right decisively; adjust if needed.
            self.get_logger().info("Obstacle detected ahead; executing avoidance maneuver (turning right).")
        else:
            # Otherwise, use wall following:
            error = self.wall_follow_distance - self.right_distance
            angular_correction = -self.kp * error * self.turning_gain
            twist.linear.x = self.drive_speed
            twist.angular.z = angular_correction
            self.get_logger().info(f"Wall following: error = {error:.2f}, correction = {angular_correction:.2f}")
            
        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("NavigationNode interrupted by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
