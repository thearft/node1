#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PointStamped
import math
import tf2_ros
import tf2_geometry_msgs  # This provides the do_transform_point function

class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')
        
        # Basic tuning parameters:
        self.declare_parameter('drive_speed', 0.05)             # Forward speed (m/s)
        self.declare_parameter('rotate_speed', 0.2)             # Base turning speed (rad/s) for obstacle avoidance
        self.declare_parameter('obstacle_threshold', 0.2)       # Obstacle detection threshold (m)
        self.declare_parameter('wall_follow_distance', 1.0)     # Desired distance from the wall on the right (m)
        self.declare_parameter('kp', 1.0)                       # Proportional gain for wall following
        self.declare_parameter('turning_gain', 1.5)             # Extra gain for faster turns during wall following
        self.declare_parameter('sector_width', 0.5236)          # Half-width of the sector window in radians (~30°)
        
        # (Fallback) Desired angles in radians (if TF lookup fails)
        self.declare_parameter('default_front_angle', math.pi)      # Default: 180° (in sensor frame, if 0 is back)
        self.declare_parameter('default_right_angle', math.pi/2)      # Default: 90° (in sensor frame)
        
        # Retrieve parameters
        self.drive_speed = self.get_parameter('drive_speed').value
        self.rotate_speed = self.get_parameter('rotate_speed').value
        self.obstacle_threshold = self.get_parameter('obstacle_threshold').value
        self.wall_follow_distance = self.get_parameter('wall_follow_distance').value
        self.kp = self.get_parameter('kp').value
        self.turning_gain = self.get_parameter('turning_gain').value
        self.sector_width = self.get_parameter('sector_width').value
        self.default_front_angle = self.get_parameter('default_front_angle').value
        self.default_right_angle = self.get_parameter('default_right_angle').value
        
        # Create TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Navigation state
        self.start_navigation = False
        
        # These will store computed distances.
        self.front_distance = float('inf')
        self.right_distance = float('inf')
        
        # Subscribers
        self.create_subscription(Empty, '/trigger_start', self.trigger_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        # Publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Timer for control loop (0.1 s)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info("NavigationNode initialized, waiting for start trigger on /trigger_start")
        
    def trigger_callback(self, msg: Empty):
        self.start_navigation = True
        self.get_logger().info("Start trigger received. Navigation started.")
        
    def compute_desired_angles(self, sensor_frame):
        """
        Use TF to convert the forward vector and right vector from base_link into the sensor frame.
        Returns (front_angle, right_angle) in radians.
        """
        from geometry_msgs.msg import PointStamped
        try:
            # Lookup transform from sensor_frame to base_link.
            # We want to know what angle in sensor_frame corresponds to the base_link's forward direction.
            now = rclpy.time.Time()
            transform = self.tf_buffer.lookup_transform(sensor_frame, 'base_link', now, timeout=rclpy.duration.Duration(seconds=0.1))
            
            # Create a point in base_link corresponding to forward (x=1, y=0).
            point_base = PointStamped()
            point_base.header.frame_id = 'base_link'
            point_base.header.stamp = self.get_clock().now().to_msg()
            point_base.point.x = 1.0
            point_base.point.y = 0.0
            point_base.point.z = 0.0
            
            point_sensor = tf2_geometry_msgs.do_transform_point(point_base, transform)
            front_angle = math.atan2(point_sensor.point.y, point_sensor.point.x)
            
            # Similarly, create a point representing right in base_link ([0, -1, 0])
            point_base.point.x = 0.0
            point_base.point.y = -1.0
            point_sensor = tf2_geometry_msgs.do_transform_point(point_base, transform)
            right_angle = math.atan2(point_sensor.point.y, point_sensor.point.x)
            
            return front_angle, right_angle
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            return self.default_front_angle, self.default_right_angle
        
    def scan_callback(self, msg: LaserScan):
        # Get the sensor frame from the LaserScan message header.
        sensor_frame = msg.header.frame_id
        
        # Compute the desired front and right angles in the sensor frame using TF.
        desired_front_angle, desired_right_angle = self.compute_desired_angles(sensor_frame)
        
        # Create a list of angles for each beam.
        num_beams = len(msg.ranges)
        angles = [msg.angle_min + i * msg.angle_increment for i in range(num_beams)]
        
        # Select beams within the desired front window.
        front_indices = [i for i, a in enumerate(angles)
                         if (desired_front_angle - self.sector_width) <= a <= (desired_front_angle + self.sector_width)]
        if front_indices:
            front_values = [msg.ranges[i] for i in front_indices if msg.ranges[i] > 0.0 and math.isfinite(msg.ranges[i])]
            self.front_distance = min(front_values) if front_values else float('inf')
        else:
            self.front_distance = float('inf')
        
        # Select beams within the desired right window.
        right_indices = [i for i, a in enumerate(angles)
                         if (desired_right_angle - self.sector_width) <= a <= (desired_right_angle + self.sector_width)]
        if right_indices:
            right_values = [msg.ranges[i] for i in right_indices if msg.ranges[i] > 0.0 and math.isfinite(msg.ranges[i])]
            self.right_distance = min(right_values) if right_values else float('inf')
        else:
            self.right_distance = float('inf')
            
        # (For debugging, you can log the desired angles.)
        self.get_logger().debug(f"Desired front angle: {desired_front_angle:.2f}, desired right angle: {desired_right_angle:.2f}")
        
    def timer_callback(self):
        twist = Twist()
        
        if not self.start_navigation:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_pub.publish(twist)
            return
        
        self.get_logger().info(f"Front distance: {self.front_distance:.2f} m, Right distance: {self.right_distance:.2f} m")
        
        # If an obstacle is detected (front distance too low), then execute avoidance.
        if self.front_distance < self.obstacle_threshold:
            # When blocked, turn decisively. For example, always turn right if front is blocked.
            twist.linear.x = 0.0
            twist.angular.z = self.rotate_speed
            self.get_logger().info("Obstacle detected ahead; executing avoidance maneuver (turning right).")
        else:
            # Otherwise, follow the wall on the right.
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
