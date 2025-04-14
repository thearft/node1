#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')
        
        # Tuning parameters (adjusted for slower debugging)
        self.declare_parameter('drive_speed', 0.05)           # decreased forward speed (m/s)
        self.declare_parameter('rotate_speed', 0.1)          # decreased turning speed (rad/s)
        self.declare_parameter('obstacle_threshold', 0.05)     # obstacle threshold remains at 0.05 m
        self.declare_parameter('wall_follow_distance', 1.0)    # desired wall-following distance (m)
        self.declare_parameter('kp', 1.0)                      # proportional gain for wall following
        
        self.drive_speed = self.get_parameter('drive_speed').value
        self.rotate_speed = self.get_parameter('rotate_speed').value
        self.obstacle_threshold = self.get_parameter('obstacle_threshold').value
        self.wall_follow_distance = self.get_parameter('wall_follow_distance').value
        self.kp = self.get_parameter('kp').value
        
        # Navigation does not start until receiving a trigger.
        self.start_navigation = False
        
        # Dictionary to store minimum distances for each of 6 sectors.
        self.sector_distances = {
            'Right_Rear': float('inf'),
            'Right': float('inf'),
            'Front_Right': float('inf'),
            'Front_Left': float('inf'),
            'Left': float('inf'),
            'Left_Rear': float('inf')
        }
        
        # Subscribers for trigger and LaserScan data.
        self.create_subscription(Empty, '/trigger_start', self.trigger_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        # Publisher for /cmd_vel to control the robot
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Timer to call the control loop every 0.1 seconds.
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info("NavigationNode initialized, waiting for start trigger on /trigger_start")
        
    def trigger_callback(self, msg: Empty):
        self.start_navigation = True
        self.get_logger().info("Start trigger received. Navigation started.")
        
    def scan_callback(self, msg):
        # Divide the LaserScan into six sectors.
        ranges = list(msg.ranges)
        total_beams = len(ranges)
        if total_beams < 6:
            return  # not enough data; exit early
        sector_size = total_beams // 6
        sectors = {
            'Right_Rear': ranges[0:sector_size],
            'Right': ranges[sector_size:2*sector_size],
            'Front_Right': ranges[2*sector_size:3*sector_size],
            'Front_Left': ranges[3*sector_size:4*sector_size],
            'Left': ranges[4*sector_size:5*sector_size],
            'Left_Rear': ranges[5*sector_size:]
        }
        for sector, values in sectors.items():
            valid = [v for v in values if v > 0.0 and math.isfinite(v)]
            self.sector_distances[sector] = min(valid) if valid else float('inf')
        
    def timer_callback(self):
        twist = Twist()
        
        if not self.start_navigation:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_pub.publish(twist)
            return
        
        # Determine the minimum distance from the two front sectors.
        front_left = self.sector_distances.get('Front_Left', float('inf'))
        front_right = self.sector_distances.get('Front_Right', float('inf'))
        min_front = min(front_left, front_right)
        
        # If any front sensor detects an obstacle below the threshold, perform avoidance.
        if min_front < self.obstacle_threshold:
            # Decide turning direction based on which front side is more blocked.
            if front_left < front_right:
                twist.angular.z = self.rotate_speed  # Turn right.
                action = "Obstacle on front left; turning right."
            else:
                twist.angular.z = -self.rotate_speed # Turn left.
                action = "Obstacle on front right; turning left."
            twist.linear.x = 0.0
            self.get_logger().info(f"Obstacle detected: min front = {min_front:.2f} m. {action}")
        else:
            # Otherwise, perform wall following using the right sector.
            right_distance = self.sector_distances.get('Right', float('inf'))
            error = self.wall_follow_distance - right_distance
            angular_correction = -self.kp * error
            twist.linear.x = self.drive_speed
            twist.angular.z = angular_correction
            self.get_logger().info(
                f"Wall following: min front = {min_front:.2f} m, right = {right_distance:.2f} m, "
                f"error = {error:.2f}, correction = {angular_correction:.2f}"
            )
        
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
