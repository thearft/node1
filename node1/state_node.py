#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy

class StateNode(Node):
    def __init__(self):
        super().__init__('state_server')
        self.publisher_ = self.create_publisher(
            String,
            '/snc_state',
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )
        self.timer = self.create_timer(5.0, self.publish_state)
        self.get_logger().info("State Node started. Publishing state periodically.")

    def publish_state(self):
        msg = String()
        # For testing, we publish "Exploring"
        msg.data = "Exploring"
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published state: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = StateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
