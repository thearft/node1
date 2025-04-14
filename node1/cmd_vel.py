#!/usr/bin/env python  

import rclpy
from rclpy.node import Node

import geometry_msgs.msg

class CmdVel(Node):
    def __init__(self):
        super().__init__('aiil_cmdvel')
        
        # Define Parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('driveSpeed', 2.0),
                ('rotateSpeed', 1.0)
            ]
        )
        
        # Look-up parameters values
        self.driveSpeed = self.get_parameter('driveSpeed').value
        self.rotateSpeed = self.get_parameter('rotateSpeed').value
        
        # Command Velocity Publisher
        self.topic = "/cmd_vel"
        self.pub = self.create_publisher(geometry_msgs.msg.Twist, self.topic, 10)
        
        # Timer callback
        self.timer = self.create_timer(2.0, self.transform)
    
    def transform(self):
        time = self.get_clock().now() - rclpy.duration.Duration(seconds=0.1)
        
        self.get_logger().info("--------------------------------")
        self.get_logger().info("Time:" + str(time))

        # Setup Twist message
        cmd_vel = geometry_msgs.msg.Twist()
        cmd_vel.linear.x = self.driveSpeed
        cmd_vel.angular.z = self.rotateSpeed
        
        # Publish
        self.get_logger().info(f"Sending command: {cmd_vel}")
        self.pub.publish(cmd_vel)

        # Update velocities
        self.driveSpeed = -1 * self.driveSpeed

def main():
    rclpy.init()
    node = CmdVel()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    exit(0)


