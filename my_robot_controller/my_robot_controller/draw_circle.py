#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class DrawCircleNode(Node):
    def __init__(self):
        # Initialize the node with the name "draw_circle"
        super().__init__("draw_circle")
        # Create a publisher for the /turtle1/cmd_vel topic, with the message type Twist
        self.cmd_vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        # Set up a timer to call send_velocity_command every 0.5 seconds
        self.timer = self.create_timer(0.5, self.send_velocity_command)
        self.get_logger().info("Draw circle node has been started")

    def send_velocity_command(self):
        # Create a new Twist message
        msg = Twist()
        # Set linear velocity in the x direction (forward speed)
        msg.linear.x = 2.0
        # Set angular velocity around the z-axis (rotation)
        msg.angular.z = 1.0
        # Publish the message
        self.cmd_vel_pub.publish(msg)

def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)
    # Create an instance of the DrawCircleNode
    node = DrawCircleNode()
    # Spin the node to keep it alive and responsive to timers
    rclpy.spin(node)
    # Shutdown rclpy after the node stops
    rclpy.shutdown()

# Run the main function if this script is executed directly
if __name__ == "__main__":
    main()
