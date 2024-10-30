#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        # Initialize the node with the name "first_node"
        super().__init__("first_node")
        self.counter_ = 0
        # Create a timer that triggers every 1 second (1.0 seconds)
        self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        # Print "Hello" to the console each time the timer is triggered
        self.get_logger().info("Hello" + str(self.counter_))
        self.counter_ += 1

def main(args=None):
    # Initialize rclpy library
    rclpy.init(args=args)
    # Create an instance of MyNode
    node = MyNode()
    # Spin the node to keep it running and executing callbacks
    rclpy.spin(node)
    # Shutdown rclpy once the node is stopped
    rclpy.shutdown()

# Run the main function if this script is executed directly
if __name__ == "__main__":
    main()
