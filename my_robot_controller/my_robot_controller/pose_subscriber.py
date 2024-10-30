#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose

class PoseSubscriberNode(Node):
    def __init__(self):
        # Initialize the node with the name "pose_subscriber"
        super().__init__("pose_subscriber")
        # Create a subscriber for the /turtle1/pose topic
        self.pose_subscriber = self.create_subscription(
            Pose,
            "/turtle1/pose",
            self.pose_callback,
            10  # Queue size
        )
        self.get_logger().info("Pose subscriber node has been started")

    def pose_callback(self, msg: Pose):
        # Log the x and y position from the Pose message
        self.get_logger().info("Turtle Position: x = " + str(msg.x) + ", y = " + str(msg.y))

def main(args=None):
    # Initialize rclpy
    rclpy.init(args=args)
    # Create an instance of PoseSubscriberNode
    node = PoseSubscriberNode()
    # Spin the node to keep it running
    rclpy.spin(node)
    # Shutdown rclpy after the node stops
    rclpy.shutdown()

# Run the main function if this script is executed directly
if __name__ == "__main__":
    main()
