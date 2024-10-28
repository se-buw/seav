#!/usr/bin/env python3 
#above is interpreter

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class DrawCircleNode(Node):
    
    def __init__(self):
        super().__init__("draw_circle")
        self.cmv_vel_pub_ = self.create_publisher(Twist, "/turtle1/cmd_vel",10)
        self.timer = self.create_timer(0.5, self.send_velocity_command)
        self.get_logger().info("Draw circle node has been started ")
     
    def send_velocity_command(self):
        msg = Twist()
        #now we want to send something
        msg.linear.x = 2.0
        msg.angular.z = 1.0
        #in above we have created a msg, now have to publish a message
        
        self.cmv_vel_pub_.publish(msg)
        
def main(args=None):
    rclpy.init(args=args)
    node = DrawCircleNode()
    rclpy.spin(node) #without spin the issue is there , by not calling function continously and takes exit
    rclpy.shutdown()