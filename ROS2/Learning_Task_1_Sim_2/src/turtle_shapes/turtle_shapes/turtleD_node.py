#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
import math

class D_Controller(Node):
    def __init__(self):
        super().__init__("D_Controller")
        self.get_logger().info("D Controller started.")
        self.time = 0.0
        self.publisher = self.create_publisher(Twist,"/turtle1/cmd_vel",10)
        
        self.control_time = 0.01
        self.linear_v = 1.5

        self.timer = self.create_timer(self.control_time,self.timer_callback)

    def timer_callback(self):
        publish_msg = Twist()
        if self.time < 1.0:
            publish_msg.linear.x = 0.0
            publish_msg.angular.z = math.pi/2
        elif self.time < 4.0:
            publish_msg.angular.z = 0.0
            publish_msg.linear.x = self.linear_v
        elif self.time < 5.0:
            publish_msg.linear.x = 0.0
            publish_msg.angular.z = -math.pi/2
        elif self.time < (2.25*math.pi/self.linear_v+5):
            publish_msg.angular.z = -self.linear_v/2.25 
            publish_msg.linear.x = self.linear_v
        else:
            publish_msg.linear.x = 0.0
            publish_msg.angular.z = 0.0

        self.publisher.publish(publish_msg)
        self.time += self.control_time

def main(args=None):
    try:
        rclpy.init(args=args)
        node = D_Controller()
        rclpy.spin(node)
    except:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == "__main__":
    main()