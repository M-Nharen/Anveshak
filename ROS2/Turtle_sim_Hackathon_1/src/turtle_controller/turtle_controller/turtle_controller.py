#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from rclpy.node import Node
import math

class D_Controller(Node):
    def __init__(self):
        super().__init__("D_Controller")
        self.get_logger().info("Controller started.")
        self.time = 0.0
        self.publisher = self.create_publisher(Twist,"/turtle1/cmd_vel",10)
        self.subscriber = self.create_subscription(Pose,'/turtle1/pose', self.listener_callback, 10)
        self.ref_theta = 0.0
        self.detected = False
        self.bounce1 = False
    def listener_callback(self, msg):
        x = msg.x
        y = msg.y
        theta = msg.theta
        print(x,y,theta)
        publish_msg = Twist()
        if(y<0.3 or y>10.7): #horizontal
            self.bounce1 = True
            if self.detected==False:
                self.ref_theta = theta
                self.detected = True
            publish_msg.linear.x = 0.0
            publish_msg.angular.z = 0.3
            print(self.ref_theta)
            if(theta<=((-1*self.ref_theta)+0.1) and theta>=((-1*self.ref_theta)-0.1)):
                self.detected = False
                publish_msg.linear.x = 10.0
                
        elif (x>10.7 or x<0.3):
            self.bounce1 = True
            if self.detected==False:
                self.ref_theta = theta
                self.detected=True
            publish_msg.linear.x = 0.0
            publish_msg.angular.z = 0.3

            final_angle = math.pi - self.ref_theta

            if(final_angle>math.pi):
               final_angle = final_angle - 2*(math.pi)
            if(final_angle<(-1*math.pi)):
                final_angle = final_angle + 2*(math.pi)
            print(final_angle)
            if(theta<=((final_angle)+0.1) and theta>=((final_angle-0.1))):
                self.detected = False
                publish_msg.linear.x = 10.0
                    
        else:
            if(self.bounce1==False):
                publish_msg.linear.x = 0.5
                publish_msg.angular.z = 0.5/(math.sqrt((x-5.54)*(x-5.54) + (y-5.54)*(y-5.54)) + 0.1)
            else:
                publish_msg.linear.x = 1.0
                publish_msg.angular.z = 0.0

        self.publisher.publish(publish_msg)

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