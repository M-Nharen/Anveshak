#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8

class CounterNode(Node):
    def __init__(self):
        super().__init__("Counter_Node")
        self.get_logger().info("Counter Node Started")
        self.counter = 0

        self.subscription = self.create_subscription(Int8,"/number",self.listener_callback,10)
        self.publisher = self.create_publisher(Int8,"/count",10)

    def listener_callback(self,msg):
        received_number = msg.data
        self.counter += 1

        self.get_logger().info(f"Count of {received_number} is {self.counter}.")

        count_msg = Int8()
        count_msg.data = self.counter
        self.publisher.publish(count_msg)

        if self.counter >= received_number:
            self.counter = 0


def main(args=None):
    try:
        rclpy.init(args=args)
        node = CounterNode()
        rclpy.spin(node)
    except:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == "__main__":
    main()