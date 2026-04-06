#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8

class NumberNode(Node):
    def __init__(self):
        super().__init__("Number_Node")
        self.get_logger().info("Number Node Started")
        self.n = 1

        self.publisher = self.create_publisher(Int8,"/number",10)
        self.subscriber = self.create_subscription(Int8,"/count",self.listener_callback,10)

        self.timer = self.create_timer(1.0,self.timer_callback)


    def timer_callback(self):
        initializer = Int8()
        initializer.data = 1
        self.publisher.publish(initializer)
        self.timer.cancel()

    def listener_callback(self,msg):
        received_count = msg.data

        self.get_logger().info(f"Received Count {received_count} for number {self.n}")

        if self.n == received_count:
            self.n+=1

        numbermsg = Int8()
        numbermsg.data = self.n
        self.publisher.publish(numbermsg)

def main(args=None):
    try:
        rclpy.init(args=args)
        node = NumberNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == "__main__":
    main()