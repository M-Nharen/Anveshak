import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math

class BCRBotObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('bcr_bot_avoidance')
        self.publisher_ = self.create_publisher(Twist, '/bcr_bot/cmd_vel', 10)
        self.subscription_odom = self.create_subscription(Odometry, '/bcr_bot/odom', self.odom_callback, 10)
        self.subscription_scan = self.create_subscription(LaserScan, '/bcr_bot/scan', self.scan_callback, 10)
        
        self.current_x, self.current_y, self.current_yaw = 0.0, 0.0, 0.0
        self.target_x, self.target_y = 10.0, 10.0
        self.state = "FORWARD"
        self.avoidance_threshold = 2.0

    def scan_callback(self, msg):
        front_ranges = msg.ranges[0:45] + msg.ranges[315:360]
        left_ranges = msg.ranges[45:120]
        right_ranges = msg.ranges[240:315]

        if min(front_ranges) < self.avoidance_threshold:
            if min(left_ranges) > min(right_ranges):
                self.state = "LEFT"
            else:
                self.state = "RIGHT"
        else:
            self.state = "FORWARD"

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.current_yaw =  2 * math.atan2(q.z, q.w)
        self.control_loop()

    def control_loop(self):
        msg = Twist()
        dx = self.target_x - self.current_x
        dy = self.target_y - self.current_y
        distance = math.sqrt(dx**2 + dy**2)
        
        if distance < 0.2:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.publisher_.publish(msg)
            return

        if self.state == "FORWARD":
            angle_to_goal = math.atan2(dy, dx)
            msg.linear.x = 0.5
            msg.angular.z = 1.0 * (angle_to_goal - self.current_yaw)
        elif self.state == "LEFT":
            msg.linear.x = 0.2
            msg.angular.z = 0.8
        elif self.state == "RIGHT":
            msg.linear.x = 0.2
            msg.angular.z = -0.8

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = BCRBotObstacleAvoidance()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
