import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math

class BCRBotObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('bcr_bot_avoidance')
        self.publisher_ = self.create_publisher(Twist, '/bcr_bot/cmd_vel', 10)
        self.subscription_odom = self.create_subscription(Odometry, '/bcr_bot/odom', self.odom_callback, 10)
        self.subscription_scan = self.create_subscription(LaserScan, '/bcr_bot/scan', self.scan_callback, 10)
        
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.obstacle_detected = False
        self.target_x = 10.0
        self.target_y = 10.0

    def scan_callback(self, msg):
        min_range = min(msg.ranges)
        if min_range < 0.5:
            self.obstacle_detected = True

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        q = msg.pose.pose.orientation
        self.current_yaw = 2 * math.atan2(q.z, q.w)
        
        self.control_loop()

    def control_loop(self):
        msg = Twist()
        
        if self.obstacle_detected:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.publisher_.publish(msg)
            return

        dx = self.target_x - self.current_x
        dy = self.target_y - self.current_y
        distance = math.sqrt(dx**2 + dy**2)
        angle_to_goal = math.atan2(dy, dx)

        if distance > 0.1:
            msg.linear.x = 0.5
            msg.angular.z = 1.5 * (angle_to_goal - self.current_yaw)
        else:
            msg.linear.x = 0.0
            msg.angular.z = 0.0

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = BCRBotObstacleAvoidance()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
