import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

goals = [(-5,-5),(5,5),(-5,5),(5,-5)]
goal_next = 0

class GoToGoal(Node):
    def __init__(self):
        super().__init__('go_to_goal_node')
        self.publisher = self.create_publisher(Twist, '/bcr_bot/cmd_vel', 10)
        self.subscription = self.create_subscription(Odometry, '/bcr_bot/odom', self.odom_callback, 10)

        
        self.goal_x,self.goal_y = goals[goal_next]
        
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        q = msg.pose.pose.orientation
        self.current_theta = 2 * math.atan2(q.z, q.w)
        
        self.move_to_goal()

    def move_to_goal(self):
        msg = Twist()
        global goal_next
        
        if goal_next >= 4:
            return
        
        distance_to_goal = math.sqrt((self.goal_x - self.current_x)**2 + (self.goal_y - self.current_y)**2)
        angle_to_goal = math.atan2(self.goal_y - self.current_y, self.goal_x - self.current_x)
        angle_error = angle_to_goal - self.current_theta
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

        if distance_to_goal < 0.1:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.get_logger().info(f"Goal {goal_next} Reached!")
            goal_next += 1
            if goal_next < 4:
                self.goal_x,self.goal_y = goals[goal_next]
            else:
                msg.linear.x = 0.0
                msg.angular.z = 0.0
        else:
            msg.linear.x = 0.5 * distance_to_goal 
            msg.angular.z = 0.5 * angle_error 
            
            msg.linear.x = min(msg.linear.x, 0.6)
            if msg.angular.z > 1.0:
                msg.angular.z = 1.0
            elif msg.angular.z < -1.0:
                msg.angular.z = -1.0

        self.publisher.publish(msg)

def main():
    try:
        rclpy.init()
        node = GoToGoal()
        rclpy.spin(node)
    except:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
