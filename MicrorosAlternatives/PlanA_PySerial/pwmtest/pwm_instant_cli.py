import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import threading
import sys

class PwmInstantCli(Node):
    def __init__(self):
        super().__init__('pwm_instant_cli')
        self.publisher = self.create_publisher(Twist, 'cmd_pwm', 10)
        self.get_logger().info("Real-Time CLI Active.")
        
        # Non-blocking input loop run on a standalone thread worker
        self.input_thread = threading.Thread(target=self.raw_input_loop, daemon=True)
        self.input_thread.start()

    def raw_input_loop(self):
        print("\n" + "="*40)
        print(" REAL-TIME DIRECT PWM CONTROL STEPPER")
        print("="*40)
        print("Format: [Left PWM],[Right PWM] followed by Enter.")
        print("Example: 200,-150\n")
        
        while rclpy.ok():
            try:
                # Direct block read from sys stdin pipeline
                sys.stdout.write("Enter PWM (L,R): ")
                sys.stdout.flush()
                user_line = sys.stdin.readline().strip()
                
                if not user_line:
                    continue
                
                parts = user_line.split(',')
                if len(parts) == 2:
                    left = max(-255, min(255, int(parts[0])))
                    right = max(-255, min(255, int(parts[1])))
                    
                    # Package directly into structural ROS topology
                    msg = Twist()
                    msg.linear.x = float(left)
                    msg.angular.z = float(right)
                    
                    self.publisher.publish(msg)
                else:
                    print("\n[!] Invalid format. Use: Left,Right (e.g. 100,100)")
            except ValueError:
                print("\n[!] Integers numbers only.")
            except (KeyboardInterrupt, EOFError):
                break

def main(args=None):
    rclpy.init(args=args)
    node = PwmInstantCli()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
