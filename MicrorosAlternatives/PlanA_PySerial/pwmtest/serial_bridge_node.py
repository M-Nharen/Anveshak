import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray # Adjust message type if yours is different
import serial
from cobs import cobs
import pwm_proto_pb2  # Your compiled protobuf file

class SerialBridgeNode(Node):
    def __init__(self):
        super().__init__('serial_bridge_node')
        
        # Open port & instantly wipe out residual connection garbage
        self.serial_port = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)
        self.serial_port.reset_input_buffer()
        self.serial_port.reset_output_buffer()

        # Subscription to receive commands from ROS 2 environment
        self.subscription = self.create_subscription(
            Int32MultiArray,
            '/cmd_pwm',
            self.pwm_callback,
            10
        )
        self.get_logger().info("Bi-directional Pipeline Gateway initialized cleanly.")

    def pwm_callback(self, msg):
        if len(msg.data) < 2:
            return

        try:
            # 1. Build Protobuf Structure
            tx_packet = pwm_proto_pb2.JetsonToEspPacket()
            tx_packet.pwm.left_pwm = msg.data[0]
            tx_packet.pwm.right_pwm = msg.data[1]

            # 2. Serialize and Encode using COBS
            serialized = tx_packet.SerializeToString()
            encoded = cobs.encode(serialized)

            # 3. CRITICAL FIX: Append structural framing delimiter byte 
            final_packet = encoded + b'\x00'

            # 4. Transmit over wire
            self.serial_port.write(final_packet)
            self.get_logger().info(f"Sent PWM -> L: {msg.data[0]}, R: {msg.data[1]}")

        except Exception as e:
            self.get_logger().error(f"Failed to encode or send packet: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SerialBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.serial_port.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()