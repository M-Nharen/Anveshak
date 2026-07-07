import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import threading
from cobs import cobs
import pwm_proto_pb2 

class SerialBridgeNode(Node):
    def __init__(self):
        super().__init__('serial_bridge_node')
        
        # Configure hardware link with microsecond level timeouts
        self.serial_port = serial.Serial('/dev/ttyACM0', 115200, timeout=0.001, write_timeout=0.001)
        self.subscription = self.create_subscription(Twist, 'cmd_pwm', self.pwm_callback, 10)
        
        # Immediate extraction reader thread 
        self.running = True
        self.rx_thread = threading.Thread(target=self.receive_loop, daemon=True)
        self.rx_thread.start()
        
        self.get_logger().info("Bi-directional Pipeline Gateway initialized.")

    def pwm_callback(self, msg):
        packet = pwm_proto_pb2.JetsonToEspPacket()
        packet.pwm.left_pwm = int(msg.linear.x)
        packet.pwm.right_pwm = int(msg.angular.z)

        # Apply COBS Framing limits
        cobs_frame = cobs.encode(packet.SerializeToString()) + b'\x00'
        try:
            self.serial_port.write(cobs_frame)
            self.serial_port.flush()  # Force data onto physical copper line instantly
        except serial.SerialException as e:
            self.get_logger().error(f"Serial transmission crash: {e}")

    def receive_loop(self):
        buffer = bytearray()
        while self.running and rclpy.ok():
            try:
                if self.serial_port.in_waiting > 0:
                    data = self.serial_port.read(self.serial_port.in_waiting)
                    for byte in data:
                        if byte == 0x00:
                            if buffer:
                                self.process_upstream_packet(buffer)
                                buffer.clear()
                        else:
                            buffer.append(byte)
            except Exception:
                pass

    def process_upstream_packet(self, buffer):
        try:
            decoded = cobs.decode(bytes(buffer))
            packet = pwm_proto_pb2.EspToJetsonPacket()
            packet.ParseFromString(decoded)
            
            if packet.WhichOneof('packet_type') == 'log':
                # Direct real-time terminal printout rewrite
                sys.stdout.write(f"\r[ESP32 MONITOR]: {packet.log.text}\n")
                sys.stdout.write("Enter PWM (L,R): ")
                sys.stdout.flush()
        except Exception:
            pass

    def destroy_node(self):
        self.running = False
        self.serial_port.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SerialBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
