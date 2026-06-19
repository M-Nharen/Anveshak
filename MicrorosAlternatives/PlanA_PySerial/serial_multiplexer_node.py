import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32
import serial
import threading
from cobs import cobs
import rover_proto_pb2  # Generated via: protoc --python_out=. rover_proto.proto

class SerialMultiplexerNode(Node):
    def __init__(self):
        super().__init__('serial_multiplexer_node')
        
        # Open serial port to ESP32
        self.serial_port = serial.Serial('/dev/ttyACM0', 115200, timeout=0.01, write_timeout=0.01)
        self.serial_lock = threading.Lock()
        
        # ROS2 Subscriptions
        self.cmd_vel_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.led_sub = self.create_subscription(Bool, 'toggle_led', self.led_callback, 10)
        
        # ROS2 Publishers
        self.battery_pub = self.create_publisher(Float32, 'rover/battery', 10)
        
        # Start background receiver thread
        self.running = True
        self.read_thread = threading.Thread(target=self.serial_receive_loop, daemon=True)
        self.read_thread.start()
        self.get_logger().info("Serial Multiplexer Pipeline Initialized.")

    def send_to_esp(self, packet):
        serialized_data = packet.SerializeToString()
        encoded_packet = cobs.encode(serialized_data) + b'\x00'
        
        with self.serial_lock:
            try:
                self.serial_port.write(encoded_packet)
            except (serial.SerialTimeoutException, serial.SerialException) as e:
                self.get_logger().error(f"Serial write failed: {e}")

    def cmd_vel_callback(self, msg):
        packet = rover_proto_pb2.JetsonToEspPacket()
        packet.motion.linear_x = msg.linear.x
        packet.motion.angular_z = msg.angular.z
        self.send_to_esp(packet)

    def led_callback(self, msg):
        packet = rover_proto_pb2.JetsonToEspPacket()
        packet.led.b_led_on = msg.data
        self.send_to_esp(packet)

    def serial_receive_loop(self):
        buffer = bytearray()
        while self.running and rclpy.ok():
            try:
                if self.serial_port.in_waiting > 0:
                    data = self.serial_port.read(self.serial_port.in_waiting)
                    for byte in data:
                        if byte == 0x00:
                            if buffer:
                                self.process_incoming_packet(buffer)
                                buffer.clear()
                        else:
                            buffer.append(byte)
            except Exception as e:
                self.get_logger().error(f"Error in read loop: {e}")

    def process_incoming_packet(self, buffer):
        try:
            # Attempt to decode COBS and parse Protobuf
            decoded_data = cobs.decode(bytes(buffer))
            packet = rover_proto_pb2.EspToJetsonPacket()
            packet.ParseFromString(decoded_data)
            
            which_field = packet.WhichOneof('packet_type')
            if which_field == 'battery':
                ros_msg = Float32()
                ros_msg.data = packet.battery.voltage
                self.battery_pub.publish(ros_msg)
                
        except Exception:
            # Catch plain text logs from standard Serial.println()
            try:
                raw_text = buffer.decode('utf-8', errors='ignore').strip()
                if raw_text:
                    self.get_logger().info(f"[ESP32 DBG]: {raw_text}")
            except Exception:
                pass

    def destroy_node(self):
        self.running = False
        self.serial_port.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SerialMultiplexerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()