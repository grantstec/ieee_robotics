# [!] ADAPTATION REQUIRED [!]
# ---------------------------
# 1. Verify serial port matches your Arduino connection
#    Check using `ls /dev/tty*` when Arduino is connected
# 2. Confirm baudrate matches Arduino code (115200 in provided example)
# 3. Adjust timeout if experiencing connection drops

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import serial
import serial.tools.list_ports

class ArduinoBridge(Node):
    def __init__(self):
        super().__init__('arduino_bridge')
        
        # Parameters
        self.declare_parameter('port', '/dev/ttyACM0')  # [!] Might be ACM1 or other
        self.declare_parameter('baudrate', 115200)      # [!] Match Arduino code
        self.declare_parameter('timeout', 1.0)          # [!] Increase if needed
        
        # Publisher
        self.step_pub = self.create_publisher(JointState, 'wheel_steps', 10)
        
        # Serial connection
        self.connect_serial()
        
        # Timer for reading serial
        self.create_timer(0.01, self.read_serial)

    def connect_serial(self):
        port = self.get_parameter('port').value
        baud = self.get_parameter('baudrate').value
        timeout = self.get_parameter('timeout').value
        
        try:
            self.serial = serial.Serial(
                port=port,
                baudrate=baud,
                timeout=timeout
            )
            self.get_logger().info(f"Connected to Arduino at {port}")
        except serial.SerialException as e:
            self.get_logger().error(f"Serial connection failed: {str(e)}")
            raise

    def read_serial(self):
        try:
            while self.serial.in_waiting > 0:
                line = self.serial.readline().decode('utf-8').strip()
                
                # [!] Verify message format matches Arduino output:
                if line.startswith('STEPS:'):  # Must match Arduino's Serial.print
                    _, data = line.split(':')
                    left, right = map(int, data.split(','))
                    
                    msg = JointState()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.name = ['left_wheel', 'right_wheel']
                    msg.position = [left, right]
                    self.step_pub.publish(msg)
                    
        except Exception as e:
            self.get_logger().error(f"Serial read error: {str(e)}")
            self.connect_serial()  # Reconnect attempt

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()