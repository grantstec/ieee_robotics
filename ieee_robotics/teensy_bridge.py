import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import serial
import numpy as np

class TeensyBridge(Node):
    def __init__(self):
        super().__init__('teensy_bridge')
        
        # Parameters
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('timeout', 1.0)
        
        # Publisher
        self.imu_pub = self.create_publisher(Imu, 'imu/data_raw', 10)
        
        # IMU calibration parameters
        self.orientation_covariance = [0.01] * 9
        self.angular_vel_covariance = [0.01] * 9
        self.linear_accel_covariance = [0.01] * 9
        
        # Serial connection
        self.connect_serial()
        self.create_timer(0.01, self.read_serial)

    def connect_serial(self):
        try:
            self.serial = serial.Serial(
                port=self.get_parameter('port').value,
                baudrate=self.get_parameter('baudrate').value,
                timeout=self.get_parameter('timeout').value
            )
            self.get_logger().info("Connected to Teensy")
        except serial.SerialException as e:
            self.get_logger().error(f"Teensy connection failed: {str(e)}")
            raise

    def read_serial(self):
        try:
            while self.serial.in_waiting > 0:
                line = self.serial.readline().decode('utf-8').strip()
                
                if line.startswith('ORI:'):
                    _, data = line.split(':')
                    x, y, z, w = map(float, data.split(','))
                    
                    msg = Imu()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = 'imu_link'
                    
                    # Quaternion orientation
                    msg.orientation.x = x
                    msg.orientation.y = y
                    msg.orientation.z = z
                    msg.orientation.w = w
                    msg.orientation_covariance = self.orientation_covariance
                    
                    self.imu_pub.publish(msg)
                    
        except Exception as e:
            self.get_logger().error(f"Teensy read error: {str(e)}")
            self.connect_serial()

def main(args=None):
    rclpy.init(args=args)
    node = TeensyBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()