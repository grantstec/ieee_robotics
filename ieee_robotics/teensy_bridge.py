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
                
                if line.startswith('IMU:'):
                    _, data = line.split(':')
                    values = list(map(float, data.split(',')))
                    
                    if len(values) != 10:  # Fixed length check
                        self.get_logger().warn("Invalid IMU data")
                        return
                    
                    msg = Imu()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = 'imu_link'
                    
                    # Orientation
                    msg.orientation.x = values[0]
                    msg.orientation.y = values[1]
                    msg.orientation.z = values[2]
                    msg.orientation.w = values[3]
                    
                    # Angular velocity (rad/s)
                    msg.angular_velocity.x = values[4]
                    msg.angular_velocity.y = values[5]
                    msg.angular_velocity.z = values[6]
                    
                    # Linear acceleration (m/s²)
                    msg.linear_acceleration.x = values[7]
                    msg.linear_acceleration.y = values[8]
                    msg.linear_acceleration.z = values[9]
                    
                    # Covariances (update these based on your sensor specs)
                    msg.orientation_covariance = [
                        0.01, 0.0, 0.0,
                        0.0, 0.01, 0.0,
                        0.0, 0.0, 0.01
                    ]
                    msg.angular_velocity_covariance = [
                        0.001, 0.0, 0.0,
                        0.0, 0.001, 0.0,
                        0.0, 0.0, 0.001
                    ]
                    
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