#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
import serial
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

class ArduinoBridge(Node):
    def __init__(self):
        super().__init__('arduino_bridge')
        
        # Parameters - Foxy style
        self.declare_parameter('port', '/dev/ttyACM1')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('timeout', 1.0)
        self.declare_parameter('steps_per_meter', 11459.0)
        self.declare_parameter('wheel_base', 0.206)
        
        # QoS profile for Foxy
        qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE)
        
        # Publishers/Subscribers
        self.step_pub = self.create_publisher(JointState, 'wheel_steps', qos)
        self.cmd_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_callback, qos)
        
        # Serial connection
        self.serial = None
        self.connect_serial()
        self.create_timer(0.01, self.read_serial)
        
        # Motion control
        self.current_left_speed = 0.0
        self.current_right_speed = 0.0
        self.max_accel = 10000.0  # steps/s²

    def connect_serial(self):
        try:
            self.serial = serial.Serial(
                port=self.get_parameter('port').value,
                baudrate=self.get_parameter('baudrate').value,
                timeout=self.get_parameter('timeout').value
            )
            self.get_logger().info(f"Connected to Arduino at {self.get_parameter('port').value}")
        except Exception as e:
            self.get_logger().error(f"Serial connection failed: {str(e)}")

    def cmd_callback(self, msg):
        if self.serial and self.serial.is_open:
            try:
                base_width = self.get_parameter('wheel_base').value
                steps_per_meter = self.get_parameter('steps_per_meter').value
                
                target_left = (msg.linear.x - (msg.angular.z * base_width/2)) * steps_per_meter
                target_right = (msg.linear.x + (msg.angular.z * base_width/2)) * steps_per_meter
                
                if msg.linear.x == 0.0 and msg.angular.z == 0.0:
                    self.current_left_speed = 0.0
                    self.current_right_speed = 0.0
                else:
                    # Apply acceleration limits only during movement
                    self.current_left_speed = self.limit_acceleration(
                        self.current_left_speed, target_left, self.max_accel)
                    self.current_right_speed = self.limit_acceleration(
                        self.current_right_speed, target_right, self.max_accel)
                
                # Send absolute position commands
                cmd_str = f"CMD:{int(self.current_left_speed)},{int(self.current_right_speed)}\n"
                self.serial.write(cmd_str.encode())
                
            except Exception as e:
                self.get_logger().error(f"Command error: {str(e)}")

    def limit_acceleration(self, current, target, max_accel):
        delta = target - current
        max_delta = max_accel * 0.1  # 100ms cycle time
        return current + max(-max_delta, min(delta, max_delta))

    def read_serial(self):
        if self.serial and self.serial.is_open:
            try:
                while self.serial.in_waiting > 0:
                    line = self.serial.readline().decode('utf-8').strip()
                    if line.startswith("STEPS:"):
                        try:
                            _, data = line.split(':')
                            left, right = map(int, data.split(','))
                            msg = JointState()
                            msg.header.stamp = self.get_clock().now().to_msg()
                            msg.name = ['left_wheel', 'right_wheel']
                            msg.position = [float(left), float(right)]
                            self.step_pub.publish(msg)
                        except ValueError:
                            self.get_logger().warn(f"Invalid data: {line}")
            except Exception as e:
                self.get_logger().error(f"Serial error: {str(e)}")
                self.connect_serial()

    def destroy_node(self):
        if self.serial and self.serial.is_open:
            self.serial.close()
        super().destroy_node()

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