#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
import serial
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
import time

class VelocityArduinoBridge(Node):
    def __init__(self):
        super().__init__('velocity_arduino_bridge')
        
        # Parameters
        self.declare_parameter('port', '/dev/ttyACM1')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('timeout', 1.0)
        self.declare_parameter('steps_per_meter', 11459.0)  # Calibrate this value
        self.declare_parameter('wheel_base', 0.206)         # Wheel separation in meters
        self.declare_parameter('debug_output', True)        # Print debug info
        
        # Get parameter values
        self.debug_output = self.get_parameter('debug_output').value
        self.steps_per_meter = self.get_parameter('steps_per_meter').value
        
        # QoS profile 
        qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE)
        
        # Publishers/Subscribers
        self.step_pub = self.create_publisher(JointState, 'wheel_steps', qos)
        self.wheel_cmd_sub = self.create_subscription(Twist, 'wheel_commands', self.wheel_cmd_callback, qos)
        
        # Serial connection
        self.serial = None
        self.connect_serial()
        self.read_timer = self.create_timer(0.01, self.read_serial)
        
        # Command tracking
        self.last_left_speed = 0.0   # steps/second
        self.last_right_speed = 0.0  # steps/second
        
        # Create command sender timer (10Hz)
        self.send_timer = self.create_timer(0.1, self.send_command)
        
        self.get_logger().info(f"Velocity Arduino Bridge started (steps_per_meter={self.steps_per_meter})")

    def connect_serial(self):
        try:
            self.serial = serial.Serial(
                port=self.get_parameter('port').value,
                baudrate=self.get_parameter('baudrate').value,
                timeout=self.get_parameter('timeout').value
            )
            self.get_logger().info(f"Connected to Arduino at {self.get_parameter('port').value}")
            # Wait for Arduino to reset after serial connection
            time.sleep(2.0)
            # Flush serial buffer
            self.serial.reset_input_buffer()
            self.serial.reset_output_buffer()
        except Exception as e:
            self.get_logger().error(f"Serial connection failed: {str(e)}")

    def wheel_cmd_callback(self, msg):
        """Handle wheel commands from twist_to_motors node"""
        # Extract left/right wheel speeds
        left_vel_m_per_s = msg.linear.x   # Left wheel velocity in m/s
        right_vel_m_per_s = msg.linear.y  # Right wheel velocity in m/s
        
        # Convert to steps per second with high precision
        left_steps_per_sec = left_vel_m_per_s * self.steps_per_meter
        right_steps_per_sec = right_vel_m_per_s * self.steps_per_meter
        
        # Store for sending
        self.last_left_speed = left_steps_per_sec
        self.last_right_speed = right_steps_per_sec
        
        if self.debug_output:
            self.get_logger().info(
                f"Wheel cmd: L={left_vel_m_per_s:.3f} m/s -> {left_steps_per_sec:.1f} steps/s, "
                f"R={right_vel_m_per_s:.3f} m/s -> {right_steps_per_sec:.1f} steps/s"
            )

    def send_command(self):
        """Send velocity commands to Arduino at fixed rate"""
        if not self.serial or not self.serial.is_open:
            return
            
        try:
            # Send velocity command
            cmd_str = f"VEL:{self.last_left_speed:.1f},{self.last_right_speed:.1f}\n"
            self.serial.write(cmd_str.encode())
            
            if self.debug_output and (abs(self.last_left_speed) > 0 or abs(self.last_right_speed) > 0):
                self.get_logger().info(f"Sent: {cmd_str.strip()}")
            
        except Exception as e:
            self.get_logger().error(f"Failed to send command: {str(e)}")

    def read_serial(self):
        """Read position feedback from Arduino"""
        if not self.serial or not self.serial.is_open:
            return
            
        try:
            while self.serial.in_waiting > 0:
                line = self.serial.readline().decode('utf-8').strip()
                
                # Print debug messages from Arduino
                if line.startswith("MEGA:"):
                    self.get_logger().info(f"Arduino: {line[5:]}")
                
                # Process position updates
                elif line.startswith("STEPS:"):
                    try:
                        _, data = line.split(':')
                        left, right = map(int, data.split(','))
                        
                        # Create and publish joint state message
                        msg = JointState()
                        msg.header.stamp = self.get_clock().now().to_msg()
                        msg.name = ['left_wheel', 'right_wheel']
                        msg.position = [float(left), float(right)]
                        
                        self.step_pub.publish(msg)
                    except ValueError:
                        self.get_logger().warn(f"Invalid data: {line}")
                # Log any other output from Arduino
                elif len(line) > 0:
                    self.get_logger().info(f"Arduino output: {line}")
                    
        except Exception as e:
            self.get_logger().error(f"Serial read error: {str(e)}")
            # Try to reconnect
            self.connect_serial()

    def destroy_node(self):
        """Clean shutdown"""
        if self.serial and self.serial.is_open:
            # Send stop command before closing
            try:
                self.serial.write(b"VEL:0.0,0.0\n")
                time.sleep(0.1)  # Brief delay to allow command to be sent
                self.serial.close()
            except:
                pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = VelocityArduinoBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()