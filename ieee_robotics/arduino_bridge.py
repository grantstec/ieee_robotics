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
        
        # Steps per revolution and wheel radius for accurate math
        self.declare_parameter('microsteps', 16)               # TMC2209 microsteps
        self.declare_parameter('steps_per_rev', 200)           # Standard stepper motor steps
        self.declare_parameter('wheel_radius', 0.045)          # Wheel radius in meters
        
        # Get parameter values
        self.microsteps = self.get_parameter('microsteps').value
        self.steps_per_rev = self.get_parameter('steps_per_rev').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        
        # Calculate steps per radian for accurate conversion
        # steps_per_rad = (steps_per_rev * microsteps) / (2 * pi)
        self.steps_per_rad = (self.steps_per_rev * self.microsteps) / (2 * math.pi)
        
        # Log the configuration
        self.get_logger().info(f"Motor configuration: {self.steps_per_rev} steps/rev, {self.microsteps} microsteps")
        self.get_logger().info(f"Wheel radius: {self.wheel_radius}m")
        self.get_logger().info(f"Conversion factor: {self.steps_per_rad} steps/radian")
        
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
        # Extract left/right wheel angular velocities (rad/s)
        left_angular_vel = msg.linear.x
        right_angular_vel = msg.linear.y
        
        # Convert angular velocity (rad/s) to steps/s
        # steps/s = angular_velocity (rad/s) * steps_per_rad
        left_steps_per_sec = left_angular_vel * self.steps_per_rad
        right_steps_per_sec = right_angular_vel * self.steps_per_rad
        
        # Store for sending
        self.last_left_speed = left_steps_per_sec
        self.last_right_speed = right_steps_per_sec
        
        # Log conversion steps for debugging
        self.get_logger().debug(
            f"Angular velocity: L={left_angular_vel:.3f} rad/s, R={right_angular_vel:.3f} rad/s"
        )
        self.get_logger().debug(
            f"Steps/sec: L={left_steps_per_sec:.1f}, R={right_steps_per_sec:.1f}"
        )

    def send_command(self):
        """Send velocity commands to Arduino at fixed rate"""
        if not self.serial or not self.serial.is_open:
            return
            
        try:
            # Send velocity command
            cmd_str = f"VEL:{self.last_left_speed:.1f},{self.last_right_speed:.1f}\n"
            self.serial.write(cmd_str.encode())
            
            # Log non-zero commands
            if abs(self.last_left_speed) > 0 or abs(self.last_right_speed) > 0:
                self.get_logger().debug(f"Sent: {cmd_str.strip()}")
            
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