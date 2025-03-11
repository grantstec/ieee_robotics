import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped, Quaternion
import serial
import numpy as np
import time
import math


# Implementation of quaternion_from_euler to avoid tf_transformations dependency
def quaternion_from_euler(roll, pitch, yaw):
    """
    Convert Euler angles to quaternion
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = [0, 0, 0, 0]
    q[0] = sr * cp * cy - cr * sp * sy  # x
    q[1] = cr * sp * cy + sr * cp * sy  # y
    q[2] = cr * cp * sy - sr * sp * cy  # z
    q[3] = cr * cp * cy + sr * sp * sy  # w

    return q


class TeensyBridge(Node):
    # State machine states for fire detection
    (STATE_INITIAL,
     STATE_CHECK_0_DEGREES,
     STATE_CHECK_45_DEGREES_LEFT,
     STATE_CHECK_90_DEGREES,
     STATE_RESET) = range(5)

    def __init__(self):
        super().__init__('teensy_bridge')
      
        # Parameters
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('timeout', 1.0)
      
        # Fire detection state machine parameters
        self.goal_active = self.declare_parameter('goal_active', True).value
        self.current_state = self.STATE_INITIAL
        self.last_state_change_time = 0
        self.fire_locations = []
      
        # Initiate the goal poses
        goal1 = PoseStamped()
        goal2 = PoseStamped()
        goal3 = PoseStamped()
        goal1 = PoseStamped()
        goal1.header.frame_id = "map"
        goal1.header.stamp = self.get_clock().now().to_msg()
        goal1.pose.position.x = 2.0
        goal1.pose.position.y = 0.0
        goal1.pose.position.z = 0.0
        goal1.pose.orientation.x = 0.0
        goal1.pose.orientation.y = 0.0
        goal1.pose.orientation.z = 0.0  # Keep same orientation as start
        goal1.pose.orientation.w = 1.0

        goal2 = PoseStamped()
        goal2.header.frame_id = "map"
        goal2.header.stamp = self.get_clock().now().to_msg()
        goal2.pose.position.x = 2.0
        goal2.pose.position.y = 2.0
        goal2.pose.position.z = 0.0
        # For 45 degrees rotation, use quaternion values:
        goal2.pose.orientation.x = 0.0
        goal2.pose.orientation.y = 0.0
        goal2.pose.orientation.z = 0.3826834  # sin(45°/2)
        goal2.pose.orientation.w = 0.9238795  # cos(45°/2)

        goal3 = PoseStamped()
        goal3.header.frame_id = "map"
        goal3.header.stamp = self.get_clock().now().to_msg()
        goal3.pose.position.x = 0.0
        goal3.pose.position.y = 2.0
        goal3.pose.position.z = 0.0
        # For 90 degrees rotation, use quaternion values:
        goal3.pose.orientation.x = 0.0
        goal3.pose.orientation.y = 0.0
        goal3.pose.orientation.z = 0.7071068  # sin(90°/2)
        goal3.pose.orientation.w = 0.7071068  # cos(90°/2)
        self.final_destinations = [goal1, goal2, goal3]
       
        # Serial data tracking
        self.current_fire_detection = None
      
        # Publishers
        self.imu_pub = self.create_publisher(Imu, 'imu/data_raw', 10)
        self.goal_pub = self.create_publisher(PoseStamped, 'goal_pose', 10)
      
        # Serial connection
        self.connect_serial()
      
        # Single timer for both serial reading and state machine
        self.create_timer(0.01, self.process_serial_and_state_machine)

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
            # Don't raise the exception, just log it and set serial to None
            self.serial = None

    def process_serial_and_state_machine(self):
        """
        Combined method to read serial data and manage state machine
        Ensures state machine uses most recent serial data
        """
        
        if self.serial is None:
            # Try to reconnect if serial is not connected
            self.get_logger().info("Attempting to reconnect to Teensy...")
            self.connect_serial()
            return
    
        try:
            # Reset fire detection before processing
            self.current_fire_detection = None
            
            # Check if serial is available
            if self.serial.in_waiting <= 0:
                return
                
            # Read all available serial data
            line = self.serial.readline().decode('utf-8').strip()
            if not line:
                return
              
            # Process IMU data
            if line.startswith('IMU:'):
                self.process_imu_data(line)
              
            # Process fire detection data
            elif line.startswith('FIRE:'):
                self.current_fire_detection = '1' in line
                self.get_logger().info(f"Fire detection: {self.current_fire_detection}")
                
                # State machine logic using the most recent fire detection data
                if self.goal_active:
                    self.fire_detection_state_machine(line)
                    
        except Exception as e:
            self.get_logger().error(f"Serial processing error: {str(e)}")
            try:
                # Close and reopen the serial connection
                if self.serial is not None:
                    self.serial.close()
                self.connect_serial()
            except:
                self.get_logger().error("Failed to reconnect to serial")
                self.serial = None

    def process_imu_data(self, line):
        """Process IMU data from serial line"""
        try:
            _, data = line.split(':')
            values = list(map(float, data.split(',')))
          
            if len(values) != 10:
                self.get_logger().warn(f"Invalid IMU data: expected 10 values, got {len(values)}")
                return
          
            msg = Imu()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'imu_link'
          
            # Populate IMU message
            msg.orientation.x = values[0]
            msg.orientation.y = values[1]
            msg.orientation.z = values[2]
            msg.orientation.w = values[3]
          
            msg.angular_velocity.x = values[4]
            msg.angular_velocity.y = values[5]
            msg.angular_velocity.z = values[6]
          
            msg.linear_acceleration.x = values[7]
            msg.linear_acceleration.y = values[8]
            msg.linear_acceleration.z = values[9]
          
            msg.orientation_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
            msg.angular_velocity_covariance = [0.001, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.001]
            msg.linear_acceleration_covariance = [0.001, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.001]
          
            self.imu_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"IMU data processing error: {str(e)}")

    def fire_detection_state_machine(self, line):
        """
        State machine for fire detection using the most recent serial data
        Requires self.current_fire_detection to be set before calling
        """
        current_time = time.time()
        # State machine logic for fire detection and navigation
        if self.current_state == self.STATE_INITIAL:
            # Start checking at 0 degrees
            self.send_rotation_goal(0)
            self.current_state = self.STATE_CHECK_0_DEGREES
            self.last_state_change_time = current_time
            self.get_logger().info("State: Initial -> Check 0 Degrees")
            
        elif self.current_state == self.STATE_CHECK_0_DEGREES:
            # Check if we've waited long enough to detect fire
            if current_time - self.last_state_change_time > 5:
                if self.check_fire_detection(line):
                    self.publish_fire_location(0)
                    self.get_logger().info("Fire detected at 0 degrees")
                else:
                    # Move to 45 degrees left
                    self.send_rotation_goal(45)
                    self.current_state = self.STATE_CHECK_45_DEGREES_LEFT
                    self.last_state_change_time = current_time
                    self.get_logger().info("State: Check 0 Degrees -> Check 45 Degrees Left")
                    
        elif self.current_state == self.STATE_CHECK_45_DEGREES_LEFT:
            if current_time - self.last_state_change_time > 5:
                if self.check_fire_detection(line):
                    self.publish_fire_location(45)
                    self.get_logger().info("Fire detected at 45 degrees")
                else:
                    # Move to 90 degrees
                    self.send_rotation_goal(90)
                    self.current_state = self.STATE_CHECK_90_DEGREES
                    self.last_state_change_time = current_time
                    self.get_logger().info("State: Check 45 Degrees Left -> Check 90 Degrees")
                    
        elif self.current_state == self.STATE_CHECK_90_DEGREES:
            if current_time - self.last_state_change_time > 5:
                if self.check_fire_detection(line):
                    self.publish_fire_location(90)
                    self.get_logger().info("Fire detected at 90 degrees")
                else:
                    # Reset to initial position
                    self.send_rotation_goal(0)
                    self.current_state = self.STATE_RESET
                    self.last_state_change_time = current_time
                    self.get_logger().info("State: Check 90 Degrees -> Reset")
                    
        elif self.current_state == self.STATE_RESET:
            if current_time - self.last_state_change_time > 5:
                # If no fire found after full scan, reset or handle accordingly
                self.get_logger().warn("No fire detected in full scan")
                self.goal_active = False
                self.get_logger().info("Fire detection state machine disabled")

    def check_fire_detection(self, line):
        """
        Check if fire was detected during the last scan
        Uses the most recently received fire detection data
        """
        # Return the most recent fire detection state
        if '1' in line:
            self.get_logger().info("Fire detected!")
            return True
        elif '0' in line:
            self.get_logger().info("No fire detected")
            return False
        else:
            self.get_logger().warn(f"Unexpected fire detection value: {line}")
            return False

    def send_rotation_goal(self, angle):
        """Send a goal pose to rotate to a specific angle"""
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = 'base_link'
        
        # Convert angle to quaternion using our custom function
        angle_rad = angle * (math.pi / 180.0)  # Convert degrees to radians
        q = quaternion_from_euler(0, 0, angle_rad)
        
        goal_msg.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        self.get_logger().info(f"Sending rotation goal to {angle} degrees")
        self.goal_pub.publish(goal_msg)

    def publish_fire_location(self, angle):
        """Publish the final goal pose when fire is detected"""
        if angle == 0:
            goal_msg = self.final_destinations[0]
        elif angle == 45:
            goal_msg = self.final_destinations[1]
        elif angle == 90:
            goal_msg = self.final_destinations[2]
        else:
            self.get_logger().error(f"Invalid angle for goal pose: {angle}")
            return

        self.get_logger().info(f"Fire detected at {angle} degrees! Publishing goal.")
        self.goal_pub.publish(goal_msg)
  
        # Stop further fire detection
        self.goal_active = False

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