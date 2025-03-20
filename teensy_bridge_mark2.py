import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped, Quaternion
from tf_transformations import quaternion_from_euler
from std_msgs.msg import Bool
import serial
import time
import math

class TeensyBridge(Node):
    (STATE_INITIAL, STATE_CHECK_0_DEGREES, STATE_CHECK_45_DEGREES_LEFT, STATE_CHECK_90_DEGREES, STATE_RESET) = range(5)

    def __init__(self):
        super().__init__('teensy_bridge')
        
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('timeout', 1.0)
        
        self.goal_active = self.declare_parameter('goal_active', True).value
        self.current_state = self.STATE_INITIAL
        self.last_state_change_time = 0
        self.fire_locations = []
        
        self.goal_active_sub = self.create_subscription(Bool, 'teensy/goal_active', self.goal_active_callback, 10)
        
        self.final_destinations = [self.create_goal_pose(2.0, 0.0, 0),
                                   self.create_goal_pose(2.0, 2.0, 45),
                                   self.create_goal_pose(0.0, 2.0, 90)]
        
        self.current_fire_detection = None
        
        self.imu_pub = self.create_publisher(Imu, 'imu/data_raw', 10)
        self.goal_pub = self.create_publisher(PoseStamped, 'goal_pose', 10)
        
        self.connect_serial()
        self.create_timer(0.01, self.process_serial_and_state_machine)

    def goal_active_callback(self, msg):
        self.goal_active = msg.data
        self.get_logger().info(f"Fire detection state machine: {'Enabled' if self.goal_active else 'Disabled'}")

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
            self.serial = None

    def create_goal_pose(self, x, y, angle):
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, math.radians(angle)))
        return goal

    def process_serial_and_state_machine(self):
        if self.serial is None:
            self.connect_serial()
            return
        
        try:
            self.current_fire_detection = None
            
            if self.serial.in_waiting > 0:
                line = self.serial.readline().decode('utf-8').strip()
                
                if line.startswith('IMU:'):
                    self.process_imu_data(line)
                elif line.startswith('FIRE:'):
                    self.current_fire_detection = '1' in line
                    self.get_logger().info(f"Fire detection: {self.current_fire_detection}")
                    
            if self.goal_active and self.current_fire_detection is not None:
                self.fire_detection_state_machine()
        except Exception as e:
            self.get_logger().error(f"Serial processing error: {str(e)}")
            self.serial = None

    def process_imu_data(self, line):
        try:
            _, data = line.split(':')
            values = list(map(float, data.split(',')))
            if len(values) != 10:
                return
            
            msg = Imu()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'imu_link'
            msg.orientation = Quaternion(*values[:4])
            msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = values[4:7]
            msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = values[7:]
            self.imu_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"IMU data processing error: {str(e)}")

    def fire_detection_state_machine(self):
        current_time = time.time()
        
        if self.current_state == self.STATE_INITIAL:
            self.send_rotation_goal(0)
            self.current_state = self.STATE_CHECK_0_DEGREES
            self.last_state_change_time = current_time

        elif self.current_state == self.STATE_CHECK_0_DEGREES:
            if current_time - self.last_state_change_time > 5:
                if self.current_fire_detection:
                    self.publish_fire_location(0)
                else:
                    self.send_rotation_goal(45)
                    self.current_state = self.STATE_CHECK_45_DEGREES_LEFT
                    self.last_state_change_time = current_time
        
        elif self.current_state == self.STATE_CHECK_45_DEGREES_LEFT:
            if current_time - self.last_state_change_time > 5:
                if self.current_fire_detection:
                    self.publish_fire_location(45)
                else:
                    self.send_rotation_goal(90)
                    self.current_state = self.STATE_CHECK_90_DEGREES
                    self.last_state_change_time = current_time

        elif self.current_state == self.STATE_CHECK_90_DEGREES:
            if current_time - self.last_state_change_time > 5:
                if self.current_fire_detection:
                    self.publish_fire_location(90)
                else:
                    self.send_rotation_goal(0)
                    self.current_state = self.STATE_RESET
                    self.last_state_change_time = current_time

        elif self.current_state == self.STATE_RESET:
            if current_time - self.last_state_change_time > 5:
                self.get_logger().warn("No fire detected in full scan")
                self.goal_active = False

    def send_rotation_goal(self, angle):
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = 'base_link'
        goal_msg.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, math.radians(angle)))
        self.goal_pub.publish(goal_msg)

    def publish_fire_location(self, angle):
        goal_msg = self.final_destinations[{0:0, 45:1, 90:2}.get(angle, 0)]
        self.goal_pub.publish(goal_msg)
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
