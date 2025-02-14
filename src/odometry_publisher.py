import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from tf_transformations import quaternion_from_euler
import serial

class OdometryPublisher(Node):
    def __init__(self):
        super().__init__('odometry_publisher')

        # ROS2 publisher
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # Serial connections
        self.arduino = serial.Serial('/dev/ttyUSB0', 115200)  # Change to correct port
        self.teensy = serial.Serial('/dev/ttyUSB1', 115200)   # Change to correct port

        # Robot parameters
        self.wheel_radius = 0.05  # 5cm radius
        self.wheel_base = 0.3  # 30cm wheel separation
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.create_timer(0.05, self.update_odometry)

    def update_odometry(self):
        if self.arduino.in_waiting > 0:
            line = self.arduino.readline().decode('utf-8').strip()
            left_steps, right_steps = map(float, line.split())

            # Compute linear & angular displacement
            v = (left_steps + right_steps) * self.wheel_radius / 2.0
            w = (right_steps - left_steps) * self.wheel_radius / self.wheel_base

            # Update position
            self.x += v * 0.05 * cos(self.theta)
            self.y += v * 0.05 * sin(self.theta)

        if self.teensy.in_waiting > 0:
            yaw = float(self.teensy.readline().decode('utf-8').strip())
            self.theta = yaw  # Use IMU yaw

        # Publish Odometry
        odom = Odometry()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y

        quat = quaternion_from_euler(0, 0, self.theta)
        odom.pose.pose.orientation = Quaternion(*quat)

        self.odom_pub.publish(odom)

rclpy.init()
node = OdometryPublisher()
rclpy.spin(node)
