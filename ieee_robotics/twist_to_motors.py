#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TwistToMotors(Node):
    def __init__(self):
        super().__init__('twist_to_motors')
        self.sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.twist_callback,
            10
        )
        self.wheel_pub = self.create_publisher(Twist, 'wheel_commands', 10)
        
        # Robot parameters (adjust these)
        self.wheel_radius = 0.045  # meters
        self.wheel_base = 0.21        # meters

    def twist_callback(self, msg):
        # Differential drive conversion
        left = (msg.linear.x + (msg.angular.z * self.wheel_base / 2)) / self.wheel_radius
        right = (msg.linear.x - (msg.angular.z * self.wheel_base / 2)) / self.wheel_radius
        
        cmd = Twist()
        cmd.linear.x = left
        cmd.linear.y = right
        self.wheel_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = TwistToMotors()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()