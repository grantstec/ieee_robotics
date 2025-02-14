from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="ros2_control",
            executable="controller_manager",
            name="motor_controller",
            parameters=["config/drive_controller.yaml"]
        )
    ])
