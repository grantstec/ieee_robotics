from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="nav2_bringup",
            executable="nav2_bringup",
            name="nav2",
            output="screen",
            parameters=["config/nav2_params.yaml"]
        ),
        Node(
            package="rplidar_ros",
            executable="rplidar_composition",
            name="lidar_node",
            output="screen"
        ),
        Node(
            package="imu_filter_madgwick",
            executable="imu_filter_node",
            name="imu_node",
            output="screen"
        )
    ])
