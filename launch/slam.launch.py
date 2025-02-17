from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # SLAM Toolbox
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'base_frame': 'base_link',
                'odom_frame': 'odom',
                'map_frame': 'map',
                'resolution': 0.05,  # 10cm grid resolution
                'max_laser_range': 10.0,  # A1M8's max range
                'transform_publish_period': 0.02
            }]
        ),
        
        # RPLIDAR
        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': 115200,
                'frame_id': 'lidar',
                'angle_compensate': True,
                'scan_mode': 'Boost'
            }]
        )
    ])