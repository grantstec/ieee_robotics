import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Directories
    nav2_bringup_share = get_package_share_directory('nav2_bringup')
    slam_toolbox_share = get_package_share_directory('slam_toolbox')
    ieee_share = get_package_share_directory('ieee_robotics')
    
    # Paths to launch files and parameter files
    nav2_launch_file = os.path.join(nav2_bringup_share, 'launch', 'bringup_launch.py')
    nav2_params_path = os.path.join(ieee_share, 'config', 'nav2_params.yaml')
    
    # SLAM Toolbox parameters
    slam_params = {
        'use_sim_time': False,
        'base_frame': 'base_link',
        'odom_frame': 'odom',
        'map_frame': 'map',
        'resolution': 0.05,
        'max_laser_range': 10.0,
        'transform_publish_period': 0.02,
    }
    
    return LaunchDescription([
        # Launch SLAM Toolbox node (async mode)
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_params]
        ),
        # Launch RPLIDAR node
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
        ),
        # Publish a static transform from base_link to base_footprint
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_base_footprint',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint']
        ),
        # Include Nav2 bringup launch file; pass our nav2 params file and an empty map
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch_file),
            launch_arguments={
                'params_file': nav2_params_path,
                'map': ''  # No static map since SLAM provides one dynamically
            }.items()
        ),
        # Optional: Delay initial pose setting to ensure TF is stable
        ExecuteProcess(
            cmd=['sleep', '10'],
            name='initial_pose_delay'
        )
    ])
