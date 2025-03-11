import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction


def generate_launch_description():
    ieee_robotics_share = get_package_share_directory('ieee_robotics')
    
    # Use your existing URDF
    urdf_file = os.path.join(
        ieee_robotics_share,
        'urdf',
        'robot.urdf'
    )
    
    # Use a modified EKF configuration
    ekf_debug_file = os.path.join(
        ieee_robotics_share,
        'config',
        'ekf_debug.yaml'
    )
    
    return LaunchDescription([
        # 1. Setup TF tree first with higher publish rate
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': open(urdf_file, 'r').read(),
                'publish_frequency': 50.0,  # Higher frequency for better TF
                'ignore_timestamp': True,
                'use_tf_static': True
            }]
        ),
        
        # 2. Hardware interfaces
        Node(
            package='ieee_robotics',
            executable='arduino_bridge',
            name='arduino_bridge',
            parameters=[{'port': '/dev/ttyACM1'}]
        ),
        
        
        Node(
            package='ieee_robotics',
            executable='wheel_odometry',
            name='wheel_odometry'
        ),

        Node(
            package='ieee_robotics',
            executable='twist_to_motors',
            name='twist_to_motors',
        ),
        
        # 3. Fixed LIDAR setup with explicit Standard mode
        Node(
            package='rplidar_ros',
            executable='rplidar_composition',  # Foxy uses this executable
            name='rplidar',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': 115200,
                'frame_id': 'lidar_link',  # Must match the URDF
                'angle_compensate': True,
                'scan_mode': 'Boost',    # Force to Standard mode
                'scan_frequency': 10.0      # Set explicit frequency
            }]
        ),
        
        # 4. Simplified SLAM with more debugging output
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
                'resolution': 0.05,
                'max_laser_range': 8.0,
                'transform_publish_period': 0.05,  # Faster transform publishing
                'tf_buffer_duration': 30.0,        # Longer TF buffer
                'debug_logging': True,
                'throttle_scans': 1                # Process every scan
            }]
        ),
        TimerAction(
            period=10.0,
            actions=[
                Node(
                    package='ieee_robotics',
                    executable='teensy_bridge',
                    name='teensy_bridge',
                    output='screen',
                    parameters=[{'port': '/dev/ttyACM0'}]
                )
            ]
        )
        
    ])