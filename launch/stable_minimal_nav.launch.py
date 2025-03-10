import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, get_package_prefix

def generate_launch_description():
    ieee_robotics_share = get_package_share_directory('ieee_robotics')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')
    
    # Find the correct path to the BT XML files
    nav2_bt_navigator_dir = get_package_share_directory('nav2_bt_navigator')
    bt_xml_path = os.path.join(nav2_bt_navigator_dir, 'behavior_trees', 'navigate_w_replanning_and_recovery.xml')
    
    # URDF file
    urdf_file = os.path.join(ieee_robotics_share, 'urdf', 'robot.urdf')
    
    # Nav2 parameters file - you need to create this file in your config directory
    nav2_params_file = os.path.join(
        ieee_robotics_share,
        'config',
        'slam_navigation.yaml'
    )
    
    return LaunchDescription([
        # Performance mode
        ExecuteProcess(
            cmd=['bash', '-c', 'sudo nvpmodel -m 0 && sudo jetson_clocks'],
            output='screen'
        ),
        
        # 1. Robot hardware stack (minimal)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='log',  # Changed to 'log' to reduce console output
            parameters=[{
                'robot_description': open(urdf_file, 'r').read(),
                'publish_frequency': 50.0,
                'ignore_timestamp': True,
                'use_tf_static': True
            }]
        ),
        
        # Joint state publisher
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='log'  # Changed to 'log'
        ),
        
        # Arduino bridge with super quiet mode
        Node(
            package='ieee_robotics',
            executable='arduino_bridge',
            name='arduino_bridge',
            output='log',  # Changed to 'log'
            parameters=[{
                'port': '/dev/ttyACM1',
                'debug_output': False  # Disable verbose output
            }]
        ),
        
        # Teensy IMU bridge
        Node(
            package='ieee_robotics',
            executable='teensy_bridge',
            name='teensy_bridge',
            output='log',  # Changed to 'log'
            parameters=[{
                'port': '/dev/ttyACM0'
            }]
        ),
        
        # Wheel odometry (publishes TF)
        Node(
            package='ieee_robotics',
            executable='wheel_odometry',
            name='wheel_odometry',
            output='log',  # Changed to 'log'
            parameters=[{
                'publish_tf': True,
                'wheel_radius': 0.045,
                'base_width': 0.21
            }],
            remappings=[
                ('wheel_odom', 'odom')  # Map wheel_odom to odom for SLAM
            ]
        ),

        # Twist to motors
        Node(
            package='ieee_robotics',
            executable='twist_to_motors',
            name='twist_to_motors',
            output='log',  # Changed to 'log'
            parameters=[{
                'wheel_radius': 0.045,
                'wheel_base': 0.21
            }]
        ),
        
        # LIDAR with Standard mode and reduced frequency
        Node(
            package='rplidar_ros',
            executable='rplidar_composition', 
            name='rplidar',
            output='log',  # Changed to 'log'
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': 115200,
                'frame_id': 'lidar_link',  # Must exactly match URDF
                'angle_compensate': True,
                'scan_mode': 'Standard',   # More reliable than Boost
                'scan_frequency': 5.0      # Lower rate = more stable
            }]
        ),
        
        # Wait for TF tree to be established before starting SLAM
        TimerAction(
            period=3.0,  # 3 second delay to ensure TF tree is established
            actions=[
                # SLAM with conservative parameters
                Node(
                    package='slam_toolbox',
                    executable='async_slam_toolbox_node',
                    name='slam_toolbox',
                    output='screen',  # Keep this as 'screen' to see SLAM status
                    parameters=[{
                        'use_sim_time': False,
                        'base_frame': 'base_link',
                        'odom_frame': 'odom',
                        'map_frame': 'map',
                        'resolution': 0.05,
                        'max_laser_range': 5.0,
                        'transform_publish_period': 0.1,
                        'tf_buffer_duration': 30.0,
                        'map_update_interval': 5.0,
                        'use_multithread': False,
                        'debug_logging': False,  # Disable debug logging
                        'throttle_scans': 2,
                        'enable_slam_toolbox_debug': False
                    }]
                )
            ]
        ),
        
        # Wait longer before starting navigation
        TimerAction(
            period=8.0,  # 8 second delay to ensure SLAM is initialized
            actions=[
                # Lifecycle manager with correct node names
                Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager_navigation',
                    output='screen',
                    parameters=[{
                        'use_sim_time': False,
                        'autostart': True,
                        'node_names': [
                            'controller_server',
                            'planner_server',
                            'recoveries_server',
                            'bt_navigator'
                        ]
                    }]
                ),
                
                # Controller Server (Local Planner)
                Node(
                    package='nav2_controller',
                    executable='controller_server',
                    name='controller_server',
                    output='log',  # Changed to 'log'
                    parameters=[nav2_params_file]
                ),
                
                # Planner Server (Global Planner)
                Node(
                    package='nav2_planner',
                    executable='planner_server',
                    name='planner_server',
                    output='log',  # Changed to 'log'
                    parameters=[nav2_params_file]
                ),
                
                # Recovery behaviors
                Node(
                    package='nav2_recoveries',
                    executable='recoveries_server',
                    name='recoveries_server',
                    output='log',  # Changed to 'log'
                    parameters=[nav2_params_file]
                ),
                
                # BT Navigator with EXPLICIT path to the XML file
                Node(
                    package='nav2_bt_navigator',
                    executable='bt_navigator',
                    name='bt_navigator',
                    output='screen',  # Keep this as 'screen' to see nav errors
                    parameters=[
                        nav2_params_file,
                        {'default_bt_xml_filename': bt_xml_path}  # Explicit full path
                    ]
                )
            ]
        )
    ])