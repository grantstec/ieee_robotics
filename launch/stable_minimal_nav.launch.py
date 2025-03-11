#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import TimerAction, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ieee_robotics_share = get_package_share_directory('ieee_robotics')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')
    
    # Find the correct path to the BT XML file for Foxy
    nav2_bt_navigator_dir = get_package_share_directory('nav2_bt_navigator')
    bt_xml_path = os.path.join(nav2_bt_navigator_dir, 'behavior_trees', 'navigate_w_replanning_and_recovery.xml')
    
    # Read URDF file directly for robot_state_publisher
    urdf_file = os.path.join(ieee_robotics_share, 'urdf', 'robot.urdf')
    with open(urdf_file, 'r') as f:
        robot_description = f.read()
    
    # Nav2 parameters file
    nav2_params_file = os.path.join(ieee_robotics_share, 'config', 'stable_nav_params.yaml')
    
    return LaunchDescription([
        # STAGE 1: Core robot state and hardware interfaces
        # =================================================
        
        # Robot state publisher with direct URDF content
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'publish_frequency': 100.0,  # High frequency publishing
                'use_tf_static': True,
                'ignore_timestamp': True
            }]
        ),
        
        # Arduino bridge for motors
        Node(
            package='ieee_robotics',
            executable='arduino_bridge',
            name='arduino_bridge',
            output='log',
            parameters=[{
                'port': '/dev/ttyACM1',
                'debug_output': False  # Reduce logging noise
            }]
        ),
        
        # Teensy bridge for IMU
        Node(
            package='ieee_robotics',
            executable='teensy_bridge',
            name='teensy_bridge',
            output='log',
            parameters=[{
                'port': '/dev/ttyACM0'
            }]
        ),
        
        # Joint state publisher
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='log'
        ),
        
        # Wheel odometry that publishes directly to odom
        Node(
            package='ieee_robotics',
            executable='wheel_odometry',
            name='wheel_odometry',
            output='screen',
            parameters=[{
                'wheel_radius': 0.045,
                'base_width': 0.21,
                'steps_per_rev': 3200
            }],
            remappings=[
                ('wheel_odom', 'odom')  # Direct mapping to odom topic
            ]
        ),
        
        # Twist to motors control
        Node(
            package='ieee_robotics',
            executable='twist_to_motors',
            name='twist_to_motors',
            output='log',
            parameters=[{
                'wheel_radius': 0.045,
                'wheel_base': 0.21
            }]
        ),
        
        # STAGE 2: Start LIDAR after a delay
        # ==================================
        TimerAction(
            period=5.0,  # 5-second delay for TF tree to establish
            actions=[
                # RPLidar with Standard mode
                Node(
                    package='rplidar_ros',
                    executable='rplidar_composition',
                    name='rplidar',
                    output='screen',
                    parameters=[{
                        'serial_port': '/dev/ttyUSB0',
                        'serial_baudrate': 115200,
                        'frame_id': 'lidar_link',  # Must match URDF
                        'angle_compensate': True,
                        'scan_mode': 'Standard',  # Standard mode is more reliable
                        'scan_frequency': 5.0     # Lower frequency for stability
                    }]
                )
            ]
        ),
        
        # STAGE 3: Start SLAM after LIDAR is initialized
        # =============================================
        TimerAction(
            period=10.0,  # 10-second delay for LIDAR to be publishing
            actions=[
                # SLAM Toolbox with minimal parameters
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
                        'max_laser_range': 5.0,
                        'transform_timeout': 1.0,
                        'tf_buffer_duration': 30.0,
                        'transform_publish_period': 0.1,
                        'use_multithread': False
                    }]
                )
            ]
        ),
        
        # STAGE 4: Start Nav2 components after SLAM is running
        # ==================================================
        TimerAction(
            period=20.0,  # 20-second delay for SLAM to fully initialize
            actions=[
                # Lifecycle manager to control all Nav2 components
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
                
                # Controller Server (local planner)
                Node(
                    package='nav2_controller',
                    executable='controller_server',
                    name='controller_server',
                    output='screen',
                    parameters=[nav2_params_file]
                ),
                
                # Planner Server (global planner)
                Node(
                    package='nav2_planner',
                    executable='planner_server',
                    name='planner_server',
                    output='screen',
                    parameters=[nav2_params_file]
                ),
                
                # Recovery behaviors
                Node(
                    package='nav2_recoveries',
                    executable='recoveries_server',
                    name='recoveries_server',
                    output='log',
                    parameters=[nav2_params_file]
                ),
                
                # BT Navigator with explicit path to BT file
                Node(
                    package='nav2_bt_navigator',
                    executable='bt_navigator',
                    name='bt_navigator',
                    output='screen',
                    parameters=[
                        nav2_params_file,
                        {'default_bt_xml_filename': bt_xml_path}  # Explicit path
                    ]
                )
            ]
        ),
        
        # STAGE 5: RViz2 for visualization - start last
        # ===========================================
        TimerAction(
            period=25.0,  # 25-second delay for everything else to be ready
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    arguments=['-d', os.path.join(nav2_bringup_share, 'rviz', 'nav2_default_view.rviz')],
                    output='log'
                )
            ]
        )
    ])