import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ieee_robotics_share = get_package_share_directory('ieee_robotics')
    
    # URDF file
    urdf_file = os.path.join(ieee_robotics_share, 'urdf', 'robot.urdf')
    
    return LaunchDescription([
        # Performance mode for Xavier
        ExecuteProcess(
            cmd=['bash', '-c', 'sudo nvpmodel -m 0 && sudo jetson_clocks'],
            output='screen'
        ),
        
        # 1. Most basic components only
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': open(urdf_file, 'r').read(),
                'publish_frequency': 50.0,
                'ignore_timestamp': True,
                'use_tf_static': True
            }]
        ),
        
        # 2. Hardware interfaces with minimal parameters
        Node(
            package='ieee_robotics',
            executable='arduino_bridge',
            name='arduino_bridge',
            parameters=[{'port': '/dev/ttyACM1'}]
        ),
        
        Node(
            package='ieee_robotics',
            executable='teensy_bridge',
            name='teensy_bridge',
            parameters=[{'port': '/dev/ttyACM0'}]
        ),
        
        # 3. Joint publisher
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher'
        ),
        
        # 4. CRITICAL CHANGE: Don't use wheel_odometry and EKF at the same time
        # Instead, use *either* wheel_odometry *or* odometry from the IMU
        # Let's go with wheel_odometry only for simplicity
        Node(
            package='ieee_robotics',
            executable='wheel_odometry',
            name='wheel_odometry',
            parameters=[{
                'publish_tf': True,  # Make wheel_odometry publish the TF
                'wheel_radius': 0.045,
                'base_width': 0.21
            }],
            remappings=[
                ('wheel_odom', 'odom')  # Map wheel_odom to odom for SLAM
            ]
        ),
        
        # 5. Twist to motors for control
        Node(
            package='ieee_robotics',
            executable='twist_to_motors',
            name='twist_to_motors'
        ),
        
        # 6. LIDAR setup with fixed parameters 
        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': 115200,
                'frame_id': 'lidar_link',
                'angle_compensate': True,
                'scan_mode': 'Standard',  # Use Standard instead of Boost for stability
                'scan_frequency': 5.0     # Lower frequency for stability
            }]
        ),
        
        # 7. Simplified SLAM with conservative parameters
        # Wait 5 seconds before starting SLAM to ensure hardware is ready
        TimerAction(
            period=5.0,
            actions=[
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
                        'transform_publish_period': 0.1,      # Slower publication
                        'tf_buffer_duration': 30.0,
                        'map_update_interval': 10.0,          # Much slower updates
                        'use_multithread': False,             # Disable multithreading
                        'debug_logging': True,
                        'throttle_scans': 2,                  # Process every other scan
                        'solver_plugin': 'solver_plugins::CeresSolver',
                        'ceres_linear_solver': 'SPARSE_NORMAL_CHOLESKY',
                        'ceres_preconditioner': 'SCHUR_JACOBI',
                        'enable_slam_toolbox_debug': True     # Enable debug visualization
                    }]
                )
            ]
        ),
        
        # 8. RViz visualization after SLAM is properly initialized
        # TimerAction(
        #     period=15.0,
        #     actions=[
        #         Node(
        #             package='rviz2',
        #             executable='rviz2',
        #             name='rviz2',
        #             arguments=[
        #                 '-d', os.path.join(get_package_share_directory('nav2_bringup'), 
        #                                 'rviz', 'nav2_default_view.rviz')
        #             ],
        #             output='screen'
        #         ),
        #     ]
        # ),


    # Add after SLAM is working well (update the launch file)
        TimerAction(
            period=20.0,  # Longer delay after SLAM
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(os.path.join(
                        get_package_share_directory('nav2_bringup'), 
                        'launch', 'navigation_launch.py'
                    )),
                    launch_arguments={
                        'use_sim_time': 'false',
                        'params_file': os.path.join(ieee_robotics_share, 'config', 'foxy_nav2_params.yaml'),
                        'autostart': 'true',
                        'map_subscribe_transient_local': 'true'
                    }.items()
                ),
            ]
        ),
    ])