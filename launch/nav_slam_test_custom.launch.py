import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directories
    ieee_robotics_share = get_package_share_directory('ieee_robotics')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')
    
    # URDF file
    urdf_file = os.path.join(
        ieee_robotics_share,
        'urdf',
        'robot.urdf'
    )
    
    # Config files
    ekf_config_file = os.path.join(
        ieee_robotics_share,
        'config',
        'ekf.yaml'
    )
    
    # Use custom Nav2 parameters file
    nav2_params_file = os.path.join(
        ieee_robotics_share,
        'config',
        'custom_nav2_params.yaml'
    )
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    autostart = LaunchConfiguration('autostart', default='true')
    
    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if true'
        ),
        
        DeclareLaunchArgument(
            'autostart', 
            default_value='true',
            description='Automatically start up the nav2 stack'
        ),
        
        # 1. CORE ROBOT DRIVERS - From drive.launch.py
        
        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': ParameterValue(Command(['xacro ', urdf_file]), value_type=str)
            }]
        ),
        
        # Arduino bridge for motor control
        Node(
            package='ieee_robotics',
            executable='arduino_bridge',
            name='arduino_bridge',
            parameters=[{'serial_port': '/dev/ttyACM1'}]
        ),
        
        # Teensy bridge for IMU
        Node(
            package='ieee_robotics',
            executable='teensy_bridge',
            name='teensy_bridge',
            parameters=[{'serial_port': '/dev/ttyACM0'}]
        ),
        
        # Wheel odometry
        Node(
            package='ieee_robotics',
            executable='wheel_odometry',
            name='wheel_odometry'
        ),
        
        # EKF for sensor fusion
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter',
            output='screen',
            parameters=[ekf_config_file]
        ),
        
        # Joint state publisher
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher'
        ),
        
        # Twist to motors converter
        Node(
            package='ieee_robotics',
            executable='twist_to_motors',
            name='twist_to_motors'
        ),
        
        # 2. SLAM - From slam.launch.py
        
        # RPLidar
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
                'resolution': 0.05,
                'max_laser_range': 10.0,
                'transform_publish_period': 0.02
            }]
        ),
        
        # 3. NAVIGATION - with a delay to allow SLAM to initialize
        TimerAction(
            period=5.0,  # 5-second delay
            actions=[
                # Include Nav2 bringup with custom parameters
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(os.path.join(
                        nav2_bringup_share, 'launch', 'navigation_launch.py'
                    )),
                    launch_arguments={
                        'use_sim_time': use_sim_time,
                        'params_file': nav2_params_file,
                        'autostart': autostart
                    }.items()
                ),
            ]
        ),
        
        # 4. VISUALIZATION - with a delay to allow Nav2 to initialize
        TimerAction(
            period=7.0,  # 7-second delay (after SLAM and Nav2 have started)
            actions=[
                # RViz2
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    arguments=['-d', os.path.join(nav2_bringup_share, 'rviz', 'nav2_default_view.rviz')],
                    output='screen'
                ),
            ]
        ),
    ])