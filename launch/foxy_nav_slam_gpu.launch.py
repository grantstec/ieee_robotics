import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directories
    ieee_robotics_share = get_package_share_directory('ieee_robotics')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')
    
    # URDF file
    urdf_file = os.path.join(ieee_robotics_share, 'urdf', 'robot.urdf')
    
    # Config files
    ekf_config_file = os.path.join(ieee_robotics_share, 'config', 'ekf.yaml')
    
    # Use Foxy-specific Nav2 parameters file
    nav2_params_file = os.path.join(ieee_robotics_share, 'config', 'foxy_nav2_params.yaml')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    autostart = LaunchConfiguration('autostart', default='true')
    use_gpu = LaunchConfiguration('use_gpu', default='true')
    
    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('use_gpu', default_value='true'),
        
        # Xavier NX Performance Mode
        ExecuteProcess(
            cmd=['bash', '-c', 'sudo nvpmodel -m 8 && sudo jetson_clocks && sudo jetson_clocks --fan'],
            output='screen'
        ),
        
        # Core Robot Components
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': ParameterValue(Command(['xacro ', urdf_file]), value_type=str)
            }]
        ),
        
        # Hardware Interface Nodes
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
        
        Node(
            package='ieee_robotics',
            executable='wheel_odometry',
            name='wheel_odometry'
        ),
        
        # EKF
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter',
            output='screen',
            parameters=[ekf_config_file]
        ),
        
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher'
        ),
        
        Node(
            package='ieee_robotics',
            executable='twist_to_motors',
            name='twist_to_motors'
        ),
        
        # SLAM - RPLidar
        Node(
            package='rplidar_ros',
            executable='rplidar_composition',  # Different in Foxy!
            name='rplidar',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': 115200,
                'frame_id': 'lidar_link',
                'angle_compensate': True,
                'scan_mode': 'Boost'  # Foxy may use 'Standard' instead of 'Boost'
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
                'transform_publish_period': 0.02,
                # Performance settings for Xavier
                'threads': 4,  # Xavier NX has 6 cores, leave 2 for other processes
                'use_multithread': True,
                'ceres_linear_solver': 'SPARSE_NORMAL_CHOLESKY', 
                'ceres_preconditioner': 'SCHUR_JACOBI',
                'solver_plugin': 'solver_plugins::CeresSolver'
            }]
        ),
        
        # GPU-Accelerated Processing
        Node(
            package='ieee_robotics',
            executable='gpu_image_processor',
            name='gpu_image_processor',
            parameters=[{'use_gpu': use_gpu}],
            output='screen'
        ),
        
        # Navigation - with delay to allow SLAM to initialize
        TimerAction(
            period=5.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(os.path.join(
                        nav2_bringup_share, 'launch', 'navigation_launch.py'
                    )),
                    launch_arguments={
                        'use_sim_time': use_sim_time,
                        'params_file': nav2_params_file,
                        'autostart': autostart,
                        # Foxy-specific options
                        'use_lifecycle_mgr': 'true',
                        'map_subscribe_transient_local': 'true'
                    }.items()
                ),
            ]
        ),
        
        # RViz visualization
        TimerAction(
            period=7.0,
            actions=[
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