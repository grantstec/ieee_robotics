from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Directories for packages and files
    ieee_robotics_share = get_package_share_directory('ieee_robotics')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')
    
    # Path to your nav2 parameters file
    nav2_params_file = os.path.join(ieee_robotics_share, 'config', 'nav2_params.yaml')
    
    # Create the launch description
    ld = LaunchDescription([
        # Include your drive launch file


        # Robot Localization EKF
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter',
            output='screen',
            parameters=[
                os.path.join(ieee_robotics_share, 'config', 'ekf.yaml')
            ]
        ),

        # Initial Pose Publisher (Optional)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='initial_pose_pub',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
        ),

        # Delay Nav2 Bringup until the drive system is ready
        TimerAction(
            period=5.0,  # Adjust as necessary for your system's startup time
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(nav2_bringup_share, 'launch', 'bringup_launch.py')
                    ),
                    launch_arguments={
                        'use_sim_time': 'False',
                        'params_file': nav2_params_file,
                        'map': ''  # Pass an empty string since no static map file exists
                    }.items()
                )
            ]
        ),

        # # Controller Manager Node (add this)
        # Node(
        #     package='controller_manager',
        #     executable='ros2_control_node',
        #     name='controller_manager',
        #     parameters=[os.path.join(ieee_robotics_share, 'config', 'controller_config.yaml')]
        # )
    ])

    return ld