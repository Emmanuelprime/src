#!/usr/bin/env python3
"""
Complete launch file for real robot with SLAM mapping
Launches: Robot control, LIDAR, SLAM Toolbox, and Joystick
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
    # Get package directories
    jennybot_bringup_dir = get_package_share_directory('jennybot_bringup')
    ydlidar_prime_dir = get_package_share_directory('ydlidar_prime')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    jennybot_localization_dir = get_package_share_directory('jennybot_localization')
    
    # Declare arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    mapper_params_file_arg = DeclareLaunchArgument(
        'mapper_params_file',
        default_value=os.path.join(jennybot_localization_dir, 'config', 'mapper_params_online_async.yaml'),
        description='Full path to mapper params file'
    )
    
    # Launch real robot (hardware interface + controllers + joystick)
    real_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(jennybot_bringup_dir, 'launch', 'real.launch.py')
        )
    )
    
    # Launch LIDAR - Delay 5 seconds to wait for hardware interface to initialize
    lidar = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(ydlidar_prime_dir, 'launch', 'ydlidar.launch.py')
                ),
                launch_arguments={
                    'port': '/dev/ttyUSB1',
                    'baudrate': '115200',
                    'frame_id': 'laser_frame',
                }.items()
            )
        ]
    )
    
    # Launch SLAM Toolbox - Delay 10 seconds to wait for LIDAR scans
    slam = TimerAction(
        period=10.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')
                ),
                launch_arguments={
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'params_file': LaunchConfiguration('mapper_params_file'),
                }.items()
            )
        ]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        mapper_params_file_arg,
        real_robot,
        lidar,
        slam,
    ])
