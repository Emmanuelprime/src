#!/usr/bin/env python3
"""
Launch file for YDLidar Prime Node
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    
    # Declare launch arguments
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyUSB1',
        description='Serial port for YDLidar'
    )
    
    baudrate_arg = DeclareLaunchArgument(
        'baudrate',
        default_value='115200',
        description='Baudrate for serial communication'
    )
    
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='laser_frame',
        description='Frame ID for laser scan messages'
    )
    
    scan_frequency_arg = DeclareLaunchArgument(
        'scan_frequency',
        default_value='10.0',
        description='Scan frequency in Hz'
    )
    
    sample_rate_arg = DeclareLaunchArgument(
        'sample_rate',
        default_value='3',
        description='Sample rate (3, 5, or 9)'
    )
    
    # YDLidar node
    ydlidar_node = Node(
        package='ydlidar_prime',
        executable='ydlidar_node',
        name='ydlidar_prime_node',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'baudrate': LaunchConfiguration('baudrate'),
            'frame_id': LaunchConfiguration('frame_id'),
            'scan_frequency': LaunchConfiguration('scan_frequency'),
            'sample_rate': LaunchConfiguration('sample_rate'),
            'min_angle': -180.0,
            'max_angle': 180.0,
            'min_range': 0.08,
            'max_range': 16.0,
            'single_channel': True,
        }]
    )
    
    # Static transform publisher (base_link to laser_frame)
    tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser_tf',
        arguments=['0', '0', '0.02', '0', '0', '0', 'base_link', 'laser_frame']
    )
    
    return LaunchDescription([
        port_arg,
        baudrate_arg,
        frame_id_arg,
        scan_frequency_arg,
        sample_rate_arg,
        ydlidar_node,
        tf_node,
    ])
