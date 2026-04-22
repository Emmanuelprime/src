#!/usr/bin/env python3
"""
Launch UDP joystick teleoperation node
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='jennybot_controller',
            executable='udp_joystick_teleop.py',
            name='udp_joystick_teleop',
            output='screen',
            parameters=[{
                'udp_port': 4210,
                'max_linear_vel': 1.0,   # m/s
                'max_angular_vel': 5.0,  # rad/s
                'timeout': 0.5,          # seconds
            }]
        )
    ])
