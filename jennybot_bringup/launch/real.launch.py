import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    hardware_interface = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("jennybot_firmware"),
            "launch",
            "hardware_interface.launch.py"
        ),
    )
    
    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("jennybot_controller"),
            "launch",
            "controller.launch.py"
        ),
        launch_arguments={
            "use_simple_controller": "False",
        }.items(),
    )

    udp_joystick = Node(
        package='jennybot_controller',
        executable='udp_joystick_teleop.py',
        name='udp_joystick_teleop',
        output='screen',
        parameters=[{
            'udp_port': 4210,
            'max_linear_vel': 0.8,   # Increased for better speed
            'max_angular_vel': 4.0,  # Increased for faster turning
            'timeout': 0.5,
            'deadzone': 0.50,  # Large deadzone to handle calibration offset
        }]
    )

    # imu_driver_node = Node(
    #     package="bumperbot_firmware",
    #     executable="mpu6050_driver.py"
    # )
    
    return LaunchDescription([
        hardware_interface,
        controller,
        # udp_joystick,
        # joystick,
        # imu_driver_node,
    ])