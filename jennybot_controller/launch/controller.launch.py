import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    wheel_radius_arg = DeclareLaunchArgument(name="wheel_radius", default_value="0.042",description="Radius of the robot's wheels in meters")
    wheel_separation_arg = DeclareLaunchArgument(name="wheel_separation", default_value="0.312", description="Distance between the robot's wheels in meters")

    wheel_radius = LaunchConfiguration("wheel_radius")
    wheel_separation = LaunchConfiguration("wheel_separation")


    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    simple_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["simple_velocity_controller",
                "--controller-manager",
                "/controller_manager"
        ]
    )

    simple_controller_py = Node(
        package="jennybot_controller",
        executable="jenny_simple_controller.py",
        parameters=[{"wheel_radius": wheel_radius,
                     "wheel_separation": wheel_separation}]
    )

    return LaunchDescription(
        [
            wheel_radius_arg,
            wheel_separation_arg,
            joint_state_broadcaster_spawner,
            simple_controller,
            simple_controller_py,
        ]
    )