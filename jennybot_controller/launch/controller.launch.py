import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument,OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition

def noisy_controller(context, *args, **kwargs):
    wheel_radius = float(LaunchConfiguration("wheel_radius").perform(context))
    wheel_separation = float(LaunchConfiguration("wheel_separation").perform(context))
    wheel_radius_error = float(LaunchConfiguration("wheel_radius_error").perform(context))
    wheel_separation_error = float(LaunchConfiguration("wheel_separation_error").perform(context))

    noisy_controller_node = Node(
        package="jennybot_controller",
        executable="noisy_jenny_simple_controller.py",
        name="noisy_odometry_controller",
        parameters=[{"wheel_radius": wheel_radius + wheel_radius_error,
                     "wheel_separation": wheel_separation + wheel_separation_error}]
    )

    return [noisy_controller_node]

def generate_launch_description():

    wheel_radius_arg = DeclareLaunchArgument(name="wheel_radius", default_value="0.042",description="Radius of the robot's wheels in meters")
    wheel_separation_arg = DeclareLaunchArgument(name="wheel_separation", default_value="0.312", description="Distance between the robot's wheels in meters")
    use_simple_controller_arg = DeclareLaunchArgument(name="use_simple_controller", default_value="False", description="Whether to use the simple controller or the diff drive controller")

    wheel_radius_error_arg = DeclareLaunchArgument(name="wheel_radius_error", default_value="0.005", description="Standard deviation of the wheel radius noise in meters")
    wheel_separation_error_arg = DeclareLaunchArgument(name="wheel_separation_error", default_value="0.02", description="Standard deviation of the wheel separation noise in meters")

    wheel_radius = LaunchConfiguration("wheel_radius")
    wheel_separation = LaunchConfiguration("wheel_separation")
    wheel_radius_error = LaunchConfiguration("wheel_radius_error")
    wheel_separation_error = LaunchConfiguration("wheel_separation_error")
    use_simple_controller = LaunchConfiguration("use_simple_controller")

    wheel_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["jennybot_controller",
                "--controller-manager",
                "/controller_manager"
        ]
        ,
        condition=UnlessCondition(use_simple_controller)
    )


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
        ],
        condition=IfCondition(use_simple_controller)
    )

    simple_controller_py = Node(
        package="jennybot_controller",
        executable="jenny_simple_controller.py",
        name="clean_odometry_controller",
        parameters=[{"wheel_radius": wheel_radius,
                     "wheel_separation": wheel_separation}],
        condition=IfCondition(use_simple_controller)
    )

    # Removed noisy_controller_launch - it was always running unconditionally
    
    return LaunchDescription(
        [
            wheel_radius_arg,
            wheel_separation_arg,
            wheel_radius_error_arg,
            wheel_separation_error_arg,
            use_simple_controller_arg,
            joint_state_broadcaster_spawner,
            wheel_controller_spawner,
            simple_controller,
            simple_controller_py,
        ]
    )