#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TwistStamped
import numpy as np

class JennyBotSimpleController(Node):
    def __init__(self):
        super().__init__("simple_controller")

        self.declare_parameter("wheel_radius", 0.042)
        self.declare_parameter("wheel_separation", 0.312)

        self.wheel_radius = self.get_parameter("wheel_radius").get_parameter_value().double_value
        self.wheel_separation = self.get_parameter("wheel_separation").get_parameter_value().double_value

        self.get_logger().info(f"Using wheel radius: {self.wheel_radius} m")
        self.get_logger().info(f"Using wheel separation: {self.wheel_separation} m")


        self.wheel_cmd_pub_ = self.create_publisher(Float64MultiArray, "simple_velocity_controller/commands", 10)
        self.cmd_vel_sub_ = self.create_subscription(TwistStamped, "jennybot_controller/cmd_vel", self.cmd_vel_callback, 10)

        self.speed_conversion_ = np.array([[self.wheel_radius/2, self.wheel_radius/2],
                                            [self.wheel_radius/self.wheel_separation, -self.wheel_radius/self.wheel_separation]])
        self.get_logger().info(f"Speed conversion matrix:\n{self.speed_conversion_}")


    def cmd_vel_callback(self, msg: TwistStamped):
        robot_speed = np.array([[msg.twist.linear.x],
                                [msg.twist.angular.z]])
        
        wheel_speed = np.matmul(np.linalg.inv(self.speed_conversion_),robot_speed)

        wheel_speed_mg = Float64MultiArray()
        wheel_speed_mg.data = [wheel_speed[1,0], wheel_speed[0,0]]
        self.wheel_cmd_pub_.publish(wheel_speed_mg)
    

def main(args=None):
    rclpy.init(args=args)
    controller_node = JennyBotSimpleController()
    rclpy.spin(controller_node)
    controller_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()