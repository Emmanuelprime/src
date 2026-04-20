#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import numpy as np
from sensor_msgs.msg import JointState
from rclpy.time import Time
from rclpy.constants import S_TO_NS
from nav_msgs.msg import Odometry
from tf_transformations import quaternion_from_euler
from tf2_ros import TransformBroadcaster

class JennyBotNoisySimpleController(Node):
    def __init__(self):
        super().__init__("noisy_simple_controller")

        self.declare_parameter("wheel_radius", 0.042)
        self.declare_parameter("wheel_separation", 0.315)

        self.wheel_radius = self.get_parameter("wheel_radius").get_parameter_value().double_value
        self.wheel_separation = self.get_parameter("wheel_separation").get_parameter_value().double_value

        self.get_logger().info(f"Using wheel radius: {self.wheel_radius} m")
        self.get_logger().info(f"Using wheel separation: {self.wheel_separation} m")

        self.left_wheel_prev_position_ = 0.0
        self.right_wheel_prev_position_ = 0.0
        self.prev_time_ = self.get_clock().now()

        self.x_ = 0.0
        self.y_ = 0.0
        self.theta_ = 0.0



        self.joint_sub_ = self.create_subscription(JointState, "joint_states", self.joint_state_callback, 10)
        self.odom_publisher_ = self.create_publisher(Odometry, "jennybot_controller/odom_noisy", 10)

        
        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = "odom"
        self.odom_msg.child_frame_id = " base_footprint_ekf"
        self.odom_msg.pose.pose.orientation.x = 0.0
        self.odom_msg.pose.pose.orientation.y = 0.0
        self.odom_msg.pose.pose.orientation.z = 0.0
        self.odom_msg.pose.pose.orientation.w = 1.0

        self.broadcaster = TransformBroadcaster(self)
        self.transform_stamped = TransformStamped()
        self.transform_stamped.header.frame_id = "odom"
        self.transform_stamped.child_frame_id = "base_footprint_noisy"



    def joint_state_callback(self, msg: JointState):
        wheel_encoder_left = msg.position[1] + np.random.normal(0, 0.005)
        wheel_encoder_right = msg.position[0] + np.random.normal(0, 0.005)
        dp_left = wheel_encoder_left - self.left_wheel_prev_position_
        dp_right = wheel_encoder_right - self.right_wheel_prev_position_
        dt = Time.from_msg(msg.header.stamp) - self.prev_time_

        self.left_wheel_prev_position_ = wheel_encoder_left
        self.right_wheel_prev_position_ = wheel_encoder_right
        self.prev_time_ = Time.from_msg(msg.header.stamp)

        left_phi = dp_left/(dt.nanoseconds / S_TO_NS)
        right_phi = dp_right/(dt.nanoseconds / S_TO_NS)

        linear = self.wheel_radius/2 * (left_phi + right_phi)
        angular = self.wheel_radius/self.wheel_separation * (right_phi - left_phi)
        
        d_s = self.wheel_radius/2 * (dp_left + dp_right)
        d_theta = self.wheel_radius/self.wheel_separation * (dp_right - dp_left)
        self.theta_ += d_theta
        self.x_ += d_s * np.cos(self.theta_)
        self.y_ += d_s * np.sin(self.theta_)

        q = quaternion_from_euler(0, 0, self.theta_)
        self.odom_msg.pose.pose.orientation.x = q[0]
        self.odom_msg.pose.pose.orientation.y = q[1]
        self.odom_msg.pose.pose.orientation.z = q[2]
        self.odom_msg.pose.pose.orientation.w = q[3]
        self.odom_msg.header.stamp = self.get_clock().now().to_msg()
        self.odom_msg.pose.pose.position.x = self.x_
        self.odom_msg.pose.pose.position.y = self.y_
        self.odom_msg.twist.twist.linear.x = linear
        self.odom_msg.twist.twist.angular.z = angular


        self.transform_stamped.transform.translation.x = self.x_
        self.transform_stamped.transform.translation.y = self.y_
        self.transform_stamped.transform.rotation.x = q[0]
        self.transform_stamped.transform.rotation.y = q[1]
        self.transform_stamped.transform.rotation.z = q[2]
        self.transform_stamped.transform.rotation.w = q[3]
        self.transform_stamped.header.stamp = self.get_clock().now().to_msg()


        self.odom_publisher_.publish(self.odom_msg)
        self.broadcaster.sendTransform(self.transform_stamped)

        # self.get_logger().info(f"linear velocity: {linear:.3f} m/s, angular velocity: {angular:.3f} rad/s")
        # self.get_logger().info(f"x={self.x_:.3f} m, y={self.y_:.3f} m, theta={self.theta_:.3f} rad")

        

def main(args=None):
    rclpy.init(args=args)
    noisy_controller_node = JennyBotNoisySimpleController()
    rclpy.spin(noisy_controller_node)
    noisy_controller_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()