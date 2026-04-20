#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

class KalmanFilter(Node):
    def __init__(self):
        super().__init__('kalman_filter')
        self.get_logger().info('Kalman Filter node has been started.')

        self.odom_subscriber = self.create_subscription(Odometry, 'jennybot_controller/odom_noisy', self.odom_callback, 10)
        self.imu_subscriber = self.create_subscription(Imu, 'imu/out', self.imu_callback, 10)
        self.odom_publisher = self.create_publisher(Odometry, 'jennybot_controller/odom_kalman', 10)

        self.mean_ = 0.0
        self.variance_ = 1000.0

        self.imu_angular_z = 0.0
        self.is_first_odom = True
        self.last_angular_z = 0.0

        self.motion = 0.0
        self.kalman_odom = Odometry()

        self.motion_variance = 4.0
        self.measurement_variance = 0.5

    def measurementUpdate(self):
        self.mean_ = (self.measurement_variance * self.mean_ + self.variance_ * self.imu_angular_z)/(self.variance_ + self.measurement_variance)
        self.variance_ = (self.variance_ * self.measurement_variance)/(self.variance_ + self.measurement_variance)

    def statePrediction(self):
        self.mean_ = self.mean_ + self.motion
        self.variance_ = self.variance_ + self.motion_variance

    def imu_callback(self, msg: Imu):
        self.imu_angular_z = msg.angular_velocity.z

    def odom_callback(self, odom: Odometry):
        self.kalman_odom = odom
        if self.is_first_odom:
            self.mean_ = odom.twist.twist.angular.z
            self.last_angular_z = odom.twist.twist.angular.z
            self.is_first_odom = False

            return

        self.motion = odom.twist.twist.angular.z - self.last_angular_z
        self.statePrediction()

        self.measurementUpdate()

        self.kalman_odom.twist.twist.angular.z = self.mean_
        self.odom_publisher.publish(self.kalman_odom)


def main(args=None):
    rclpy.init(args=args)
    kalman_filter = KalmanFilter()
    rclpy.spin(kalman_filter)
    kalman_filter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()