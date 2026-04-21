#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial


class SimpleSerialTransmiter(Node):
    def __init__(self):
        super().__init__("simple_serial_transmiter")
        self.get_logger().info("Starting simple serial transmiter node")

        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 115200)

        self.port = self.get_parameter("port").value
        self.baudrate = self.get_parameter("baudrate").value

        self.arduino_ = serial.Serial(self.port, self.baudrate, timeout=1)
        self.get_logger().info("Serial connection established")

        self.sub_ = self.create_subscription(String,"serial_transmitter", self.serial_transmitter_callback, 10)

    def serial_transmitter_callback(self, msg: String):
        self.get_logger().info(f"Received message: {msg.data}")
        try:
            self.arduino_.write(msg.data.encode())
            self.get_logger().info("Message sent to Arduino")
        except Exception as e:
            self.get_logger().error(f"Failed to send message to Arduino: {e}")




def main(args=None):
    rclpy.init(args=args)
    node = SimpleSerialTransmiter()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()