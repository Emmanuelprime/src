#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class SimpleSerialReceiver(Node):
    def __init__(self):
        super().__init__("simple_serial_receiver")
        self.get_logger().info("Starting simple serial receiver node")

        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 115200)

        self.port = self.get_parameter("port").value
        self.baudrate = self.get_parameter("baudrate").value

        self.arduino_ = serial.Serial(self.port, self.baudrate, timeout=1)

        self.pub_ = self.create_publisher(String, "serial_receiver", 10)
        self.frequency_ = 0.01
        self.timer_ = self.create_timer(self.frequency_, self.timer_callback)

    
    def timer_callback(self):
        if rclpy.ok() and self.arduino_.is_open:
            try:
                line = self.arduino_.readline().decode().strip()
                if line:
                    self.get_logger().info(f"Received message from Arduino: {line}")
                    msg = String()
                    msg.data = line
                    self.pub_.publish(msg)
            except Exception as e:
                self.get_logger().error(f"Failed to read from Arduino: {e}")
        else:
            self.get_logger().error("Serial connection is not open")



def main(args=None):
    rclpy.init(args=args)
    node = SimpleSerialReceiver()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()