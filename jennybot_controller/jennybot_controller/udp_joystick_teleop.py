#!/usr/bin/env python3
"""
UDP Joystick Teleoperation Node
Receives joystick data via UDP and publishes cmd_vel commands
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import socket
import json
import select


class UDPJoystickTeleop(Node):
    def __init__(self):
        super().__init__('udp_joystick_teleop')
        
        # Parameters
        self.declare_parameter('udp_port', 4210)
        self.declare_parameter('max_linear_vel', 1.0)  # m/s
        self.declare_parameter('max_angular_vel', 5.0)  # rad/s
        self.declare_parameter('timeout', 0.5)  # seconds - stop if no data
        self.declare_parameter('deadzone', 0.2)  # ignore values below this threshold
        
        self.udp_port = self.get_parameter('udp_port').value
        self.max_linear = self.get_parameter('max_linear_vel').value
        self.max_angular = self.get_parameter('max_angular_vel').value
        self.timeout = self.get_parameter('timeout').value
        self.deadzone = self.get_parameter('deadzone').value
        
        # Publisher
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/jennybot_controller/cmd_vel_unstamped',
            10
        )
        
        # UDP Socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(('', self.udp_port))
        self.sock.setblocking(False)
        
        self.get_logger().info(f'Listening for UDP joystick on port {self.udp_port}')
        self.get_logger().info(f'Max velocities: linear={self.max_linear} m/s, angular={self.max_angular} rad/s')
        
        # Timer to check for UDP data
        self.timer = self.create_timer(0.02, self.timer_callback)  # 50Hz
        
        # Watchdog
        self.last_packet_time = self.get_clock().now()
        
    def timer_callback(self):
        """Check for UDP packets and publish cmd_vel"""
        try:
            # Check if data is available (non-blocking)
            ready = select.select([self.sock], [], [], 0)
            if ready[0]:
                data, addr = self.sock.recvfrom(1024)
                self.process_joystick_data(data.decode('utf-8'))
            else:
                # No data received - check timeout
                elapsed = (self.get_clock().now() - self.last_packet_time).nanoseconds / 1e9
                if elapsed > self.timeout:
                    # Send zero velocity to stop robot
                    msg = Twist()
                    self.cmd_vel_pub.publish(msg)
                    
        except Exception as e:
            self.get_logger().error(f'Error receiving UDP: {e}')
    
    def process_joystick_data(self, data_str):
        """Parse JSON joystick data and publish cmd_vel"""
        try:
            data = json.loads(data_str)
            
            # Extract joystick values (-1.0 to 1.0)
            ly = data.get('ly', 0.0)  # Left stick Y -> linear velocity
            rx = data.get('rx', 0.0)  # Right stick X -> angular velocity
            
            # Apply deadzone to prevent drift from centered joystick
            if abs(ly) < self.deadzone:
                ly = 0.0
            if abs(rx) < self.deadzone:
                rx = 0.0
            
            # Create Twist message
            msg = Twist()
            msg.linear.x = ly * self.max_linear
            msg.angular.z = rx * self.max_angular
            
            # Publish
            self.cmd_vel_pub.publish(msg)
            
            # Update watchdog
            self.last_packet_time = self.get_clock().now()
            
            # Log (only significant commands to avoid spam)
            if abs(msg.linear.x) > 0.01 or abs(msg.angular.z) > 0.01:
                self.get_logger().info(
                    f'cmd_vel: linear.x={msg.linear.x:.2f}, angular.z={msg.angular.z:.2f}',
                    throttle_duration_sec=1.0
                )
                
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Invalid JSON: {e}')
        except Exception as e:
            self.get_logger().error(f'Error processing data: {e}')
    
    def destroy_node(self):
        """Clean up socket on shutdown"""
        self.sock.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UDPJoystickTeleop()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
