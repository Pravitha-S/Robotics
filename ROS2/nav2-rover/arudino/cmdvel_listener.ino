#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial

class CmdVelListener(Node):
    def __init__(self):
        super().__init__('cmdvel_listener')

        # Declare runtime port parameter (default /dev/ttyUSB0)
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 9600)

        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value

        # Open serial connection
        try:
            self.ser = serial.Serial(port, baudrate, timeout=1)
            self.get_logger().info(f"Connected to serial port {port}")
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port {port}: {e}")
            self.ser = None

        # Subscribe to cmd_vel
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg: Twist):
        linear = msg.linear.x
        angular = msg.angular.z

        # Scale values: linear (m/s → *100), angular (rad/s → *25)
        scaled_linear = int(linear * 100)
        scaled_angular = int(angular * 25)

        data = f"{scaled_linear},{scaled_angular}\n"

        if self.ser and self.ser.is_open:
            self.ser.write(data.encode())
            self.get_logger().info(f"Sent: {data.strip()}")
        else:
            self.get_logger().warn(f"Serial not connected, would send: {data.strip()}")

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
