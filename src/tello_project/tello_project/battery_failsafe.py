#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String

class BatteryFailsafe(Node):
    def __init__(self):
        super().__init__('battery_failsafe')
        self.sub = self.create_subscription(Int32, '/tello/battery', self.callback, 10)
        self.pub = self.create_publisher(String, '/tello/control', 10)
        self.get_logger().info("Failsafe de batería activado (<30%)")

    def callback(self, msg):
        if msg.data < 30:
            self.get_logger().warn(f"¡BATERÍA CRÍTICA: {msg.data}%! → Aterrizando...")
            cmd = String()
            cmd.data = 'land'
            self.pub.publish(cmd)

def main():
    rclpy.init()
    node = BatteryFailsafe()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
