#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32

class TelemetryMonitor(Node):
    def __init__(self):
        super().__init__('telemetry_monitor')

        # Suscripciones a los tópicos reales del driver
        self.bat_sub = self.create_subscription(
            Int32, '/tello/battery', self.battery_callback, 10)
        
        self.height_sub = self.create_subscription(
            Float32, '/tello/height', self.height_callback, 10)

        self.get_logger().info("Monitor de telemetría iniciado")

    def battery_callback(self, msg):
        self.get_logger().info(f"BATERÍA: {msg.data}%")

    def height_callback(self, msg):
        self.get_logger().info(f"ALTURA: {msg.data:.1f} cm")

def main(args=None):
    rclpy.init(args=args)
    node = TelemetryMonitor()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
