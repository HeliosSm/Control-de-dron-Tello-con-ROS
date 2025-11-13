#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tello_msgs.srv import TelloControl

class DroneConnector(Node):
    def __init__(self):
        super().__init__('drone_connector')
        self.cli = self.create_client(TelloControl, '/tello/control')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Esperando servicio Tello...')
        self.get_logger().info('Conexión con Tello establecida.')
        self.send_command('command')

    def send_command(self, cmd):
        req = TelloControl.Request()
        req.command = cmd
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f"Respuesta: {future.result().response}")
        else:
            self.get_logger().error("Fallo al enviar comando")

def main():
    rclpy.init()
    node = DroneConnector()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
