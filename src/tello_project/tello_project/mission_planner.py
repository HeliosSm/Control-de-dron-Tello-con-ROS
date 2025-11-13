#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class TelloMission(Node):
    def __init__(self):
        super().__init__('tello_mission')
        self.cmd_pub = self.create_publisher(String, '/tello/control', 10)
        self.get_logger().info("MISIÓN INICIADA - PUBLICANDO COMANDOS...")

        # Ejecutar misión
        self.execute_mission()

    def send(self, cmd):
        msg = String()
        msg.data = cmd
        self.cmd_pub.publish(msg)
        self.get_logger().info(f"→ {cmd}")
        time.sleep(0.5)  # Pequeña pausa entre comandos

    def execute_mission(self):
        # 1. INICIAR
        self.send('command')
        time.sleep(2)

        # 2. DESPEGAR
        self.send('takeoff')
        time.sleep(3)

        # 3. AVANZAR 50 cm
        self.send('back 50')
        time.sleep(3)

        # 4. GIRAR 90° DERECHA
        self.send('cw 90')
        time.sleep(4)
        
        # 5. GIRAR 90° DERECHA
        self.send('ccw 90')
        time.sleep(4)


        # 6. RETROCEDER 50 cm
        self.send('forward 50')
        time.sleep(3)

        # 7. ATERRIZAR
        self.send('land')
        time.sleep(3)

        self.get_logger().info("MISIÓN COMPLETA - ¡VOLVIÓ AL INICIO!")

def main(args=None):
    rclpy.init(args=args)
    node = TelloMission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
