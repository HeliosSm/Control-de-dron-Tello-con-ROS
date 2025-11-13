#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String, Int32, Float32
from djitellopy import Tello
import cv2
import threading
import time

class TelloDriver(Node):
    def __init__(self):
        super().__init__('tello_driver')
        self.tello = Tello()
        self.bridge = CvBridge()

        # Publishers
        self.image_pub = self.create_publisher(Image, '/tello/image_raw', 10)
        self.battery_pub = self.create_publisher(Int32, '/tello/battery', 10)
        self.height_pub = self.create_publisher(Float32, '/tello/height', 10)
        self.status_pub = self.create_publisher(String, '/tello/status', 10)

        # Subscriber para comandos
        self.cmd_sub = self.create_subscription(
            String, '/tello/control', self.cmd_callback, 10)

        # Conectar
        try:
            self.tello.connect()
            self.get_logger().info(f'Tello conectado! Batería: {self.tello.get_battery()}%')
        except Exception as e:
            self.get_logger().error(f'Error conectando: {e}')
            return

        # Iniciar video y estado
        self.tello.streamon()
        threading.Thread(target=self.video_loop, daemon=True).start()
        threading.Thread(target=self.status_loop, daemon=True).start()

    def cmd_callback(self, msg):
        cmd = msg.data.strip().lower()
        self.get_logger().info(f'Comando recibido: {cmd}')

        try:
            if cmd == 'takeoff':
                self.tello.takeoff()
            elif cmd == 'land':
                self.tello.land()
            elif cmd == 'emergency':
                self.tello.emergency()
            elif cmd.startswith('forward'):
                dist = int(cmd.split()[1]) if len(cmd.split()) > 1 else 50
                self.tello.move_forward(dist)
            elif cmd.startswith('back'):
                dist = int(cmd.split()[1]) if len(cmd.split()) > 1 else 50
                self.tello.move_back(dist)
            elif cmd.startswith('left'):
                dist = int(cmd.split()[1]) if len(cmd.split()) > 1 else 50
                self.tello.move_left(dist)
            elif cmd.startswith('right'):
                dist = int(cmd.split()[1]) if len(cmd.split()) > 1 else 50
                self.tello.move_right(dist)
            elif cmd.startswith('up'):
                dist = int(cmd.split()[1]) if len(cmd.split()) > 1 else 50
                self.tello.move_up(dist)
            elif cmd.startswith('down'):
                dist = int(cmd.split()[1]) if len(cmd.split()) > 1 else 50
                self.tello.move_down(dist)
            elif cmd.startswith('cw'):
                deg = int(cmd.split()[1]) if len(cmd.split()) > 1 else 90
                self.tello.rotate_clockwise(deg)
            elif cmd.startswith('ccw'):
                deg = int(cmd.split()[1]) if len(cmd.split()) > 1 else 90
                self.tello.rotate_counter_clockwise(deg)
            elif cmd.startswith('flip'):
                direction = cmd.split()[1] if len(cmd.split()) > 1 else 'f'
                getattr(self.tello, f'flip_{direction}')()
            elif cmd == 'command':
                self.tello.send_rc_control(0, 0, 0, 0)
            else:
                self.get_logger().warn(f'Comando desconocido: {cmd}')

        except Exception as e:
            self.get_logger().error(f'Error ejecutando comando: {e}')

    def video_loop(self):
        frame_read = self.tello.get_frame_read()
        while rclpy.ok():
            frame = frame_read.frame
            if frame is not None and frame.size != 0:
                msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
                self.image_pub.publish(msg)
            time.sleep(1/30)  # ~30 FPS

    def status_loop(self):
        while rclpy.ok():
            try:
                battery = self.tello.get_battery()
                height = self.tello.get_height()
                speed = self.tello.get_speed_x()  # o get_speed_y, get_speed_z

                self.battery_pub.publish(Int32(data=battery))
                self.height_pub.publish(Float32(data=float(height)))

                status_msg = String()
                status_msg.data = f"bat:{battery}% h:{height}cm"
                self.status_pub.publish(status_msg)

            except Exception as e:
                self.get_logger().warn(f"Error leyendo estado: {e}")

            time.sleep(1)

def main(args=None):
    rclpy.init(args=args)
    node = TelloDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Apagando...")
    finally:
        node.tello.streamoff()
        node.tello.end()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
