#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from cv_bridge import CvBridge
import cv2
import numpy as np
import os

# Forzar render por CPU
os.environ["QT_QPA_PLATFORM"] = "xcb"
os.environ["LIBGL_ALWAYS_SOFTWARE"] = "1"
os.environ["XDG_RUNTIME_DIR"] = "/tmp"

class ObjectDetector(Node):
    def __init__(self):
        super().__init__('object_detector')
        self.bridge = CvBridge()

        # Suscripción al video
        self.sub = self.create_subscription(
            Image, '/tello/image_raw', self.callback, 10)

        # Publicador
        self.pub = self.create_publisher(Int32, '/detected_objects', 10)

        # Ventana
        cv2.namedWindow("Detección Rojo/Negro", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Detección Rojo/Negro", 640, 480)

        self.get_logger().info("Detección iniciada - ROJO Y NEGRO CORRECTOS")

    def callback(self, msg):
        try:
            # --- CORREGIR COLOR: YUV → BGR ---
            img = self.bridge.imgmsg_to_cv2(msg, "rgb8")      # ← rgb8
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)        # ← a BGR

            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

            # --- MÁSCARA ROJO (mejorada) ---
            lower_red1 = np.array([0, 100, 100])
            upper_red1 = np.array([10, 255, 255])
            lower_red2 = np.array([160, 100, 100])
            upper_red2 = np.array([180, 255, 255])
            mask_red = cv2.inRange(hsv, lower_red1, upper_red1) + \
                       cv2.inRange(hsv, lower_red2, upper_red2)

            # --- MÁSCARA NEGRO ---
            lower_black = np.array([0, 0, 0])
            upper_black = np.array([180, 255, 50])
            mask_black = cv2.inRange(hsv, lower_black, upper_black)

            # --- CONTORNOS ROJOS ---
            contours_red, _ = cv2.findContours(mask_red, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            red_count = 0
            for cnt in contours_red:
                area = cv2.contourArea(cnt)
                if area > 800:
                    x, y, w, h = cv2.boundingRect(cnt)
                    cv2.rectangle(img, (x, y), (x+w, y+h), (0, 0, 255), 3)
                    cv2.putText(img, "ROJO", (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                    red_count += 1

            # --- CONTORNOS NEGROS ---
            contours_black, _ = cv2.findContours(mask_black, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            black_count = 0
            for cnt in contours_black:
                area = cv2.contourArea(cnt)
                if area > 800:
                    x, y, w, h = cv2.boundingRect(cnt)
                    cv2.rectangle(img, (x, y), (x+w, y+h), (255, 255, 255), 3)
                    cv2.putText(img, "NEGRO", (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                    black_count += 1

            # --- TOTAL ---
            total = red_count + black_count

            # --- TEXTO ---
            cv2.putText(img, f"ROJOS: {red_count}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
            cv2.putText(img, f"NEGROS: {black_count}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)
            cv2.putText(img, f"TOTAL: {total}", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 3)

            # --- MOSTRAR ---
            cv2.imshow("Detección Rojo/Negro", img)
            cv2.waitKey(1)

            # --- PUBLICAR ---
            count_msg = Int32()
            count_msg.data = total
            self.pub.publish(count_msg)

        except Exception as e:
            self.get_logger().error(f"Error: {e}")

def main():
    rclpy.init()
    node = ObjectDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
