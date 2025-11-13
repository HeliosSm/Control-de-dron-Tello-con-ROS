#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class SaveImage(Node):
    def __init__(self):
        super().__init__('save_image')
        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, '/tello/image_raw', self.callback, 10)
        self.count = 0

    def callback(self, msg):
        if self.count >= 1:
            return
        self.count += 1

        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        filename = "/tmp/tello_real_image.jpg"
        cv2.imwrite(filename, cv_image)
        self.get_logger().info(f"IMAGEN GUARDADA EN: {filename}")
        print(f"\nABRE ESTA IMAGEN CON CUALQUIER VISOR: {filename}\n")
        rclpy.shutdown()

def main():
    rclpy.init()
    node = SaveImage()
    rclpy.spin(node)

if __name__ == '__main__':
    main()