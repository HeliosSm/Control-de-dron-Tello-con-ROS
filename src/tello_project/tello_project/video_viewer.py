#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

os.environ["QT_QPA_PLATFORM"] = "xcb"
os.environ["LIBGL_ALWAYS_SOFTWARE"] = "1"

class VideoViewer(Node):
    def __init__(self):
        super().__init__('video_viewer')
        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, '/tello/image_raw', self.callback, 10)

    def callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        cv2.imshow("Tello - Video en Vivo", cv_image)
        cv2.waitKey(1)

def main():
    rclpy.init()
    node = VideoViewer()
    rclpy.spin(node)
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
