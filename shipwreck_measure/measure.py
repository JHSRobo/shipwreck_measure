#!/usr/bin/env python3

import rclpy
from rclpy.node import Node 
from sensor_msgs.msg import Image 
import cv2
from cv_bridge import CvBridge 

class ShipwreckMeasureNode(Node):
    def __init__(self):
        super().__init__('shipwreck_measure')

        self.log = self.get_logger()

        self.clicks = [] 

        self.bridge = CvBridge()

        self.img_capture_sub = self.create_subscription(Image, "shipwreck", self.receive_frame, 10)

        cv2.namedWindow("Camera Feed", cv2.WND_PROP_FULLSCREEN)
        cv2.setWindowProperty("Camera Feed", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_NORMAL)

    def receive_frame(self, frame):
        frame = self.bridge.imgmsg_to_cv2(frame)
        cv2.imshow("Camera Feed", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)

    measure_node = ShipwreckMeasureNode()

    # Runs the program until shutdown
    rclpy.spin(measure_node)

    # On shutdown
    measure_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
