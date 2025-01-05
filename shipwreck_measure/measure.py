#!/usr/bin/env python3

import rclpy
from rclpy.node import Node 
from std_msgs.msg import Bool
from sensor_msgs.msg import Image 
import cv2
from cv_bridge import CvBridge 
from math import sqrt

class ShipwreckMeasureNode(Node):
    def __init__(self):
        super().__init__('shipwreck_measure')

        self.log = self.get_logger()

        self.clicks = [] # List contianing all locations clicked on an image after selecting it from the feed
        self.actual_width = 20.75 # width of the ship in cm

        self.bridge = CvBridge()

        self.img_capture_sub = self.create_subscription(Image, "shipwreck", self.receive_frame, 10)
        self.waiting = True # variable contianing the information regarding whether the user has selected an image

        self.window_name = "Shipwreck"
        
        cv2.namedWindow(self.window_name, cv2.WND_PROP_FULLSCREEN)
        cv2.setWindowProperty(self.window_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(self.window_name, self.add_clicks)

    def receive_frame(self, frame):
        if self.waiting:
            self.frame = self.bridge.imgmsg_to_cv2(frame)
            cv2.imshow(self.window_name, self.frame)
            cv2.waitKey(1)

    def add_clicks(self, event, x, y, flags, param):
        # Checks if mouse was clicked
        if event == cv2.EVENT_LBUTTONDOWN:
            # If first click, stop receiving new photos 
            if self.waiting:
                self.waiting = False
                cv2.waitKey(0)
                return

            # If not first click, use the click location to calculate the length of the shipwreck
            # The first two clicks are along the width and the second two are along the length.
            self.clicks.append((x, y))
            self.frame = cv2.circle(self.frame, (x,y), 10, (255,0,0), -1)
            cv2.imshow(self.window_name, self.frame)

        if len(self.clicks) == 4:
            # Distance formula to find the width and length in pixels
            pixel_width = sqrt((self.clicks[0][0]-self.clicks[1][0])**2 + (self.clicks[0][1]-self.clicks[1][1])**2)
            pixel_length = sqrt((self.clicks[2][0]-self.clicks[3][0])**2 + (self.clicks[2][1]-self.clicks[3][1])**2)
            # Use a proportion to find the cm / pixel
            ratio = self.actual_width / pixel_width
            # Use that proportion to find the actual length given the pixel length
            actual_length = pixel_length *  ratio 
            
            # Log that lenght to terminal
            self.log.info(str(actual_length))
            cv2.destroyAllWindows()
            exit()



def main(args=None):
    rclpy.init(args=args)

    measure_node = ShipwreckMeasureNode()

    # Runs the program until shutdown
    rclpy.spin(measure_node)

    # On shutdown
    cv2.destroyAllWindows()
    measure_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
