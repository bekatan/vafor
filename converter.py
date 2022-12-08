#!/usr/bin/env python
from __future__ import print_function
import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:
    def __init__(self):
        self.image_pub = rospy.Publisher("/detection_image",Image)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/darknet_ros/detection_image",Image,self.callback)
    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "rgb8")
        except CvBridgeError as e:
            print(e)
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            print(e)
    
def main(args):
    ic = image_converter()
    rospy.init_node('detection_image_converter', anonymous=True)
    rospy.spin()
if __name__ == '__main__':
    main(sys.argv)
