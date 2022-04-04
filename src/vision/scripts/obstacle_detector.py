#!/usr/bin/env python

import cv2
import numpy as np
import roslib
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String


# Kinect vision for Turtlebot
class TurtlebotKinect:
    def __init__(self):
        self.depth_sub = rospy.Subscriber('/camera/depth/image_raw', Image, self.get_depth_image)
        self.depth_pub = rospy.Publisher('/occupancy_state', String, queue_size=10)
        self.bridge = CvBridge()
        self.depth = None
        self.run()

    # Receive depth image from Kinect sensor
    def get_depth_image(self, data):
        self.depth = self.bridge.imgmsg_to_cv2(data)

    # Process images from Kinect sensor @5Hz
    def run(self):
        while not rospy.is_shutdown() and self.depth_sub.get_num_connections() == 0:
            pass
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            self.detect_obstacles()
            rate.sleep()

    # Detect obstacles in estimated depth image
    def detect_obstacles(self):
        if self.depth is None:
            self.depth_pub.publish('free')
        # TODO: detect obstacle in image matrix
        self.depth_pub.publish(str(self.depth))

# Detect obstacles in Kinect images
if __name__ == '__main__':
    rospy.init_node('obstacle_detector')
    handler = TurtlebotKinect()
    rospy.spin()
