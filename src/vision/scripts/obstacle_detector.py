#!/usr/bin/env python

import numpy as np
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

    # Check if objects in matrix partition are closer than 0.6 [m]
    @staticmethod
    def check_distance(matrix):
        # If at least 80% of the pixels are NaN, assume an obstacle
        size = matrix.shape[0] * matrix.shape[1]
        if np.count_nonzero(np.isnan(matrix)) >= size * 0.8:
            return True

        # If any of the pixels have distance < 0.6 then there is an obstacle
        fixed_matrix = np.nan_to_num(matrix, nan=1000)  # Replace NaN with a large invalid distance
        return np.any(fixed_matrix < 0.6)

    # Detect obstacles in estimated depth image
    def detect_obstacles(self):
        # By default the robot assumes the path is free
        if self.depth is None:
            self.depth_pub.publish('free')
            return

        # Divide the depth image in 3 (left, center, right) and detect obstacles in each partition
        detection = 'free'
        if self.check_distance(self.depth[:, 212:426]):  # Center
            detection = 'obstacle_center'
        elif self.check_distance(self.depth[:, :212]):  # Left
            detection = 'obstacle_left'
        elif self.check_distance(self.depth[:, 426:]):  # Right
            detection = 'obstacle_right'
        self.depth_pub.publish(detection)  # Publish detection

# Detect obstacles in Kinect images
if __name__ == '__main__':
    rospy.init_node('obstacle_detector')
    handler = TurtlebotKinect()
    rospy.spin()
