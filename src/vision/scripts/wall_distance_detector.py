#!/usr/bin/env python

import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray


# Detect distance to walls
class WallDistanceDetector:
    RATE_HZ = 10
    MAX_DEPTH = 1.5
    CHANGE_TRESHOLD = 0.4

    def __init__(self):
        rospy.init_node('wall_distance')
        self.depth_sub = rospy.Subscriber('/camera/depth/image_raw', Image, self.on_image)
        self.distance_pub = rospy.Publisher('/wall_distance', Float32MultiArray, queue_size=10)
        self.rate = rospy.Rate(self.RATE_HZ)
        self.bridge = CvBridge()

    def on_image(self, data):
        img = self.bridge.imgmsg_to_cv2(data)
        width, height = img.shape
        matrix = img.copy()

        # Return invalid in case of a wall too close to the robot
        if np.count_nonzero(np.isnan(matrix)) >= width * height * 0.98:
            self.publish_distance_msg([-1, -1])
            return
        matrix = self.preprocess(matrix)
        change_coords = self.detect_wall_change(matrix)

        distance_left = matrix[height // 2, 0]
        distance_right = matrix[height // 2, -1]

        if change_coords:
            vals = [distance_left, distance_right]
            self.publish_distance_msg(vals)
            return

        slope = self.get_horizontal_slope(matrix)
        vals = [distance_left, -1] if slope > 0 else [-1, distance_right] 
        self.publish_distance_msg(vals)

    @staticmethod
    def preprocess(matrix):
        matrix[matrix > WallDistanceDetector.MAX_DEPTH] = 20
        matrix = np.nan_to_num(matrix)
        return matrix

    @staticmethod
    def detect_wall_change(matrix):
        height, width = matrix.shape
        patch_width = min(10, width)
        patch_height = min(20, height)
        last_mean = None
        half = height // 2
        for offset in range(0, width, patch_width):
            current_patch_end = min(offset + patch_width, width)
            patch = matrix[half - patch_height // 2 : half + patch_height // 2,
                           offset : current_patch_end]
            mean = np.mean(patch)
            if last_mean is None: last_mean = mean
            diff = abs(last_mean - mean)
            if diff >= WallDistanceDetector.CHANGE_TRESHOLD:
                return (offset, half)
            last_mean = mean
        return None

    @staticmethod
    def get_horizontal_slope(matrix):
        _, width = matrix.shape
        half_left = np.mean(matrix[:, :width // 2])
        half_right = np.mean(matrix[:, width // 2:])
        return half_right - half_left

    def publish_distance_msg(self, values):
        msg = Float32MultiArray(data=values)
        self.distance_pub.publish(msg)

    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

# Wall distance detection
if __name__ == "__main__":
    detector = WallDistanceDetector()
    rospy.sleep(0.4)
    detector.run()
    rospy.spin()
