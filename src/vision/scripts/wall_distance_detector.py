#!/usr/bin/env python3

import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray


# Detect distance to walls
class WallDistanceDetector:
    RATE_HZ = 10
    MAX_DEPTH = 10
    CHANGE_TRESHOLD = 0.4
    MEAN_PATCH_SIZE = 30

    def __init__(self):
        rospy.init_node('wall_distance')
        self.depth_sub = rospy.Subscriber('/camera/depth/image_raw', Image, self.on_image)
        self.distance_pub = rospy.Publisher('/wall_distance', Float32MultiArray, queue_size=1)
        self.new_img_pub = rospy.Publisher('/new_img', Image, queue_size=1)
        self.rate = rospy.Rate(self.RATE_HZ)
        self.bridge = CvBridge()

    def on_image(self, data):
        img = self.bridge.imgmsg_to_cv2(data)
        width, height = img.shape
        matrix = img.copy()
        
        cut_left = 0.2
        cut_right = 0.2
        
        matrix = matrix / 1000
        matrix = matrix[:,int(cut_left * height): int((1 - cut_right) * height)]
        
        height, width = matrix.shape

        # Return invalid in case of a wall too close to the robot
        
        # print(np.count_nonzero(matrix < 0.3))
        
        if np.count_nonzero(matrix == 0) >= matrix.shape[0] * matrix.shape[1] * 0.7:
            self.publish_distance_msg([-1, -1])
            print("stop")
            return
        
        matrix = self.preprocess(matrix)
        
        # self.new_img_pub.publish(self.bridge.cv2_to_imgmsg(matrix * 1000, encoding="passthrough"))
        
        change_coords = self.detect_wall_change(matrix)
        
        # print(matrix[height // 2, width // 2])

        distance_left_matrix = matrix[height // 2 - self.MEAN_PATCH_SIZE: height // 2 + self.MEAN_PATCH_SIZE, :self.MEAN_PATCH_SIZE]
        distance_right_matrix = matrix[height // 2 - self.MEAN_PATCH_SIZE: height // 2 + self.MEAN_PATCH_SIZE,  -self.MEAN_PATCH_SIZE:]
        
        # print(distance_left_matrix)
        
        distance_left = distance_left_matrix.mean()
        distance_right = distance_right_matrix.mean()

        # if change_coords:
        vals = [max(0.3, distance_left), 1]
        print(vals)
        self.publish_distance_msg(vals)
        return

        slope = self.get_horizontal_slope(matrix)
        vals = [distance_left, -1] if slope > 0 else [-1, distance_right] 
        self.publish_distance_msg(vals)

    @staticmethod
    def preprocess(matrix):
        matrix[matrix > WallDistanceDetector.MAX_DEPTH] = WallDistanceDetector.MAX_DEPTH
        matrix = np.nan_to_num(matrix)
        return matrix

    @staticmethod
    def detect_wall_change(matrix):
        height, width = matrix.shape
        patch_width = min(20, width)
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
