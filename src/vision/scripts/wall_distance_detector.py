#!/usr/bin/env python3

import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray


# Detect distance to walls
class WallDistanceDetector:
    RATE_HZ = 10
    MAX_DEPTH = 4
    # CHANGE_TRESHOLD = 0.4
    MEAN_PATCH_SIZE = 30

    def __init__(self):
        rospy.init_node('wall_distance')
        self.depth_sub = rospy.Subscriber('/camera/depth/image_raw', Image, self.on_image)
        self.distance_pub = rospy.Publisher('/wall_distance', Float32MultiArray, queue_size=1)
        self.new_img_pub = rospy.Publisher('/new_img', Image, queue_size=1)
        self.rate = rospy.Rate(self.RATE_HZ)
        self.bridge = CvBridge()
        self.print_counter = 0

    def on_image(self, data):
        img = self.bridge.imgmsg_to_cv2(data)
        width, height = img.shape
        matrix = img.copy()
        
        cut_left = 0.2
        cut_right = 0.2
        
        matrix = matrix / 1000
        matrix = matrix[:,int(cut_left * width): int((1 - cut_right) * width)]
        
        height, width = matrix.shape

        # Return invalid in case of a wall too close to the robot
        
        # print(np.count_nonzero(matrix < 0.3))
        
        if np.count_nonzero(matrix == 0) >= matrix.shape[0] * matrix.shape[1] * 0.7:
            self.publish_distance_msg([-1, -1])
            return
        
        matrix = self.preprocess(matrix)
        
        # self.new_img_pub.publish(self.bridge.cv2_to_imgmsg(matrix * 1000, encoding="passthrough"))
        
        # change_coords = self.detect_wall_change(matrix)
        
        # print(matrix[height // 2, width // 2])

        distance_left_matrix = matrix[:, :self.MEAN_PATCH_SIZE]
        distance_right_matrix = matrix[:,  -self.MEAN_PATCH_SIZE:]
        
        distance_left = distance_left_matrix.mean()
        distance_right = distance_right_matrix.mean()
        
        # tresh = 1
        # if distance_left == self.MAX_DEPTH:
        #     vals = [1, max(0.3, distance_right)]
        # else:
        #     vals = [max(0.3, distance_left), 1]
        
        self.publish_distance_msg([distance_left, distance_right])
        
        self.print_counter += 1
        
        if self.print_counter % 20 / 2 == 0:
            print([distance_left, distance_right])
    
    
    @staticmethod
    def preprocess(matrix):
        matrix[matrix > WallDistanceDetector.MAX_DEPTH] = WallDistanceDetector.MAX_DEPTH
        matrix = np.nan_to_num(matrix)
        # matrix[matrix == 0] = WallDistanceDetector.MAX_DEPTH
        return matrix

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
