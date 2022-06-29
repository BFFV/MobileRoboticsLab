#!/usr/bin/env python3

import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray


# Detect distance to walls
class WallDistanceDetector:
    RATE_HZ = 10
    MAX_DEPTH = 3
    # CHANGE_TRESHOLD = 0.4
    MEAN_PATCH_SIZE = 300

    def __init__(self):
        rospy.init_node('wall_distance')
        self.depth_sub = rospy.Subscriber('/camera/depth/image_raw', Image, self.on_image)
        self.distance_pub = rospy.Publisher('/wall_distance', Float32MultiArray, queue_size=1)
        # self.new_img_pub = rospy.Publisher('/new_img', Image, queue_size=1)
        self.rate = rospy.Rate(self.RATE_HZ)
        self.bridge = CvBridge()
        self.print_counter = 0

    def on_image(self, data):
        img = self.bridge.imgmsg_to_cv2(data)
        width, height = img.shape
        matrix = img.copy()
        
        cut_left = 0.1
        cut_right = 0.1
        
        matrix = matrix / 1000
        # matrix = matrix[:,int(cut_left * width): int((1 - cut_right) * width)]
        
        height, width = matrix.shape

        # Return invalid in case of a wall too close to the robot
        
        # print(np.count_nonzero(matrix < 0.3))
        
        # if np.count_nonzero(matrix == 0) >= matrix.shape[0] * matrix.shape[1] * 0.7:
        #     self.publish_distance_msg([-1, -1])
        #     return
        
        matrix = self.preprocess(matrix)
        
        # self.new_img_pub.publish(self.bridge.cv2_to_imgmsg(matrix * 1000, encoding="passthrough"))
        
        # change_coords = self.detect_wall_change(matrix)
        
        # print(matrix[height // 2, width // 2])
        
        y_size = 120

        distance_left_matrix = matrix[200 :, :self.MEAN_PATCH_SIZE]
        distance_right_matrix = matrix[200 :,  -self.MEAN_PATCH_SIZE:]
        
        # distance_left = np.max(distance_left_matrix)
        # distance_right = np.max(distance_right_matrix)
        
        # distance_left = np.percentile(distance_left_matrix, 80)
        # distance_right = np.percentile(distance_right_matrix, 80)
        
        distance_left = np.mean(distance_left_matrix)
        distance_right = np.mean(distance_right_matrix)
        
        
        # tresh = 1
        # if distance_left == self.MAX_DEPTH:
        #     vals = [1, max(0.3, distance_right)]
        # else:
        #     vals = [max(0.3, distance_left), 1]
        
        self.print_counter += 1
        
        if self.print_counter % 20  == 0:
            # print([distance_left, distance_right, distance_right - distance_left])
            print(distance_right - distance_left)
            
        
        # if distance_left < distance_right:
        stop  = 1 if min(distance_left, distance_right) < 0.4 else 0
        
        self.publish_distance_msg([distance_left, distance_right, stop])
        # else:
            # self.publish_distance_msg([distance_right * 1.5, distance_right])
            
    @staticmethod
    def noise_reduce(matrix):
        # out = matrix.copy()
        out = matrix
        for y, row in enumerate(matrix):
            for x, element in enumerate(row):
                if element == 0:
                    for side in range(30, 100, 10):
                        limit_up = max(0, y - side)
                        limit_down = min(matrix.shape[0], y + side)
                        limit_left = max(0, x - side)
                        limit_right = min(matrix.shape[1], x + side)
                        window = matrix[limit_up:limit_down, limit_left:limit_right]
                        
                        value = window[window > 0].mean()
                        if value > 0:
                            out[y, x] = value
                            break
        return out
    
    
    @staticmethod
    def preprocess(matrix):
        matrix[matrix > WallDistanceDetector.MAX_DEPTH] = WallDistanceDetector.MAX_DEPTH
        matrix = np.nan_to_num(matrix)
        # matrix = WallDistanceDetector.noise_reduce(matrix)
        # matrix[matrix > 0] *= 2
        # matrix[matrix == 0] = 0.1
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
