#!/usr/bin/env python
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from pdb import set_trace as bp


class WallDistanceDetector:
    RATE_HZ = 5
    
    def __init__(self):
        rospy.init_node('wall_distance_detector', anonymous=True)
        self.depth_sub = rospy.Subscriber('/camera/depth/image_raw', Image, self.on_image)
        self.distance_pub = rospy.Publisher('/wall_distance', Float32MultiArray, queue_size=10)
        self.rate = rospy.Rate(self.RATE_HZ)
        self.bridge = CvBridge()


    def on_image(self, data):
        img = self.bridge.imgmsg_to_cv2(data)
        width, height = img.shape
        print(img[width // 2][-10:])
        # breakpoint()
        # print('Got dat a %s', depth) 

    def run(self):
        while not rospy.is_shutdown(): 
            self.rate.sleep()

if __name__ == "__main__":
    detector = WallDistanceDetector()
    rospy.sleep(0.4)
    detector.run()
    rospy.spin()