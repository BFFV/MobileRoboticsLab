#!/usr/bin/env python

import os
import rospy
from geometry_msgs.msg import PoseArray, Pose


# Reads poses from a file
class PoseFileReader:
    """Reads poses from a file. A pose is a line with float values separated by spaces"""
    def __init__(self):
        rospy.init_node('pose_file_reader', anonymous=True)
        self.pose_pub = rospy.Publisher('/goal_list', PoseArray, queue_size=10)

    def read_and_publish(self, filename):
        dirname = os.path.dirname(__file__)
        abs_file = os.path.join(dirname, filename)
        poses = []
        with open(abs_file, encoding='utf-8') as file:
            for line in file:
                line = line.strip()
                if not line or line[0] == '#':
                    continue # Skip empty lines and comments
                x, y, w = [float(num) for num in line.split()]
                pose = Pose()
                pose.position.x = x
                pose.position.y = y
                pose.orientation.w = w
                poses.append(pose)
        self.pose_pub.publish(PoseArray(poses=poses))
        rospy.loginfo('Successfully published %d goal poses', len(poses))

# Pose reading from files
if __name__ == '__main__':
    reader = PoseFileReader()
    rospy.sleep(0.5)
    reader.read_and_publish('../input_files/trajectory/square.txt')
