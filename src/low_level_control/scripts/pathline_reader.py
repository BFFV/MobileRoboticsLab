#!/usr/bin/env python

import os
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path


# Reads pathlines from a file
class PathlineReader:
    """Reads pathlines from a file. A pathline is a list of (x,y) coordinates."""
    def __init__(self):
        rospy.init_node('pathline_reader')
        self.path_pub = rospy.Publisher('/nav_plan', Path, queue_size=1)

    # Read pathline from file
    def read_and_publish(self, filename):
        dirname = os.path.dirname(__file__)
        abs_file = os.path.join(dirname, filename)
        points = []
        with open(abs_file, encoding='utf-8') as file:
            for line in file:
                line = line.strip()
                if not line or line[0] == '#':
                    continue # Skip empty lines and comments
                x, y = [float(num) for num in line.split(',')]
                point = PoseStamped()
                point.pose.position.x = x
                point.pose.position.y = y
                points.append(point)
        self.path_pub.publish(Path(poses=points))
        rospy.loginfo('Successfully published %d points for a pathline!', len(points))

# Pathline reading from files
if __name__ == '__main__':
    reader = PathlineReader()
    rospy.sleep(2)
    reader.read_and_publish('../input_files/paths/path_line.txt')
