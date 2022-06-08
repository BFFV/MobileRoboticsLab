#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseArray


# Writes Poses from topic /position_diff_writer to a file
class PositionDiffWriter:
    """Writes Poses from topic /position_diff_writer to a file"""
    def __init__(self, filename):
        self.filename = filename
        rospy.init_node('position_diff_writer', anonymous=True)
        rospy.diff_sub = rospy.Subscriber('/position_diff', PoseArray, self.position_cb)
        self.output_file = open(self.filename, 'w')

    def position_cb(self, poses):
        # Two arguments must be passed
        real_pose, odom_pose = poses.poses
        x_real = real_pose.position.x
        y_real = real_pose.position.y
        w_real = real_pose.orientation.w
        x_odom = odom_pose.position.x
        y_odom = odom_pose.position.y
        w_odom = odom_pose.orientation.w
        csv_line = (x_real, y_real, w_real, x_odom, y_odom, w_odom)
        self.output_file.write(','.join([str(x) for x in csv_line]) + '\n')
        rospy.loginfo(f'Got Real: {x_real} {y_real} {w_real}\n'
                      f'Got Odom: {x_odom} {y_odom} {w_odom}')

# Write to output file
if __name__ == '__main__':
    diff = PositionDiffWriter('output.csv')
    rospy.sleep(0.3)
    rospy.spin()
