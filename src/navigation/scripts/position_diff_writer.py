#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseArray

class PositionDiffWriter:
    
    def __init__(self):
        rospy.init_node("position_diff_writer", anonymous=True)
        rospy.diff_sub = rospy.Subscriber('/position_diff', PoseArray, self.position_cb)
        self.output_file = open("output.csv", "w")
        
    def position_cb(self, poses):
        real_pose, odom_pose, expected = poses.poses
        
        x_real = real_pose.position.x
        y_real = real_pose.position.y
        w_real = real_pose.orientation.w
        
        
        x_odom = odom_pose.position.x
        y_odom = odom_pose.position.y
        w_odom = odom_pose.orientation.w
        
        x_expected = expected.position.x
        y_expected = expected.position.y
        w_expected = expected.orientation.w
        
        self.output_file.write(",".join([str(x) for x in (x_real, y_real, w_real, x_odom, y_odom, w_odom, x_expected, y_expected, w_expected)]) + "\n")
        
        
        rospy.loginfo(f"Got Real: {x_real} {y_real} {w_real}\n"
                      f"Got Odom: {x_odom} {y_odom} {w_odom}\n"
                      f"Got Expected: {x_expected} {y_expected} {w_expected}")
        
        
if __name__ == "__main__":
    diff = PositionDiffWriter()
    rospy.sleep(0.3)
    rospy.spin()