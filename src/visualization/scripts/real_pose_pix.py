#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose
from tf.transformations import euler_from_quaternion


# Real pose to pixels in an image
class RealPosePixel():
    def __init__(self):
        rospy.init_node('real_pose_pix')
        self.variables_init()
        self.connections_init()
        rospy.spin()

    # Initialize variables
    def variables_init(self):
        self.resolution = 0.01
        self.map_height = 270

    # Initialize connections
    def connections_init(self):
        self.pub_real_pose_pix = rospy.Publisher('/real_pose_pix', Pose, queue_size=1)
        rospy.Subscriber('/real_pose', Pose, self.real_pose_pix)

    # Get data from real pose and transform into pixels
    def real_pose_pix(self, pose_data):
        robot_x_pix = int(pose_data.position.x / self.resolution)
        robot_y_pix = int(self.map_height - pose_data.position.y / self.resolution)
        quaternion = (pose_data.orientation.x,
                      pose_data.orientation.y,
                      pose_data.orientation.z,
                      pose_data.orientation.w)
        _, _, yaw = euler_from_quaternion(quaternion)
        robot_ang = yaw
        pose_pix = Pose()
        pose_pix.position.x = robot_x_pix
        pose_pix.position.y = robot_y_pix
        pose_pix.orientation.z = robot_ang
        self.pub_real_pose_pix.publish(pose_pix)


# Publish robot real pose in pixels format
if __name__ == '__main__':
    real_pose_pixel = RealPosePixel()
