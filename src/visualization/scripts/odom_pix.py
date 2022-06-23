#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


# Odometry to pixels in an image
class OdomPixel():
    def __init__(self):
        rospy.init_node('odom_pix')
        self.variables_init()
        self.connections_init()
        rospy.spin()

    # Initialize variables
    def variables_init(self):
        self.resolution = 0.01
        self.map_height = 270
        self.init_x = 0.5
        self.init_y = 0.5
        self.init_angle = 0
        self.initialized = False

    # Initialize connections
    def connections_init(self):
        self.pub_odom_pix = rospy.Publisher('/odom_pix', Pose, queue_size=1)
        rospy.Subscriber('/real_pose', Pose, self.set_init_pose)
        rospy.Subscriber('/odom', Odometry, self.odom_pix)

    # Set initial pose
    def set_init_pose(self, pose_data):
        if self.initialized:
            return
        self.initialized = True
        self.init_x = pose_data.position.x
        self.init_y = pose_data.position.y
        quaternion = (pose_data.orientation.x,
                      pose_data.orientation.y,
                      pose_data.orientation.z,
                      pose_data.orientation.w)
        _, _, self.init_angle = euler_from_quaternion(quaternion)

    # Get data from odometry and transform into pixels
    def odom_pix(self, odom_data):
        if not self.initialized:
            return
        pose_data = odom_data.pose.pose
        robot_x_pix = int((pose_data.position.x + self.init_x) / self.resolution)
        robot_y_pix = int(self.map_height - (pose_data.position.y + self.init_y) / self.resolution)
        quaternion = (pose_data.orientation.x,
                      pose_data.orientation.y,
                      pose_data.orientation.z,
                      pose_data.orientation.w)
        _, _, yaw = euler_from_quaternion(quaternion)
        robot_ang = yaw + self.init_angle
        pose_pix = Pose()
        pose_pix.position.x = robot_x_pix
        pose_pix.position.y = robot_y_pix
        pose_pix.orientation.z = robot_ang
        self.pub_odom_pix.publish(pose_pix)


# Publish robot odometry in pixels format
if __name__ == '__main__':
    odom_pixel = OdomPixel()
