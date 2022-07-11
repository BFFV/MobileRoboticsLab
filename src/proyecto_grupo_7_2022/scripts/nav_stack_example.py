#!/usr/bin/env python

import actionlib
import rospy
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Header


# Navigation stack demo
class StackDemo:
    RATE_HZ = 10

    def __init__(self):
        rospy.init_node('demo')
        self.rate = rospy.Rate(self.RATE_HZ)
        self.initial_pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
        self.move_base_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)

    def run(self):
        self.move_base_client.wait_for_server()
        self.set_initial_pose(2.1, 3.2, 0.1)
        self.move_to_pose([
            (24, 3.5, 0.99),
            (25, 16, 0.018),
            (8, 16, 0.54),
            (2.1, 3.2, 0.4)
        ])

    def move_to_pose(self, pose_array):
        if not pose_array: return

        x, y, theta = pose_array[0]

        # Define a goal pose
        goal_pose = Pose()
        goal_pose.position.x, goal_pose.position.y = x, y
        goal_pose.orientation.w = theta

        # Define MoveBaseGoal (ROS ActionLib API)
        move_base_goal = MoveBaseGoal()
        move_base_goal.target_pose.header.frame_id = 'map'
        move_base_goal.target_pose.header.stamp = rospy.get_rostime()
        move_base_goal.target_pose.pose = goal_pose

        def next_call(status, result):
            rospy.loginfo('DONE status: %s (%s)', self.move_base_client.get_goal_status_text(), str(status))
            self.move_to_pose(pose_array[1:])

        self.move_base_client.send_goal(move_base_goal, done_cb=next_call)

    def set_initial_pose(self, x, y, theta):
        initial_pose = Pose()
        initial_pose.position.x, initial_pose.position.y = x, y
        initial_pose.orientation.w = theta
        pose_w_cov_s = PoseWithCovarianceStamped()
        pose_w_cov_s.pose.pose = initial_pose
        header = Header()
        header.frame_id = 'map'
        header.stamp = rospy.get_rostime()
        pose_w_cov_s.header = header
        self.initial_pose_pub.publish(pose_w_cov_s)


# Navigate corridor
if __name__ == '__main__':
    navigator = StackDemo()
    rospy.sleep(0.4)
    navigator.run()
    rospy.spin()
