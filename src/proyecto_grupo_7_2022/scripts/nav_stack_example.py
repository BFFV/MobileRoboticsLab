#!/usr/bin/env python

import rospy
import actionlib
from geometry_msgs.msg import Pose


def goal_done(status, result):
    rospy.loginfo('DONE status: %s (%s)', move_base_client.get_goal_status_text(), str(status))


if __name__ == '__main__':
    rospy.init_node('nav_stack_example')

    # TODO: MoveBaseAction needs to be defined (ROS ActionLib API)
    move_base_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
    move_base_client.wait_for_server()

    # TODO: Publish to /initialpose topic to set the starting point for the robot (only once)

    # TODO: Define a goal pose
    goal_pose = Pose(position, orientation)

    # TODO: Define MoveBaseGoal (ROS ActionLib API)
    move_base_goal = MoveBaseGoal()
    move_base_goal.target_pose.header.frame_id = 'map'
    move_base_goal.target_pose.header.stamp = rospy.Time.now()
    move_base_goal.target_pose.pose = goal_pose
    move_base_client.send_goal(move_base_goal, done_cb=goal_done)

    # TODO: Repeat for 3 goal poses sequentially

    rospy.spin()
