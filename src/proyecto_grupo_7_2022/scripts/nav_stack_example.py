#!/usr/bin/env python3

import rospy
import actionlib
from geometry_msgs.msg import Pose


def goal_done( status, result ):
  rospy.loginfo( 'DONE status: %s (%s)', move_base_client.get_goal_status_text(), str( status ) )


if __name__ == '__main__':

  rospy.init_node( 'nav_stack_example' )

  move_base_client = actionlib.SimpleActionClient( '/move_base', MoveBaseAction )
  move_base_client.wait_for_server()

  goal_pose = Pose( position, orientation )
  move_base_goal = MoveBaseGoal()
  move_base_goal.target_pose.header.frame_id = 'map'
  move_base_goal.target_pose.header.stamp = rospy.Time.now()
  move_base_goal.target_pose.pose = goal_pose
  move_base_client.send_goal( move_base_goal, done_cb = goal_done )

  rospy.spin()


