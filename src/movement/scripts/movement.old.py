#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

class MoveInCircles:
  def __init__( self ):
    self.max_w = 1.0 # [rad/s]
    self.max_v = 0.2 # [m/s]
    rospy.init_node( 'move_in_circles_node' )
    self.cmd_vel_mux_pub = rospy.Publisher( '/yocs_cmd_vel_mux/input/navigation', Twist, queue_size = 10 )
    self.rate_hz = 10
    self.rate_obj = rospy.Rate( self.rate_hz )

  def move( self, lin_speed, ang_speed ):
    speed = Twist()
    speed.linear.x = lin_speed
    speed.angular.z = ang_speed
    while not rospy.is_shutdown():
      rospy.loginfo( 'publishing speeds (%f, %f)' % (lin_speed, ang_speed) )
      self.cmd_vel_mux_pub.publish( speed )
      self.rate_obj.sleep()
      

if __name__ == '__main__':
  mic = MoveInCircles()
  mic.move( 0.5, 1.0 )