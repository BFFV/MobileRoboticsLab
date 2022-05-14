#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64

class PIDController( object ):

  def __init__(self, topic:str = "controller"):
    # r(t) Estado objetivo
    self.set_point = rospy.Publisher( f'/{topic}/setpoint', Float64, queue_size = 1 )
    while self.set_point.get_num_connections() == 0 and not rospy.is_shutdown():
      rospy.sleep( 0.2 )

    # y(t) Estado actual
    self.state = rospy.Publisher( f'/{topic}/state', Float64, queue_size = 1 )
    while self.state.get_num_connections() == 0 and not rospy.is_shutdown():
      rospy.sleep( 0.2 )

    # u(t) Actuación del nodo PID
    rospy.Subscriber( f'/{topic}/control_effort', Float64, self.actuation )

    self.speed = 0
    
    rospy.loginfo(f"ready {topic} connections")

  def pub_set_point(self, set_point):
    msg = Float64()
    msg.data = set_point
    self.set_point.publish(msg)

  def pub_state(self, state):
    msg = Float64()
    msg.data = state
    self.state.publish(msg.data)

  def actuation(self, data):
    self.speed = float(data.data)
    
