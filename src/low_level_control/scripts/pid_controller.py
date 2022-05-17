#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64

# PID Controller Wrapper
class PIDController:
    def __init__(self, topic='controller'):
        # Reference value
        self.set_point = rospy.Publisher(f'/{topic}/setpoint', Float64, queue_size=1)
        while not rospy.is_shutdown() and self.set_point.get_num_connections() == 0:
            rospy.sleep(0.2)

        # Estimated real value
        self.state = rospy.Publisher(f'/{topic}/state', Float64, queue_size=1)
        while not rospy.is_shutdown() and self.state.get_num_connections() == 0:
            rospy.sleep(0.2)

        # Actuation value
        rospy.Subscriber(f'/{topic}/control_effort', Float64, self.actuation)
        self.speed = 0
        rospy.loginfo(f'Ready {topic} connections!')

    # Publish reference
    def pub_set_point(self, set_point):
        self.set_point.publish(set_point)

    # Publish real value
    def pub_state(self, state):
        self.state.publish(state)

    # Receive actuation value
    def actuation(self, data):
        self.speed = float(data.data)
