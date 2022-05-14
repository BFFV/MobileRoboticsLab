#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64

class PIDController:
    def __init__( self, topic:str = "controller"):
        self.set_point = rospy.Publisher( f'/{topic}/setpoint', Float64, queue_size = 1 )
        while self.set_point.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.sleep(0.2)

        self.state = rospy.Publisher( f'/{topic}/state', Float64, queue_size = 1 )
        while self.state.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.sleep(0.2)

        rospy.Subscriber(f'/{topic}/control_effort', Float64, self.actuation)

        self.speed = 0

        rospy.loginfo( f"ready {topic} connections")

    def pub_set_point( self, set_point ):
        self.set_point.publish( set_point )

    def pub_state( self, state ):
        self.state.publish( state )

    def actuation( self, data ):
        self.speed = float( data.data )
        # rospy.loginfo( 'speed received: %f' % ( self.speed ))
