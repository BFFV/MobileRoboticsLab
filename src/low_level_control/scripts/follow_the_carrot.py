#!/usr/bin/env python3

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from pid_controller import PIDController
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from tf.transformations import euler_from_quaternion


# Follow the Carrot
class FollowTheCarrot:
    def __init__(self):
        rospy.init_node('follow_the_carrot')
        self.variables_init()
        self.connections_init()
        rospy.spin()

    def variables_init(self):
        # value and goal goal value
        self.value = 0.0
        self.goal_value = 141.0

        # last orientation and delta ang
        self.last_value = 0.0
        self.delta_value = 0.0
        # message sending frequency: 10Hz
        self.hz = 10
        self.rate_obj = rospy.Rate(self.hz)
        # msg type: Twist
        self.speed_msg = Twist()
        # line speed
        self.speed_msg.linear.x = 0.1
        # controller topic, in launch "ns = topic"
        self.topic = "robot_ang"
        self.bridge = CvBridge()

    def connections_init(self):
        # Turtlebot connections
        self.cmd_vel_mux_pub = rospy.Publisher('/yocs_cmd_vel_mux/input/navigation', Twist, queue_size=10)

        #  angle PID controller
        self.ang_PID_controller = PIDController(self.topic)

        # set point
        rospy.Subscriber('/goal_value', Float64, self.run)

    def run(self, goal):
        self.goal_value = goal.data
        self.ang_PID_controller.pub_set_point(self.goal_value)
        while True:
            self.speed_msg.angular.z = self.ang_PID_controller.speed
            self.cmd_vel_mux_pub.publish(self.speed_msg)
            self.rate_obj.sleep()

# Follow the Carrot
if __name__ == '__main__':
  follow_the_carrot = FollowTheCarrot()
  rospy.spin()
