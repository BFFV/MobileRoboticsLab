#!/usr/bin/env python3

import numpy as np
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from pid_controller import PIDController
from tf.transformations import euler_from_quaternion


# Follow the Carrot
class FollowTheCarrot:
    def __init__(self):
        rospy.init_node('follow_the_carrot')
        self.variables_init()
        self.connections_init()
        self.run()

    # Initialize variables
    def variables_init(self):
        # Orientation
        self.angle = 0.0
        self.goal_angle = 0.0

        # Message frequency: 10Hz
        self.hz = 10
        self.rate_obj = rospy.Rate(self.hz)

        # Speed message for actuation
        self.speed_msg = Twist()
        self.speed_msg.linear.x = 0.1

        # Controller topic, namespace in launch file
        self.topic = 'pathline_follower'

    # Initialize connections
    def connections_init(self):
        # Turtlebot connections
        self.cmd_vel_mux_pub = rospy.Publisher('/yocs_cmd_vel_mux/input/navigation', Twist, queue_size=10)

        # Angle PID controller
        self.angle_controller = PIDController(self.topic)

        # Odometry
        rospy.Subscriber('/odom', Odometry, self.orientation)

    # Update orientation dynamically
    def orientation(self, odom_data):
        # Get odometry orientation info
        pose_data = odom_data.pose.pose
        quaternion = (pose_data.orientation.x,
                      pose_data.orientation.y,
                      pose_data.orientation.z,
                      pose_data.orientation.w)
        _, _, yaw = euler_from_quaternion(quaternion)
        self.angle = yaw

        # Map angles to [0,2*pi)
        if self.angle < 0:
            self.angle += 2 * np.pi

        # Update controller with current state
        self.angle_controller.pub_state(self.angle)

    # Pathline tracking with low level control
    def run(self):
        while True:
            # TODO: update goal
            if self.goal_angle != np.pi:
                self.goal_angle = np.pi
                self.angle_controller.pub_set_point(self.goal_angle)

            # Adjust orientation with controller
            self.speed_msg.angular.z = self.angle_controller.speed

            # Actuation
            self.cmd_vel_mux_pub.publish(self.speed_msg)
            self.rate_obj.sleep()

# Follow the Carrot pathline tracking
if __name__ == '__main__':
    follow_the_carrot = FollowTheCarrot()
