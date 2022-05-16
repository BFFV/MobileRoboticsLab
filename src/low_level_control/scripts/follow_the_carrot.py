#!/usr/bin/env python3

import numpy as np
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from pid_controller import PIDController
from tf.transformations import euler_from_quaternion


# Follow the Carrot
class FollowTheCarrot:
    def __init__(self):
        rospy.init_node('follow_the_carrot')
        self.variables_init()
        self.connections_init()

    # Initialize variables
    def variables_init(self):
        # TODO: actual Positions
        self.position = 0
        self.target = 0
        self.path = []
        # TODO: adjust
        self.look_ahead = 0.4

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

        # Pathline to follow
        rospy.Subscriber('/nav_plan', Path, self.start_tracking)

    # Obtain pathline to follow and start tracking
    def start_tracking(self, pathline):
        self.path = pathline.poses
        self.goal_angle = self.calculate_angle_diff(self.path[10])
        self.run()

    # Calculate angle difference between the robot and target
    def calculate_angle_diff(self, target):
        #TODO:
        return np.pi / 2

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

        # Update controller with current state
        self.angle_controller.pub_state(self.angle)

        # Update controller with current goal
        # TODO: Calculate angle to target
        self.angle_controller.pub_set_point(self.goal_angle)

    # Move the target along the pathline
    def move_target(self):
        # TODO: Calculate shortest point, then sum distances until look-ahead
        min_distance = float('inf')
        closest_point_idx = 0
        for idx, point in enumerate(self.path):
            # if distance < min_distance:
            # min_distance = distance
            # closest_point_idx = idx
            pass
        last_point = self.path[closest_point_idx]
        for point in self.path[closest_point_idx:]:
            #total_distance += distance(point, last_point)
            #if total_distance >= self.look_ahead:
                # self.target = point
                # return
            #last_point = point
            pass
        #self.target = point

    # Pathline tracking with low level control
    def run(self):
        while True:
            # Adjust orientation with controller
            self.speed_msg.angular.z = self.angle_controller.speed

            # Actuation
            self.cmd_vel_mux_pub.publish(self.speed_msg)

            # TODO: Move the target
            # TODO: Calculate shortest point, then sum distances until look-ahead
            #self.move_target()

            # Sleep
            self.rate_obj.sleep()

# Follow the Carrot pathline tracking
if __name__ == '__main__':
    follow_the_carrot = FollowTheCarrot()
    rospy.spin()
