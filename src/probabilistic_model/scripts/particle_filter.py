#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose, PoseArray, Twist
from math import atan2, sqrt
from nav_msgs.msg import Odometry, Path
from tf.transformations import euler_from_quaternion


# Particle filter model for localization
class ParticleFilter:
    def __init__(self):
        rospy.init_node('particle_filter')
        self.variables_init()
        self.connections_init()

    # Initialize variables
    def variables_init(self):
        # Position
        self.position = None
        self.target = None

        # Orientation
        self.angle = 0.0
        self.goal_angle = 0.0

        # Path
        self.path = []
        self.real_path = []
        self.look_ahead = 0.3

        # Message frequency: 10Hz
        self.hz = 10
        self.rate_obj = rospy.Rate(self.hz)

        # Speed message for actuation
        self.speed_msg = Twist()
        self.speed_msg.linear.x = 0.1

    # Initialize connections
    def connections_init(self):
        # Turtlebot connections
        self.cmd_vel_mux_pub = rospy.Publisher('/yocs_cmd_vel_mux/input/navigation', Twist, queue_size=10)

        # Odometry
        rospy.Subscriber('/odom', Odometry, self.read_odometry)

        # Plot real trajectory
        self.trajectory = rospy.Publisher('/trajectory', PoseArray, queue_size=1)

    # Calculate distance between two points
    @staticmethod
    def distance(a, b):
        return sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2)

    # Calculate angle difference between the robot and target
    def calculate_angle_diff(self):
        return atan2(self.target.y - self.position.y, self.target.x - self.position.x)

    # Update orientation dynamically
    def read_odometry(self, odom_data):
        # Get odometry info
        pose_data = odom_data.pose.pose
        pos = Pose()
        pos.position.x = pose_data.position.x + 1
        pos.position.y = pose_data.position.y + 1
        self.position = pos.position
        quaternion = (pose_data.orientation.x,
                      pose_data.orientation.y,
                      pose_data.orientation.z,
                      pose_data.orientation.w)
        _, _, yaw = euler_from_quaternion(quaternion)
        self.angle = yaw

    # Main loop
    def run(self):
        pass

# Run particle filter
if __name__ == '__main__':
    particle_filter = ParticleFilter()
    rospy.spin()
