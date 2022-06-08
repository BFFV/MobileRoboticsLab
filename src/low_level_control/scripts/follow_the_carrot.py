#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose, PoseArray, Twist
from math import atan2, sqrt
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

        # Plot real trajectory
        self.trajectory = rospy.Publisher('/trajectory', PoseArray, queue_size=1)

    # Obtain pathline to follow and start tracking
    def start_tracking(self, pathline):
        self.trajectory.publish(PoseArray(poses=[p.pose for p in pathline.poses]))
        self.path = [p.pose.position for p in pathline.poses]
        self.move_target()
        self.goal_angle = self.calculate_angle_diff()
        self.run()

    # Calculate distance between two points
    @staticmethod
    def distance(a, b):
        return sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2)

    # Calculate angle difference between the robot and target
    def calculate_angle_diff(self):
        return atan2(self.target.y - self.position.y, self.target.x - self.position.x)

    # Calculate Mean Squared Error for the trajectory
    def obtain_error(self):
        squared_error = 0
        for p in self.real_path:
            min_distance = float('inf')
            for point in self.path:
                distance = self.distance(p, point)
                if distance < min_distance:
                    min_distance = distance
            squared_error += min_distance ** 2
        print(f'MSE: {squared_error / len(self.real_path)}')

    # Update orientation dynamically
    def orientation(self, odom_data):
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

        # Save info
        self.real_path.append(pos.position)

        # Update controller with current state
        self.angle_controller.pub_state(self.angle)

        # Update controller with current goal
        if self.target:
            self.goal_angle = self.calculate_angle_diff()
            self.angle_controller.pub_set_point(self.goal_angle)

    # Move the target along the pathline
    def move_target(self):
        # Calculate closest point to the robot
        min_distance = float('inf')
        closest_point_idx = 0
        for idx, point in enumerate(self.path):
            distance = self.distance(self.position, point)
            if distance < min_distance:
                min_distance = distance
                closest_point_idx = idx

        # Find target
        last_point = self.path[closest_point_idx]
        total_distance = 0
        for point in self.path[closest_point_idx:]:
            total_distance += self.distance(last_point, point)
            if total_distance >= self.look_ahead:
                self.target = point
                return
            last_point = point
        self.target = point

    # Pathline tracking with low level control
    def run(self):
        # Stop when the goal is reached
        while self.distance(self.position, self.target) > 0.1:
            # Adjust orientation with controller
            self.speed_msg.angular.z = self.angle_controller.speed

            # Actuation
            self.cmd_vel_mux_pub.publish(self.speed_msg)

            # Move the target
            self.move_target()

            # Sleep
            self.rate_obj.sleep()

        # Calculate Mean Squared Error
        self.obtain_error()

# Follow the Carrot pathline tracking
if __name__ == '__main__':
    follow_the_carrot = FollowTheCarrot()
    rospy.spin()
