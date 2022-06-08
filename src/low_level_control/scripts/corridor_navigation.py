#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from pid_controller import PIDController
from std_msgs.msg import Float32MultiArray


# Corridor reactive navigation
class CorridorNavigator:
    RATE_HZ = 10
    MISSING_WALL_DISTANCE = 0.3

    def __init__(self):
        rospy.init_node('corridor_navigator')
        self.rate = rospy.Rate(self.RATE_HZ)
        self.cmd_vel_mux_pub = rospy.Publisher('/yocs_cmd_vel_mux/input/navigation', Twist, queue_size=10)
        self.angle_controller = PIDController('wall_follower')
        self.wall_distance_sub = rospy.Subscriber('/wall_distance', Float32MultiArray, self.wall_distance_cb)
        self.stop = False

    def run(self):
        self.angle_controller.pub_set_point(0)
        while not rospy.is_shutdown():
            self.apply_velocity(angular=self.angle_controller.speed)
            self.rate.sleep()

    def wall_distance_cb(self, distances):
        distances = [self.MISSING_WALL_DISTANCE if d == -1 else d for d in distances.data]
        distance_left, distance_right = distances
        if distance_left == distance_right == self.MISSING_WALL_DISTANCE:
            self.stop = True
            return
        self.stop = False
        diff = distance_right - distance_left
        self.angle_controller.pub_state(diff)

    def apply_velocity(self, linear=0.15, angular=0):
        speed_msg = Twist()
        speed_msg.linear.x = linear if not self.stop else 0
        speed_msg.angular.z = angular if not self.stop else 0
        self.cmd_vel_mux_pub.publish(speed_msg)

# Navigate corridor
if __name__ == "__main__":
    navigator = CorridorNavigator()
    rospy.sleep(0.4)
    navigator.run()
    rospy.spin()
