#!/usr/bin/env python

import numpy as np
import rospy
from geometry_msgs.msg import Twist, Pose2D
from nav_msgs.msg import Odometry
from pid_controller import PIDController
from robot_pose import RobotPose
from std_msgs.msg import Float64
from tf.transformations import euler_from_quaternion

# Controlled movement
class Movement:
    def __init__(self):
        rospy.init_node('controlled_movement')
        self.variables_init()
        self.connection_init()

    def variables_init(self):
        self.is_mov = False

        # angle values
        self.ang = 0.0
        self.goal_ang = 0.0
        self.last_ang = 0.0
        self.delta_ang = 0.0

        # dist values
        self.dist = 0.0
        self.goal_dist = 0.0
        self.last_dist = Pose2D()
        self.last_dist.x = 0
        self.last_dist.y = 0

        self.hz = 10
        self.rate_obj = rospy.Rate(self.hz)

        self.speed_msg = Twist()

        self.topic_ang = 'robot_ang'
        self.topic_dist = 'robot_dist'

    def connection_init(self):
        # Turtle bot connection
        self.cmd_vel_mux_pub = rospy.Publisher('/yocs_cmd_vel_mux/input/navigation',
                                                Twist, queue_size=10)

        # Angle PID controller
        self.ang_PID_controller = PIDController(self.topic_ang)

        # set point angle
        rospy.Subscriber('/goal_ang', Float64, self.run_angle)

        # Dist PID Controller
        self.dist_PID_controller = PIDController(self.topic_dist)

        # set point distance
        rospy.Subscriber('/goal_dist', Pose2D, self.run_dist)

        # Odometry
        rospy.Subscriber('/odom', Odometry, self.set_odom)

    def giro_controlado(self, theta):
        twist_pub = rospy.Publisher('/goal_ang', Float64, queue_size=10)
        while twist_pub.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.sleep(0.2)
        msg = Float64()
        msg.data = theta
        twist_pub.publish(msg)

    def desplazamiento_controlado(self, dist):
        dist_pub = rospy.Publisher('/goal_dist', Pose2D, queue_size=10)
        while dist_pub.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.sleep( 0.2 )
        dist_pub.publish(dist)

    def set_odom(self, odom_data):
        pose_data = odom_data.pose.pose
        quaternion = (pose_data.orientation.x,
                    pose_data.orientation.y,
                    pose_data.orientation.z,
                    pose_data.orientation.w)
        _, _, yaw = euler_from_quaternion(quaternion)

        # angle definitions
        self.ang = yaw
        self.delta_ang = self.ang - self.last_ang
        self.last_ang = self.ang
        self.ang_PID_controller.pub_state(self.ang)

        # dist definitions
        x = odom_data.pose.pose.position.x
        y = odom_data.pose.pose.position.y
        self.dist = x + y
        self.last_dist.x = x
        self.last_dist.y = y
        self.dist_PID_controller.pub_state(self.dist)

    def mover_robot_a_destino_ctrl(self, goal_pose):
        self.movements = [RobotPose(*pose) for pose in goal_pose]
        for movement in self.movements:
            rospy.loginfo(movement)
            self.is_mov = True
            pose = Pose2D()
            pose.x = movement.x
            pose.y = movement.y
            self.desplazamiento_controlado(pose)
            while self.is_mov:
                self.rate_obj.sleep()
            rospy.loginfo("termino una distancia")
            self.is_mov = True
            self.giro_controlado(movement.theta)
            while self.is_mov:
                self.rate_obj.sleep()
            rospy.loginfo("termino un giro")
            rospy.loginfo("un movimiento completo")
        rospy.loginfo("Se terminaron todos los movimientos")

    # angle application
    def run_angle(self, goal_ang):
        self.goal_ang = goal_ang.data * (np.pi/180)
        self.ang_PID_controller.pub_set_point(self.goal_ang)

        while abs(self.goal_ang - self.ang) > np.pi/120 or self.delta_ang > np.pi/180:
            self.speed_msg.angular.z = self.ang_PID_controller.speed
            self.cmd_vel_mux_pub.publish(self.speed_msg)
            self.rate_obj.sleep()

        self.speed_msg.angular.z = 0
        self.cmd_vel_mux_pub.publish(self.speed_msg)

        self.ang_PID_controller.pub_state(0)
        self.is_mov = False

    # distance application
    def run_dist(self, goal_dist):
        self.goal_dist = goal_dist.x + goal_dist.y

        self.dist_PID_controller.pub_set_point(self.goal_dist)

        while abs(self.goal_dist - self.dist) > 0.05:
            self.speed_msg.linear.x = abs(self.dist_PID_controller.speed)
            self.cmd_vel_mux_pub.publish(self.speed_msg)
            self.rate_obj.sleep()

        self.speed_msg.linear.x = 0
        self.cmd_vel_mux_pub.publish(self.speed_msg)

        self.dist = 0
        self.dist_PID_controller.pub_state(0)
        self.is_mov = False

# Controlled movement
if __name__ == '__main__':
    movement = [(1,0, 90), (1,1,180), (0,1,-90), (0,0,0)]
    controller = Movement()
    rospy.sleep(3)
    controller.mover_robot_a_destino_ctrl(movement)
    rospy.spin()
