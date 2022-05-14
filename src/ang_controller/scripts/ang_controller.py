#!/usr/bin/env python
import rospy
import numpy as np
from pid_controller import PIDController
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from tf.transformations import euler_from_quaternion


class AngController(object):

  def __init__(self) -> None:
    rospy.init_node('ang_controller')
    self.variables_init()
    self.connection_init()

  def variables_init(self):

    self.ang = 0.0
    self.goal_ang = 0.0

    self.last_ang = 0.0
    self.delta_ang = 0.0
    
    self.hz = 10
    self.rate_obj = rospy.Rate(self.hz)

    self.speed_msg = Twist()

    self.topic = "robot_ang"

  def connection_init(self):

    # Turtle bot connection
    self.cmd_vel_mux_pub = rospy.Publisher('/yocs_cmd_vel_mux/input/navigation',
                                               Twist, queue_size=10)

    # Angle PID controller
    self.ang_PID_controller = PIDController(self.topic)
    # set point
    rospy.Subscriber('/goal_ang', Float64, self.run)

    # Odometry
    rospy.Subscriber('/odom', Odometry, self.set_odom)
    


  def set_odom(self, odom_data):
    pose_data = odom_data.pose.pose

    quaternion = (pose_data.orientation.x,
                  pose_data.orientation.y,
                  pose_data.orientation.z,
                  pose_data.orientation.w)

    row, pitch, yaw = euler_from_quaternion(quaternion)

    self.ang = yaw
    
    self.delta_ang = self.ang - self.last_ang
    self.last_ang = self.ang

    self.ang_PID_controller.pub_state(self.ang)


  def run(self, goal_ang):
    rospy.loginfo(goal_ang.data)
    self.goal_ang = goal_ang.data*(np.pi/180)
    self.ang_PID_controller.pub_set_point(self.goal_ang)

    while abs(self.goal_ang - self.ang) > np.pi/120 or self.delta_ang > np.pi/180:
      self.speed_msg.angular.z = self.ang_PID_controller.speed
      self.cmd_vel_mux_pub.publish(self.speed_msg)
      self.rate_obj.sleep()

    rospy.loginfo("ready")


controller = AngController()
rospy.spin()

