#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class Movement:
    
    RATE_HZ = 5
    
    def __init__(self):
        rospy.init_node('movement', anonymous=True)
        self.max_v = 0.2 # [m/s]
        self.max_w = 1.0 # [rad/s]
        self.cmd_vel_mux_pub = rospy.Publisher('/yocs_cmd_vel_mux/input/navigation', Twist, queue_size = 10)
        self.odom_sub = rospy.Subscriber( '/odom', Odometry, self.odometry_cb )
        self.real_pose_sub = rospy.Subscriber('/real_pose', Pose, self.real_pose_cb)
        self.rate = rospy.Rate(self.RATE_HZ)
    
    def apply_velocity(self, lin_vel_list, ang_vel_list, time_list):
        args = zip(lin_vel_list, ang_vel_list, time_list)
        
        for lin_vel, ang_vel, time in args:
            initial_time = rospy.Time.now().to_sec()
            current_time = initial_time
            while current_time < initial_time + time:
                speed = Twist()
                speed.linear.x = lin_vel
                speed.angular.z = ang_vel
                # rospy.loginfo('Applied Speeds of (%f, %f)' % (lin_vel, ang_vel))
                self.cmd_vel_mux_pub.publish(speed)
                self.rate.sleep()
                current_time = rospy.Time.now().to_sec()
            
            
            # Stop the current movement
            speed = Twist()
            speed.linear.x = 0
            speed.angular.z = 0
            self.cmd_vel_mux_pub.publish(speed)
        
    
    def move_robot_to_destiny(self, goal_pose):
        pass
    
    def odometry_cb(self, odom):
        x = odom.pose.pose.position.x
        y = odom.pose.pose.position.y
        z = odom.pose.pose.position.z
        roll, pitch, yaw = euler_from_quaternion(( odom.pose.pose.orientation.x,
                                                    odom.pose.pose.orientation.y,
                                                    odom.pose.pose.orientation.z,
                                                    odom.pose.pose.orientation.w ) )
        rospy.loginfo(f'Current pose - lin: ({x}, {y}, {z}) ang: ({roll}, {pitch}, {yaw})')
    
    def real_pose_cb(self, odom):
        x = odom.position.x
        y = odom.position.y
        z = odom.position.z
        roll, pitch, yaw = euler_from_quaternion((  odom.orientation.x,
                                                    odom.orientation.y,
                                                    odom.orientation.z,
                                                    odom.orientation.w ) )
        rospy.loginfo(f'Real pose - lin: ({x}, {y}, {z}) ang: ({roll}, {pitch}, {yaw})')

if __name__ == '__main__':
    mov = Movement()
    mov.apply_velocity( 
        [0.5], # 0, 2, 4],
        [0], # 3.14159, 0, -0.2],   
        [5.1], # 1, 4, 3] 
    )