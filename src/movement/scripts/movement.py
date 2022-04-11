#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

from math import acos


def vector_norm(x, y):
    return (x ** 2 + y ** 2) ** 0.5

def dot_product(x0, y0, x1, y1):
    return x0 * x1 + y0 * y1

def vector_angle(x0, y0, x1, y1):
    denom = vector_norm(x0, y0) * vector_norm(x1, y1)
    return acos(dot_product(x0, y0, x1, y1) / denom)
     


class RobotPose:
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta
        
    def linear_distance_to(self, other):
        return vector_norm(other.x - self.x, other.y - self.y)
        

    def angular_distance_to(self, other):
        return vector_angle(other.x, other.y, self.x, self.y)
        
        
    def __add__(self, other):
        self.x += other.x
        self.y += other.y
        self.theta += other.theta
    
    def __sub__(self, other):
        self.x -= other.x
        self.y -= other.y
        self.theta -= other.theta
        
    def __repr__(self):
        return f"<{self.x}, {self.y}, {self.theta}>"
        

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
        self.initial_pose = RobotPose(1, 1, 0)
        self.current_pose = self.initial_pose
    
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
        
    
    def move_robot_to_destiny(self, goal_pose_list):
        poses = [ RobotPose(*pose) for pose in goal_pose_list ] + [None]
        rospy.loginfo(f"Current pos list {poses}")
         
        # TODO:s
        # - Point to next point
        # - Go forwards
        # - Set orientation to target 
        
        
        pose_iter = iter(poses)
        current_pose = self.current_pose
        next_pose = next(pose_iter)  
        
        velocities = []
        
        while next_pose:

            # Align rotation
            rospy.loginfo(f"Moving from {current_pose} to {next_pose}")
            
            diff_pose = next_pose - current_pose
            fake_pos = RobotPose(1, 0, 0)
            
            
            final_angle = fake_pos.angular_distance_to(diff_pose)
            rospy.loginfo(f"The angle difference is {final_angle}")
            
            
            # angle_diff = current_pose.angular_distance_to(next_pose)
            # distance = current_pose.linear_distance_to(next_pose)
            
            # w_time = angle_diff / self.max_w 
            # sign = 1 if angle_diff > 0 else -1
            # velocities.append((0, sign * self.max_w, w_time))
                    
            # dist_time = distance / self.max_v
            # velocities.append((self.max_v, 0, dist_time))
                    
            current_pose = next_pose
            next_pose = next(pose_iter)
            
        # self.apply_velocity(*zip(*velocities))
              
    
    def odometry_cb(self, odom):
        x = odom.pose.pose.position.x
        y = odom.pose.pose.position.y
        orientation = odom.pose.pose.orientation
        _, _, yaw = euler_from_quaternion(( orientation.x,
                                            orientation.y,
                                            orientation.z,
                                            orientation.w))
        
        self.current_pose = self.initial_pose + RobotPose(x, y, yaw)
        rospy.logdebug(f'Current pose - lin: ({x}, {y}) ang: ({yaw})')
    
    def real_pose_cb(self, odom):
        x = odom.position.x
        y = odom.position.y
        z = odom.position.z
        roll, pitch, yaw = euler_from_quaternion((  odom.orientation.x,
                                                    odom.orientation.y,
                                                    odom.orientation.z,
                                                    odom.orientation.w ) )
        rospy.logdebug(f'Real pose - lin: ({x}, {y}, {z}) ang: ({roll}, {pitch}, {yaw})')

if __name__ == '__main__':
    mov = Movement()
    mov.move_robot_to_destiny([[0, 0, 1], [2, 1, 4]])
    rospy.spin()  
    # mov.apply_velocity( SS
    #     [0.5], # 0, 2, 4],
    #     [0], # 3.14159, 0, -0.2],   
    #     [5.1], # 1, 4, 3] 
    # )