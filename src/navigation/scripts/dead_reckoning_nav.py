#!/usr/bin/env python
from time import time
from copy import copy
from math import atan2, pi, sin, cos

import rospy
from geometry_msgs.msg import Twist, Pose, PoseArray
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion



def vector_norm(x, y):
    return (x ** 2 + y ** 2) ** 0.5

def dot_product(x0, y0, x1, y1):
    return x0 * x1 + y0 * y1

def vector_angle(x0, y0, x1, y1):
    dot = dot_product(x0, y0, x1, y1)
    det = x0 * y1 - y0 * x1
    return atan2(det, dot) 
     


class RobotPose:
    """Represents a pose and provides operations between them"""
    
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta
        
    def linear_distance_to(self, other):
        return vector_norm(other.x - self.x, other.y - self.y)
        

    def angular_distance_to(self, other):
        return vector_angle(self.x, self.y, other.x, other.y)
        
        
    def __add__(self, other):
        return RobotPose(
            self.x + other.x, 
            self.y + other.y, 
            self.theta + other.theta
        )
    
    def __sub__(self, other):
        return RobotPose(
            self.x - other.x, 
            self.y - other.y, 
            self.theta - other.theta
        )
        
    def __repr__(self):
        return f"<{self.x}, {self.y}, {self.theta}>"
        

class DeadReckoningNav:
    
    RATE_HZ = 60
    ROTATION_CORRECTION_FACTOR = 1.09845578 # Found Experimentally
    
    
    def __init__(self):
        rospy.init_node('movement', anonymous=True)
        self.max_v = 0.2 # [m/s]
        self.max_w = 1 * self.ROTATION_CORRECTION_FACTOR # [rad/s]
        self.cmd_vel_mux_pub = rospy.Publisher('/yocs_cmd_vel_mux/input/navigation', 
                                               Twist, queue_size = 10)
        self.position_diff_pub = rospy.Publisher('/position_diff', 
                                                 PoseArray, queue_size=10)
        self.odom_sub = rospy.Subscriber( '/odom', Odometry, self.odometry_cb )
        self.real_pose_sub = rospy.Subscriber('/real_pose', Pose, self.real_pose_cb)
        self.rate = rospy.Rate(self.RATE_HZ)
        self.pose_sub = None
        self.initial_pose = RobotPose(1, 1, 0)
        
        # Attrs for correctly reporting the current position
        self.last_real_pose = None
        self.last_real_pose_time = None
        self.last_odom_pose = None
        self.last_odom_pose_time = None
    
    def apply_velocity(self, lin_vel_list, ang_vel_list, time_list):
        """Applies velocities secuentally. Note that arguments are split 
           in different lists"""
        
        args = zip(lin_vel_list, ang_vel_list, time_list)
        
        for lin_vel, ang_vel, time in args:
            initial_time = rospy.Time.now().to_sec()
            current_time = initial_time
            
            # Loop in short steps to prevent velocity limit
            while current_time < initial_time + time:
                speed = Twist()
                speed.linear.x = lin_vel
                speed.angular.z = ang_vel 
                self.cmd_vel_mux_pub.publish(speed)
                self.rate.sleep()
                current_time = rospy.Time.now().to_sec()
            
            
            # Stop the current movement
            speed = Twist()
            speed.linear.x = 0
            speed.angular.z = 0
            self.cmd_vel_mux_pub.publish(speed)
    
    def apply_velocity_tuples(self, *vel_tuples):
        """ Frontend for apply_velocities so we can pass just tuples of
            velocities components"""
        self.apply_velocity(*zip(*vel_tuples))
            
    
    def move_robot_to_destiny(self, goal_pose_list):
        """Moves robot to all poses in the list secuentally"""
        
        # TODO: Refactor
        
        poses = [ RobotPose(*pose) for pose in goal_pose_list ] + [None]
        
        current_pose = self.initial_pose
        pose_iter = iter(poses)
        next_pose = next(pose_iter)  
        
        velocities = []
        
        while next_pose:
            rospy.loginfo(f"Moving from {current_pose} to {next_pose}")
            
            # We rotate until we are pointing to the next pose
            diff_pose = next_pose - current_pose
            fake_pos = RobotPose(cos(current_pose.theta), sin(current_pose.theta), 0)
            angle_diff = fake_pos.angular_distance_to(diff_pose)
            w_time = abs(angle_diff / self.max_w)
            sign = 1 if angle_diff >= 0 else -1    
            velocities.append((0, sign * self.max_w, w_time))
            
            
            # We go forward
            distance = current_pose.linear_distance_to(next_pose)
            d_time = distance / self.max_v
            velocities.append((self.max_v, 0, d_time))
            
            # We set the target orientation
            current_theta = current_pose.theta + angle_diff
            final_fake = RobotPose(cos(current_theta), sin(current_theta), 0)
            new_fake = RobotPose(cos(next_pose.theta), sin(next_pose.theta), 0)
            angle_diff = final_fake.angular_distance_to(new_fake)
            w_time = abs(angle_diff / self.max_w) 
            sign = 1 if angle_diff >= 0 else -1
            velocities.append((0, sign * self.max_w, w_time))
                    
            current_pose = next_pose
            next_pose = next(pose_iter)
            
        self.apply_velocity(*zip(*velocities))
              
    
    def odometry_cb(self, odom):
        x = odom.pose.pose.position.x
        y = odom.pose.pose.position.y
        orientation = odom.pose.pose.orientation
        _, _, yaw = euler_from_quaternion(( orientation.x,
                                            orientation.y,
                                            orientation.z,
                                            orientation.w))
        
        self.last_odom_pose = self.initial_pose + RobotPose(x, y, yaw)
        self.last_odom_pose_time = time()
        
    def real_pose_cb(self, odom):
        x = odom.position.x
        y = odom.position.y
        z = odom.position.z
        roll, pitch, yaw = euler_from_quaternion((  odom.orientation.x,
                                                    odom.orientation.y,
                                                    odom.orientation.z,
                                                    odom.orientation.w ) )
        
        self.last_real_pose = RobotPose(x, y, yaw)
        self.last_real_pose_time = time()
        
        
    def goal_pose_cb(self, pose_array):
        def transform(pose):
            x = pose.position.x
            y = pose.position.y
            w = pose.orientation.w
            return (x, y, w)
            
        pose_list = [ transform(pose) for pose in pose_array.poses ]
        
        self.move_robot_to_destiny(pose_list)
        
        
    def listen_and_spin(self):
        self.pose_sub = rospy.Subscriber('/goal_list', PoseArray, self.goal_pose_cb)
        rospy.spin()
        
    def wait_and_publish_position(self):
        """ For debugging purpouses. Waits for an odometry and real pose at same
            time and sends it to position diff writer"""
            
        while True:
            
            if not self.last_odom_pose_time: 
                self.rate.sleep()
                continue
                    
            time_diff = abs(self.last_odom_pose_time - self.last_real_pose_time)
            if time_diff > 0.01: # They are too far away
                self.rate.sleep()
                continue
            
            real_pose = Pose()
            real_pose.position.x = self.last_real_pose.x
            real_pose.position.y = self.last_real_pose.y
            real_pose.orientation.w = self.last_real_pose.theta
            
            odom_pose = Pose()
            odom_pose.position.x = self.last_odom_pose.x
            odom_pose.position.y = self.last_odom_pose.y
            odom_pose.orientation.w = self.last_odom_pose.theta
            
            poses = [real_pose, odom_pose]
            
            self.position_diff_pub.publish(PoseArray(poses=poses))
            return
        

if __name__ == '__main__':
    nav = DeadReckoningNav()
    # Prevent missing log messages by waiting a small amount
    rospy.sleep(0.3)
    nav.listen_and_spin()