#!/usr/bin/env python

import numpy as np
import rospy
from geometry_msgs.msg import Point32, Pose
from sensor_msgs.msg import LaserScan, PointCloud


# Obtain laser scan points
class Scan2PointCloud():
    def __init__(self):
        rospy.init_node('scan2pointcloud')
        self.variables_init()
        self.connections_init()
        rospy.spin()

    # Initialize variables
    def variables_init(self):
        # Map features
        self.map_height = 270
        self.map_width = 270
        self.resolution = 0.01

        # Robot pose
        self.robot_x_pix = 0.0
        self.robot_y_pix = 0.0
        self.robot_ang = 0.0

    # Initialize connections
    def connections_init(self):
        self.pub_pcl = rospy.Publisher('/lidar_points', PointCloud, queue_size=5)
        rospy.Subscriber('/scan', LaserScan, self.laser_scan_hd, queue_size=1)

    # Set laser measurements
    def laser_scan_hd(self, scan):
        self.update_odom_pix()
        num_angles = int((scan.angle_max - scan.angle_min) / scan.angle_increment)
        angles = np.linspace(scan.angle_min, scan.angle_max, num_angles)
        point_cloud = PointCloud()
        for z_ang, zk in zip(angles, scan.ranges):
            zx_pix = zk / self.resolution
            global_ang = -(z_ang + self.robot_ang)
            px_pix = int(zx_pix * np.cos(global_ang) + self.robot_x_pix)
            py_pix = int(zx_pix * np.sin(global_ang) + self.robot_y_pix)
            px_in_range = 0 < py_pix < self.map_height
            py_in_range = 0 < px_pix < self.map_width
            if px_in_range and py_in_range and zk < scan.range_max:
                point = Point32()
                point.x, point.y = px_pix, py_pix
                point_cloud.points.append(point)
        self.pub_pcl.publish(point_cloud)

    # Update position with odometry
    def update_odom_pix(self):
        odom_pix_data = rospy.wait_for_message('/odom_pix', Pose, timeout=3)
        self.robot_x_pix = int(odom_pix_data.position.x)
        self.robot_y_pix = int(odom_pix_data.position.y)
        self.robot_ang = odom_pix_data.orientation.z


# Obtain laser measurements
if __name__ == '__main__':
    scan2cloud = Scan2PointCloud()
