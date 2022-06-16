#!/usr/bin/env python

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, PoseArray
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud, Image
from std_msgs.msg import Empty


# Display state
class DisplayState():
    def __init__(self):
        rospy.init_node('display_state')
        self.variables_init()
        self.connections_init()
        rospy.spin()

    # Initialize variables
    def variables_init(self):
        self.draw_map = DrawState()
        self.bridge = CvBridge()
        self.map = None
        self.point_cloud = None

    # Initialize connections
    def connections_init(self):
        self.pub_map = rospy.Publisher('/img_map', Image, queue_size=10)
        rospy.Subscriber('/map', OccupancyGrid, self.set_map)
        rospy.Subscriber('/lidar_points', PointCloud, self.show_pointcloud)
        rospy.Subscriber('/location', Pose, self.set_location)
        rospy.Subscriber('/show_map', Empty, self.show_map)
        rospy.Subscriber('/particles', PoseArray, self.show_particles)

    # Create map image
    def set_map(self, map):
        width = map.info.width
        height = map.info.height
        np_map = np.array(map.data)
        np_map = np_map.reshape((height, width))
        map_img = 100 - np_map
        map_img = (map_img * (255 / 100.0)).astype(np.uint8)
        self.map = cv2.cvtColor(map_img, cv2.COLOR_GRAY2RGB)

    # Show laser points in the map
    def show_pointcloud(self, pointcloud):
        points = pointcloud.points
        map_copy = self.map.copy()
        self.draw_map.update_odom_pix()
        self.draw_map.draw_robot(map_copy)
        self.draw_map.draw_point_cloud(map_copy, points)
        img_msg = self.bridge.cv2_to_imgmsg(map_copy, 'bgr8')
        self.pub_map.publish(img_msg)

    # Show map
    def show_map(self):
        map_copy = self.map.copy()
        self.draw_map.update_odom_pix()
        self.draw_map.draw_robot(map_copy)
        if self.location:
            self.draw_map.draw_location(map_copy, self.location)
        img_msg = self.bridge.cv2_to_imgmsg(map_copy, 'bgr8')
        self.pub_map.publish(img_msg)

    # Set final location
    def set_location(self, location):
        map_copy = self.map.copy()
        self.draw_map.draw_location(map_copy, location)
        self.map = map_copy

    # Show particles in the map
    def show_particles(self, particle_data):
        particles = particle_data.poses
        map_copy = self.map.copy()
        self.draw_map.draw_particles(map_copy, particles)
        self.map = map_copy


# Draw state
class DrawState():
    def __init__(self):
        # Colors
        self.robot_color = (0, 0, 255)     # red
        self.points_color = (0, 255, 0)    # green
        self.line_color = (255, 255, 255)  # white
        self.text_color = (0, 0, 0)        # black

        # Robot
        self.robot_radius = 0.18  # meters
        self.points_radius = 0.02
        self.resolution = 0.01

        # Pose
        self.robot_x_pix = 0
        self.robot_y_pix = 0
        self.robot_ang = 0

    # Update robot position
    def update_odom_pix(self):
        odom_pix_data = rospy.wait_for_message('/odom_pix', Pose, timeout=3)
        self.robot_x_pix = int(odom_pix_data.position.x)
        self.robot_y_pix = int(odom_pix_data.position.y)
        self.robot_ang = odom_pix_data.orientation.z

    # Draw robot
    def draw_robot(self, map):
        robot_radius_pix = int(self.robot_radius / self.resolution)

        # Get front of the robot
        head_x = int(robot_radius_pix * np.cos(-self.robot_ang) + self.robot_x_pix)
        head_y = int(robot_radius_pix * np.sin(-self.robot_ang) + self.robot_y_pix)

        # Draw robot
        robot_pose_pix = tuple([self.robot_x_pix, self.robot_y_pix])
        head_pose = tuple([head_x, head_y])
        cv2.circle(map, robot_pose_pix, robot_radius_pix, self.robot_color, -1)
        cv2.line(map, robot_pose_pix, head_pose, self.line_color, 2)

    # Draw laser points
    def draw_point_cloud(self, map, points):
        point_radius_pix = int(self.points_radius / self.resolution)
        for point in points:
            x, y = int(point.x), int(point.y)
            cv2.circle(map, (x, y), point_radius_pix, self.points_color, -1)

    # Draw current location
    def draw_location(self, map, pose):
        x = int(pose.position.x)
        y = int(pose.position.y)
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(map, 'Localization Done', (102, 30), font, 0.6, self.text_color, 1, cv2.LINE_AA)
        cv2.putText(map, f'({x}, {y})', (110, 60), font, 0.6, self.text_color, 1, cv2.LINE_AA)

    # Draw current particles
    # TODO: Draw particles in the image, each with a color that represents the angle of it's pose (in some color scale)
    def draw_particles(self, map, particles):
        pass


# Display current state
if __name__ == '__main__':
    display_map = DisplayState()
