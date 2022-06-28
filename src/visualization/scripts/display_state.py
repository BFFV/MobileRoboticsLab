#!/usr/bin/env python

import cv2
import numpy as np
import rospy
from cmocean import cm
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
        self.fresh_map = None
        self.location = None
        self.location_pix = None
        self.resolution = 0.01

    # Initialize connections
    def connections_init(self):
        self.pub_map = rospy.Publisher('/img_map', Image, queue_size=10)
        rospy.sleep(1)
        rospy.Subscriber('/map', OccupancyGrid, self.set_map)
        rospy.Subscriber('/lidar_points', PointCloud, self.show_pointcloud)
        rospy.Subscriber('/location', Pose, self.set_location)
        rospy.Subscriber('/show_map', Empty, self.show_map)
        rospy.Subscriber('/particles', PoseArray, self.set_particles)
        rospy.Subscriber('/real_pose_pix', Pose, self.draw_map.set_real_pose)

    # Create map image
    def set_map(self, map):
        width = map.info.width
        height = map.info.height
        np_map = np.array(map.data)
        np_map = np_map.reshape((height, width))
        map_img = 100 - np_map
        map_img = (map_img * (255 / 100.0)).astype(np.uint8)
        self.map = cv2.cvtColor(map_img, cv2.COLOR_GRAY2RGB)
        self.fresh_map = self.map.copy()

    # Show laser points in the map
    def show_pointcloud(self, pointcloud):
        points = pointcloud.points
        map_copy = self.map.copy()
        map_copy = self.draw_map.draw_robot(map_copy)
        if self.location:
            self.draw_map.draw_location(map_copy, self.location, self.location_pix)
        #map_copy = self.draw_map.draw_point_cloud(map_copy, points)
        img_msg = self.bridge.cv2_to_imgmsg(map_copy, 'bgr8')
        self.pub_map.publish(img_msg)

    # Show map
    def show_map(self):
        map_copy = self.map.copy()
        map_copy = self.draw_map.draw_robot(map_copy)
        img_msg = self.bridge.cv2_to_imgmsg(map_copy, 'bgr8')
        self.pub_map.publish(img_msg)

    # Set final location
    def set_location(self, location):
        self.location_pix = (int(location.position.x), int(location.position.y))
        self.location = (round(location.position.x * self.resolution, 2),
                         round((270 - location.position.y) * self.resolution, 2))

    # Show particles in the map
    def set_particles(self, particle_data):
        particles = particle_data.poses
        map_copy = self.fresh_map.copy()
        self.draw_map.draw_particles(map_copy, particles)
        self.map = map_copy


# Draw state
class DrawState():
    def __init__(self):
        # Colors
        self.robot_color = (0, 0, 255)     # red
        self.points_color = (0, 255, 0)    # green
        self.location_color = (255, 0, 0)  # blue
        self.line_color = (255, 255, 255)  # white
        self.text_color = (0, 0, 0)        # black
        self.robot_alpha = 0.4             # robot transparency
        self.laser_alpha = 0.6             # laser transparency

        # Robot
        self.robot_radius = 0.18  # meters
        self.resolution = 0.01

        # Points
        self.points_radius = 0.02
        self.particles_radius = 0.01
        self.location_radius = 0.05
        self.points_radius_pix = int(self.points_radius / self.resolution)
        self.particles_radius_pix = int(self.particles_radius / self.resolution)
        self.location_radius_pix = int(self.location_radius / self.resolution)

        # Pose
        self.robot_x_pix = 0
        self.robot_y_pix = 0
        self.robot_ang = 0

    # Set real pose of the robot
    def set_real_pose(self, pose):
        self.robot_x_pix = int(pose.position.x)
        self.robot_y_pix = int(pose.position.y)
        self.robot_ang = pose.orientation.z

    # Draw robot
    def draw_robot(self, map):
        robot_radius_pix = int(self.robot_radius / self.resolution)

        # Get front of the robot
        head_x = int(robot_radius_pix * np.cos(-self.robot_ang) + self.robot_x_pix)
        head_y = int(robot_radius_pix * np.sin(-self.robot_ang) + self.robot_y_pix)

        # Draw robot
        robot_pose_pix = tuple([self.robot_x_pix, self.robot_y_pix])
        head_pose = tuple([head_x, head_y])
        aux_map = map.copy()
        cv2.circle(aux_map, robot_pose_pix, robot_radius_pix, self.robot_color, -1)
        cv2.line(aux_map, robot_pose_pix, head_pose, self.line_color, 2)
        return cv2.addWeighted(aux_map, self.robot_alpha, map, 1 - self.robot_alpha, 0)

    # Draw laser points
    def draw_point_cloud(self, map, points):
        aux_map = map.copy()
        for point in points:
            x, y = int(point.x), int(point.y)
            cv2.circle(aux_map, (x, y), self.points_radius_pix, self.points_color, -1)
        return cv2.addWeighted(aux_map, self.laser_alpha, map, 1 - self.laser_alpha, 0)

    # Draw current location
    def draw_location(self, map, pose, pose_pix):
        x, y = pose
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(map, 'Localization Done', (102, 30), font, 0.6, self.text_color, 1, cv2.LINE_AA)
        cv2.putText(map, f'({x}, {y})', (110, 60), font, 0.6, self.text_color, 1, cv2.LINE_AA)
        cv2.circle(map, (pose_pix[0], pose_pix[1]), self.location_radius_pix, self.location_color, thickness=2)

    # Draw current particles
    def draw_particles(self, map, particles):
        for particle in particles:
            x, y = int(particle.position.x), int(particle.position.y)
            angle = particle.orientation.z / (2 * np.pi)
            angle_color = tuple(reversed(tuple(c * 255 for c in cm.phase(angle))[:3]))
            cv2.circle(map, (x, y), self.particles_radius_pix, angle_color, -1)


# Display current state
if __name__ == '__main__':
    display_map = DisplayState()
