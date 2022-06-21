#!/usr/bin/env python

import random
from turtle import position
import numpy as np
import pandas as pd
import rospy
from geometry_msgs.msg import Pose, PoseArray, Twist, Point32
from nav_msgs.msg import OccupancyGrid
from scipy import spatial
from sensor_msgs.msg import LaserScan, PointCloud


# Particle filter model for localization
class ParticleFilter:
    def __init__(self):
        rospy.init_node('particle_filter')
        self.variables_init()
        self.connections_init()
        rospy.sleep(3)
        while not self.distance_tree:
            rospy.sleep(1)
        self.run()
        rospy.spin()

    # Initialize variables
    def variables_init(self):
        # Message frequency: 10Hz
        self.hz = 10
        self.rate_obj = rospy.Rate(self.hz)

        # Speed message for actuation
        self.linear_speed = 0.0
        self.angular_speed = 0.0
        self.speed_msg = Twist()

        # Obstacles map
        self.map_info = None
        self.obstacles = []
        self.distance_tree = None
        self.free = []
        self.resolution = 0.01

        # Sensor
        self.sensor = None
        self.z_hit = 15
        self.z_random = 0
        self.z_max = 0.001

        # Particle filter parameters
        self.n_particles = 5
        # TODO: Add params for the particle filter
        """
        # IDEA: Use this to spawn "self.n_random" random particles every "self.random_frequency" iterations
        # This is to avoid trapping the robot if it has bad luck with the clustering of particles in the wrong place
        #self.n_random = 5
        #self.random_frequency = 5
        """

    # Initialize connections
    def connections_init(self):
        # Turtlebot connections
        self.cmd_vel_mux_pub = rospy.Publisher('/yocs_cmd_vel_mux/input/navigation', Twist, queue_size=10)

        # Show particles
        self.particles_pub = rospy.Publisher('/particles', PoseArray, queue_size=1)

        # Show location when finished
        self.location_pub = rospy.Publisher('/location', Pose, queue_size=1)

        # Read map data
        rospy.Subscriber('/map', OccupancyGrid, self.generate_map)

        # Read sensor data
        rospy.Subscriber('/scan', LaserScan, self.set_sensor_data, queue_size=1)

    # TODO: BENJA: Detect if a pixel is an obstacle edge
    def is_obstacle_edge(self, pixel, map):
        return True

    # Store obstacles from map
    def generate_map(self, map):
        self.map_info = map.info
        width = self.map_info.width
        height = self.map_info.height
        np_map = np.array(map.data)
        np_map = np_map.reshape((height, width))
        map_img = 100 - np_map
        map_img = (map_img * (255 / 100.0)).astype(np.uint8)
        self.obstacles = []
        for h in range(map_img.shape[0]):
            for w in range(map_img.shape[1]):
                if map_img[h, w] == 0 and self.is_obstacle_edge((h, w), map_img):
                    self.obstacles.append([w, h])
                elif map_img[h, w] == 254:
                    self.free.append([w, h])
        #print(self.obstacles)
        # TODO: BENJA: use is_obstacle_edge method to improve this
        self.distance_tree = spatial.KDTree(self.obstacles)

    # Set sensor data
    def set_sensor_data(self, sensor_data):
        self.sensor = sensor_data

    # Read laser measurements
    def laser_scan(self, state, scan):
        # Get state data
        robot_x_pix = int(state.position.x)
        robot_y_pix = int(state.position.y)
        robot_ang = state.orientation.z

        # Calculate laser points
        laser_points = []
        num_angles = int((scan.angle_max - scan.angle_min) / scan.angle_increment)
        angles = np.linspace(scan.angle_min, scan.angle_max, num_angles)
        for z_ang, zk in zip(angles, scan.ranges):
            zx_pix = zk / self.map_info.resolution
            global_ang = -(z_ang + robot_ang)
            px_pix = int(zx_pix * np.cos(global_ang) + robot_x_pix)
            py_pix = int(zx_pix * np.sin(global_ang) + robot_y_pix)
            py_in_range = 0 < py_pix < self.map_info.height
            px_in_range = 0 < px_pix < self.map_info.width
            if px_in_range and py_in_range and zk < scan.range_max:
                laser_points.append((px_pix, py_pix))
        return laser_points

    # Normal distribution
    @staticmethod
    def gaussian(x, mean, std_dev):
        a = 1 / (std_dev * np.sqrt(2 * np.pi))
        return a * np.exp(-(x - mean) ** 2 / (2 * std_dev ** 2))

    # Likelihood fields model
    def sensor_model(self, states, measurements):
        states_likelihood = []
        for state in states:
            likelihood = 1

            # Get laser points (assuming state)
            laser_points = self.laser_scan(state, measurements)

            # Calculate distance to closest obstacles (assuming laser points from state)
            distances, _ = self.distance_tree.query(laser_points)

            # TODO: BENJA: improve gaussian with the obstacle edge change
            # TODO: BENJA: test different states likelihood (and adjust params)

            # Get likelihood according to expected distribution of distances
            for dist in distances:
                p_hit = self.gaussian(dist, 0, 5)
                likelihood *= self.z_hit * p_hit + self.z_random / self.z_max
            states_likelihood.append(likelihood)
        return states_likelihood

    # Particle filter algorithm (Monte Carlo localization)
    def particle_filter(self, states):
        # TODO: Sample motion of particles (x_t[m]) (from gaussian sample)
        points_list = list()
        m = 50
        x = pd.Series(np.linspace(0.2, 0.8, m))/self.resolution
        y = pd.Series(np.linspace(0.8, 2, m))/self.resolution
        # x = pd.Series(random.choices(np.arange(0.2, 0.8, 0.1), m))/self.resolution
        # y = pd.Series(random.choices(np.arange(0.2, 0.8, 0.1), m))/self.resolution
        points = zip(x,y)
        for x, y in points:
            point = Pose()
            # rospy.loginfo((x,y))
            point.position.x, point.position.y = x, y
            points_list.append(point)

        point_cloud = PoseArray(poses=points_list)
        self.particles_pub.publish(point_cloud)

        # TODO: Get measurement model weights (using sensor_model)

        points = [] # Posibles valores a tomar (posiciones que estén en el piso)
        dist = [] # Lista de mismo tamaño que positions
        # draw = np.random.choice(positions, m, p=dist)

        # TODO: Finish testing
        translated_states = []
        for i in range(0, 2):
            state = Pose()
            state.position.x = 50
            state.position.y = 50 + i * 60
            state.orientation.z = 0
            translated_states.append(state)
        weights = self.sensor_model(translated_states, self.sensor)
        print(weights)
        rospy.sleep(20)

        # TODO: Resampling of particles
        # TODO: Return particles
        return states  # TODO: replace this with the real updated particles

    # Main loop
    def run(self):
        located = False
        particles = []  # TODO: Initial particles
        while not located:
            # Stop the robot
            self.speed_msg.linear.x = 0
            self.speed_msg.angular.z = 0
            self.cmd_vel_mux_pub.publish(self.speed_msg)

            # Particle filter iteration
            particles = self.particle_filter(particles)
            # TODO: Complete the "draw_particles" method in "display_state.py" from the "visualization" package
            # TODO: Publish current particles as a PoseArray with particles_pub (with the above method completed they will be displayed)

            located = False  # TODO: STOPPING CRITERION: Check if the particles converge into a single point
            if located:  # The robot has successfully located itself
                # Stop the robot
                self.speed_msg.linear.x = 0
                self.speed_msg.angular.z = 0
                self.cmd_vel_mux_pub.publish(self.speed_msg)

                # Display robot localization
                """
                # This code will show the location found on the state image
                # Use it when the particle filter has finished and found a probable pose (x,y)
                location = Pose()
                location.position.x = the x value for the position found
                location.position.y = the y value for the position found
                self.location_pub.publish(location)
                """
                # TODO: play sound to say that the robot has located itself (sound play)
            else:
                # TODO: Resume movement until the next particle filter iteration
                # TODO: Move robot reactively with a PID controller to be at a fixed distance from a wall
                pass
            self.rate_obj.sleep()


# Run particle filter
if __name__ == '__main__':
    particle_filter = ParticleFilter()
