#!/usr/bin/env python

import numpy as np
import rospy
from geometry_msgs.msg import Pose, PoseArray, Twist
from nav_msgs.msg import OccupancyGrid
from scipy import spatial
from sensor_msgs.msg import LaserScan
from time import time
from tf.transformations import euler_from_quaternion  # TODO: remove


# Particle filter model for localization
class ParticleFilter:
    def __init__(self):
        rospy.init_node('particle_filter')
        self.variables_init()
        self.connections_init()
        while not self.distance_tree:
            rospy.sleep(1)
        self.run()
        rospy.spin()

    # Initialize variables
    def variables_init(self):
        # TODO: remove later
        self.angle = 0
        self.angle_old = 0

        # Actuation
        self.linear_speed = 0.1
        self.angular_speed = 0.0
        self.speed_msg = Twist()
        self.timer = None

        # Map
        self.map_info = None
        self.resolution = 0.01
        self.obstacles = []
        self.distance_tree = None
        self.free = []
        self.valid = set()

        # Sensor
        self.sensor = None
        self.z_hit = 13
        self.z_random = 0
        self.z_max = 0.001

        # Particle filter parameters
        self.n_particles = 100
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

        # TODO: remove later, ODOM
        rospy.Subscriber('/real_pose', Pose, self.set_init_pose)

    # TODO: remove later, Set initial pose
    def set_init_pose(self, pose_data):
        quaternion = (pose_data.orientation.x,
                      pose_data.orientation.y,
                      pose_data.orientation.z,
                      pose_data.orientation.w)
        _, _, yaw = euler_from_quaternion(quaternion)
        self.angle = yaw

    # Detect if a pixel is an obstacle edge
    def is_obstacle_edge(self, pixel, map):
        for y_idx in range(-1, 2):
            for x_idx in range(-1, 2):
                if y_idx == 0 and x_idx == 0:
                    continue
                adjacent_pixel = list(pixel)
                adjacent_pixel[0] += y_idx
                adjacent_pixel[1] += x_idx
                if map[adjacent_pixel[0], adjacent_pixel[1]] == 254:
                    return True
        return False

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
                    free_pose = Pose()
                    free_pose.position.x = w
                    free_pose.position.y = h
                    self.free.append(free_pose)
                    self.valid.add((w, h))
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
            if len(laser_points) == 0:
                print("asdasd",laser_points)
                print("state",state)
            distances, _ = self.distance_tree.query(laser_points)

            # Get likelihood according to expected distribution of distances
            for dist in distances:
                p_hit = self.gaussian(dist, 0, 3)
                likelihood *= self.z_hit * p_hit + self.z_random / self.z_max
            states_likelihood.append(likelihood)
        return states_likelihood

    # Particle filter algorithm (Monte Carlo localization)
    def particle_filter(self, states):
        # Get action time
        action_time = 0
        if self.timer:
            action_time = time() - self.timer

        # Motion model
        if action_time:
            if self.angular_speed:
                ideal_angle = self.angular_speed * action_time
                angular = np.random.normal(ideal_angle, abs(ideal_angle * 0.5))
                # TODO: check this
                states[idx].orientation.z += angular
            if self.linear_speed:
                ideal_distance = (self.linear_speed * action_time) / self.resolution
                linear = round(np.random.normal(ideal_distance, ideal_distance * 0.8))
                for idx in range(self.n_particles):
                    # TODO: formula for every possible angle
                    current_angle = states[idx].orientation.z
                    states[idx].position.x += round(linear * np.cos(current_angle))
                    states[idx].position.y += round(linear * np.sin(current_angle))

        # TODO: test all this
        # Filter invalid translated states
        translated_states = np.array([point for point in states if (point.position.x, point.position.y) in self.valid])
        out_of_boundry_points = len(states) - len(translated_states)
        
        new_points = np.random.choice(self.free, out_of_boundry_points)

        translated_states = np.append(translated_states, new_points)

        # Sensor model
        weights = self.sensor_model(translated_states, self.sensor)
        #print(self.weights)
        rospy.sleep(1)
        return np.random.choice(translated_states, self.n_particles, weights)
        # return np.random.choice(translated_states, self.n_particles)

    # Main loop
    def run(self):
        located = False

        # Initialize particles

        # Testing
        state = Pose()
        state.position.x, state.position.y = 249, 237
        particles = np.array([state]*100)
        self.particles_pub.publish(PoseArray(poses=particles))
        
        # Production
        # particles = np.random.choice(self.free, self.n_particles)
        # self.particles_pub.publish(PoseArray(poses=particles))

        # Start iterations
        while not located:
            # Stop the robot
            self.speed_msg.linear.x = 0
            self.speed_msg.angular.z = 0
            self.cmd_vel_mux_pub.publish(self.speed_msg)

            # Particle filter iteration
            particles = self.particle_filter(particles)
            self.particles_pub.publish(PoseArray(poses=particles))

            # TODO: Check if the particles have converged
            located = False
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
                print()
                self.speed_msg.linear.x = self.linear_speed
                self.speed_msg.angular.z = self.angular_speed
                self.cmd_vel_mux_pub.publish(self.speed_msg)
                rospy.sleep(0.1)
                self.timer = time()
                # TODO(Refactor, linear speed better): Move robot reactively with a PID controller to be at a fixed distance from a wall



# Run particle filter
if __name__ == '__main__':
    particle_filter = ParticleFilter()
