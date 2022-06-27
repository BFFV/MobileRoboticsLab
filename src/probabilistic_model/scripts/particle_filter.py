#!/usr/bin/env python

import numpy as np
import rospy
from collections import Counter, deque
from geometry_msgs.msg import Pose, PoseArray, Twist
from nav_msgs.msg import OccupancyGrid
from pid_controller import PIDController
from random import choices, uniform, random
from scipy import spatial
from sensor_msgs.msg import LaserScan
from time import time


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
        # Actuation
        self.linear_speed = 0.2  # Linear speed when following wall
        self.angular_speed = 0.5  # Angular speed when changing direction
        self.speed_msg = Twist()
        self.timer = None
        self.changing_direction = False
        self.free_distance = 0.7
        self.blocked_distance = 0.4
        self.wall_distance = 0.7
        self.wall_tracker = PIDController('wall_tracker')
        self.wall_tracker.pub_set_point(self.wall_distance)
        self.right_distances = deque(maxlen=5)

        # Map
        self.map_info = None
        self.resolution = 0.01
        self.obstacles = []
        self.distance_tree = None
        self.free = []
        self.valid = set()

        # Sensor
        self.sensor = None
        self.z_hit = 12
        self.z_random = 0
        self.z_max = 0.001

        # Particle filter parameters
        self.n_particles = 3000  # Number of particles
        self.confidence_likelihood = 10 ** (-5)  # Any particles with less likelihood have a chance of being replaced by random ones
        self.min_likelihood = 10 ** (-15)  # Minimum value of likelihood to consider a particle
        self.likelihood_range = self.confidence_likelihood - self.min_likelihood  # Likelihood range for partially valuable particles
        self.default_likelihood = self.confidence_likelihood - self.likelihood_range / 2  # Default likelihood for new random particles
        self.invalid_threshold = 0.8  # Min proportion of valid laser points to consider a state in the likelihood calculation
        self.localization_threshold = 0.995  # Proportion of equal particles to assume that the location has been found

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
        self.distance_tree = spatial.cKDTree(self.obstacles)

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
        n_valid_measurements = 0
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
            if zk < scan.range_max:
                n_valid_measurements += 1
        return laser_points, n_valid_measurements

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
            laser_points, n_valid = self.laser_scan(state, measurements)

            # State lacks valid information (directly incompatible)
            if len(laser_points) < n_valid * self.invalid_threshold:
                states_likelihood.append(0.0)
                continue

            # Calculate distance to closest obstacles (assuming laser points from state)
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
                ideal_angle = abs(self.angular_speed) * action_time
                for state in states:
                    angular = max(np.random.normal(ideal_angle * 0.92, ideal_angle * 0.35), 0)
                    if self.angular_speed < 0:
                        angular *= -1
                    state.orientation.z += angular
                    state.orientation.z %= 2 * np.pi
            if self.linear_speed:
                ideal_distance = (self.linear_speed * action_time) / self.resolution
                for state in states:
                    linear = max(np.random.normal(ideal_distance * 1.1, ideal_distance * 0.05), 0)
                    current_angle = state.orientation.z
                    state.position.x += round(linear * np.cos(current_angle))
                    state.position.y -= round(linear * np.sin(current_angle))

        # Replace invalid particles with random ones
        translated_states = []
        for state in states:
            if (state.position.x, state.position.y) in self.valid:
                translated_states.append(state)
            else:
                random_state = choices(self.free, k=1)[0]
                rand = Pose()
                rand.position.x, rand.position.y = random_state.position.x, random_state.position.y
                rand.orientation.z = uniform(0, 2 * np.pi - 1)
                translated_states.append(rand)

        # Sensor model
        weights = self.sensor_model(translated_states, self.sensor)

        # Replace unlikely particles with random ones
        for idx, state in enumerate(translated_states):
            if weights[idx] < self.confidence_likelihood:
                confidence = 1 - min((self.confidence_likelihood - weights[idx]) / self.likelihood_range, 1)
                if random() >= confidence:  # Particle will be replaced
                    rand = choices(self.free, k=1)[0]
                    translated_states[idx].position.x, translated_states[idx].position.y = rand.position.x, rand.position.y
                    translated_states[idx].orientation.z = uniform(0, 2 * np.pi - 1)
                    weights[idx] = self.default_likelihood

        # Resampling using weights
        new_particles = []
        for p in choices(translated_states, weights=weights, k=self.n_particles):
            new_particle = Pose()
            new_particle.position.x, new_particle.position.y = p.position.x, p.position.y
            new_particle.orientation.z = p.orientation.z
            new_particles.append(new_particle)
        return new_particles

    # Navigate reactively using walls as a reference
    def reactive_navigation(self, scan):
        # Valid laser measurements
        valid_lasers = [l for l in scan.ranges if l < scan.range_max]

        # Front laser distance
        if len(valid_lasers) % 2:
            front_distance = valid_lasers[int(len(valid_lasers) / 2)]
        else:
            left_front = valid_lasers[int(len(valid_lasers) / 2) - 1]
            right_front = valid_lasers[int(len(valid_lasers) / 2)]
            front_distance = (left_front + right_front) / 2

        # If already rotating and front laser measures enough distance to move, stop rotating
        if self.changing_direction and front_distance > self.free_distance:
            self.changing_direction = False

        # If front laser measures wall close enough or is already rotating, start/keep rotating
        if self.changing_direction or front_distance < self.blocked_distance:
            self.changing_direction = True
            self.speed_msg.linear.x = 0
            self.speed_msg.angular.z = self.angular_speed
            self.right_distances.clear()
            return

        # Rightmost laser distance (with smoother transitions)
        self.right_distances.append(valid_lasers[0])
        right_distance = sum(self.right_distances) / len(self.right_distances)

        # Reactive tracking of right wall distance
        self.speed_msg.linear.x = self.linear_speed
        self.wall_tracker.pub_state(right_distance)
        self.speed_msg.angular.z = self.wall_tracker.speed

    # Main loop
    def run(self):
        located = False

        # Initialize particles
        particles = []
        for p in choices(self.free, k=self.n_particles):
            particle = Pose()
            particle.position.x, particle.position.y = p.position.x, p.position.y
            particle.orientation.z = uniform(0, 2 * np.pi - 1)
            particles.append(particle)
        self.particles_pub.publish(PoseArray(poses=particles))

        # Start iterations
        while not located:
            # Stop the robot
            self.speed_msg.linear.x = 0
            self.speed_msg.angular.z = 0
            self.cmd_vel_mux_pub.publish(self.speed_msg)

            # Particle filter iteration
            particles = self.particle_filter(particles)
            self.particles_pub.publish(PoseArray(poses=particles))

            # Check if the particles have converged
            loc, count = Counter([(p.position.x, p.position.y) for p in particles]).most_common(1)[0]
            located = count >= self.localization_threshold * self.n_particles
            if located:  # The robot has successfully located itself
                # Stop the robot
                self.speed_msg.linear.x = 0
                self.speed_msg.angular.z = 0
                self.cmd_vel_mux_pub.publish(self.speed_msg)

                # Display robot localization
                location = Pose()
                location.position.x, location.position.y = loc
                self.location_pub.publish(location)
                # TODO: play sound to say that the robot has located itself (sound play)
            else:
                self.reactive_navigation(self.sensor)
                self.cmd_vel_mux_pub.publish(self.speed_msg)
                self.timer = time()
                rospy.sleep(0.2)


# Run particle filter
if __name__ == '__main__':
    particle_filter = ParticleFilter()
