#!/usr/bin/env python

import numpy as np
import rospy
from collections import Counter, deque
from geometry_msgs.msg import Pose, PoseArray, Twist
from nav_msgs.msg import OccupancyGrid
from pid_controller import PIDController
from random import choices, uniform
from scipy import spatial
from sensor_msgs.msg import LaserScan
from sound_play.libsoundplay import SoundClient
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
        self.default_linear_speed = 0.2  # Linear speed when following wall
        self.default_angular_speed = 1.0  # Angular speed when changing direction
        self.test_angular_speed = 6.0  # Angular speed when testing angles for candidate
        self.linear_speed = 0  # Current linear speed
        self.angular_speed = 0  # Current angular speed
        self.speed_msg = Twist()  # Message to move the robot
        self.action_time = 0.2  # Action time per iteration (in seconds)
        self.timer = None  # Used to get the real action time each iteration
        self.changing_direction = False  # True if the robot is in the changing direction subroutine
        self.free_distance = 0.7  # Distance limit required to stop changing direction and start moving again
        self.blocked_distance = 0.4  # Distance limit to stop moving and start changing direction
        self.wall_distance = 0.7  # Desired distance to the right wall when navigating
        self.wall_tracker = PIDController('wall_tracker')  # PID controller for navigation
        self.wall_tracker.pub_set_point(self.wall_distance)
        self.right_distances = deque(maxlen=5)  # The 'x' most recent distances to the right wall

        # Map
        self.map_info = None
        self.resolution = 0.01
        self.obstacles = []
        self.distance_tree = None  # KDTree for quick access to the nearest obstacle + closest distance
        self.free = []  # Valid poses for the particles
        self.valid = set()  # Valid coordinates (used for checking validity of translated particles)

        # Localization
        self.located = False  # True if the robot has located itself
        self.candidate = None  # Current candidate for the location
        self.checking = False  # True if the robot is checking a candidate
        self.angles_tested = 0  # Angles used to test the candidate

        # Sensor
        self.sensor = None  # Sensor measurements
        self.z_hit = 13  # Weight for the gaussian component of the likelihood calculation
        self.z_random = 0  # Weight for the random noise component of the likelihood calculation
        self.z_max = 0.001  # Weight for the error component of the likelihood calculation

        # Particle filter parameters
        self.n_particles = 2000  # Number of particles
        self.confidence_likelihood = 10 ** (-3)  # Any particles with less likelihood will be replaced by random ones
        self.default_likelihood = 10 ** (-15)  # Default likelihood for new random particles
        self.invalid_threshold = 0.9  # Min proportion of valid laser points to consider a state in the likelihood calculation
        self.localization_threshold = 0.9  # Proportion of equal particles to assume convergence

        # Soundplay
        self.sound_handler = SoundClient()

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

        # Rightmost laser distance (with smoother transitions)
        self.right_distances.append(valid_lasers[0])
        right_distance = min(self.right_distances)
        self.wall_tracker.pub_state(right_distance)

        # If already rotating and front laser measures enough distance to move, stop rotating
        if self.changing_direction and front_distance > self.free_distance:
            self.changing_direction = False

        # If front laser measures wall close enough or is already rotating, start/keep rotating
        if self.changing_direction or front_distance < self.blocked_distance:
            self.changing_direction = True
            self.linear_speed = 0
            self.angular_speed = self.default_angular_speed
            self.speed_msg.linear.x = self.linear_speed
            self.speed_msg.angular.z = self.angular_speed
            return

        # Reactive tracking of right wall distance
        self.linear_speed = self.default_linear_speed
        self.angular_speed = self.wall_tracker.speed
        self.speed_msg.linear.x = self.linear_speed
        self.speed_msg.angular.z = self.angular_speed

    # Check candidate by testing with different angles
    def check_candidate(self, scan):
        # Candidate is chosen as real location or discarded
        if self.angles_tested >= 2 * np.pi or self.candidate is None:
            self.checking = False
            self.located = self.candidate is not None
            return

        # Try next angle
        ideal_angle = abs(self.test_angular_speed) * self.action_time
        angular = max(np.random.normal(ideal_angle * 0.92, ideal_angle * 0.35), 0)
        self.angles_tested += angular

        # Rotate to try different angles
        self.linear_speed = 0
        self.angular_speed = self.test_angular_speed
        self.speed_msg.linear.x = self.linear_speed
        self.speed_msg.angular.z = self.angular_speed
        self.changing_direction = False

        # Valid laser measurements
        valid_lasers = [l for l in scan.ranges if l < scan.range_max]

        # Rightmost laser distance (with smoother transitions)
        self.right_distances.append(valid_lasers[0])
        right_distance = min(self.right_distances)
        self.wall_tracker.pub_state(right_distance)

    # Main loop
    def run(self):
        # Initialize particles
        particles = []
        for p in choices(self.free, k=self.n_particles):
            particle = Pose()
            particle.position.x, particle.position.y = p.position.x, p.position.y
            particle.orientation.z = uniform(0, 2 * np.pi - 1)
            particles.append(particle)
        self.particles_pub.publish(PoseArray(poses=particles))

        # Start iterations
        while not self.located:
            # Stop the robot
            self.speed_msg.linear.x = 0
            self.speed_msg.angular.z = 0
            self.cmd_vel_mux_pub.publish(self.speed_msg)

            # Particle filter iteration
            particles = self.particle_filter(particles)
            self.particles_pub.publish(PoseArray(poses=particles))

            # Check if the particles have converged
            location, count = Counter([(p.position.x, p.position.y) for p in particles]).most_common(1)[0]
            converged = count >= self.localization_threshold * self.n_particles
            if not self.checking and converged:
                self.candidate = location
                self.checking = True
                self.angles_tested = 0
            elif self.checking and (not converged or location != self.candidate):
                self.candidate = None

            # Navigation
            if self.checking:
                self.check_candidate(self.sensor)
                if self.located:
                    break
            if not self.checking:
                self.reactive_navigation(self.sensor)
            self.cmd_vel_mux_pub.publish(self.speed_msg)
            self.timer = time()
            rospy.sleep(self.action_time)

        # The robot has successfully located itself

        # Stop the robot
        self.speed_msg.linear.x = 0
        self.speed_msg.angular.z = 0
        self.cmd_vel_mux_pub.publish(self.speed_msg)
        self.wall_tracker.disable()

        # Display robot localization
        location = Pose()
        location.position.x, location.position.y = self.candidate
        self.location_pub.publish(location)
        self.sound_handler.say('Robot Localized!', voice='voice_kal_diphone', volume=1.0)


# Run particle filter
if __name__ == '__main__':
    particle_filter = ParticleFilter()
