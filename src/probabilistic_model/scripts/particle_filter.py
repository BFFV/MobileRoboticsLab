#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose, PoseArray, Twist
from math import atan2, sqrt
from nav_msgs.msg import Odometry, Path
from tf.transformations import euler_from_quaternion
# TODO: remove unnecessary imports


# Particle filter model for localization
class ParticleFilter:
    def __init__(self):
        rospy.init_node('particle_filter')
        self.variables_init()
        self.connections_init()
        rospy.sleep(3)
        self.run()
        rospy.spin()

    # Initialize variables
    def variables_init(self):
        # Message frequency: 10Hz
        self.hz = 10
        self.rate_obj = rospy.Rate(self.hz)

        # Speed message for actuation
        self.linear_speed = 0.0
        self.angular_speed = 0.5
        self.speed_msg = Twist()

        # Obstacles map
        self.obstacles = None

        # Sensor measurements
        self.measurements = []

        # Particle filter parameters
        self.n_particles = 5
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

    # Calculate distance between two points
    @staticmethod
    def distance(a, b):
        return sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2)

    # Store obstacles from map
    def generate_map(self):
        # TODO: use a KDTree
        self.obstacles = []

    # Read laser measurements
    def laser_scan(self):
        # TODO: use the laser scan from scan2pointcloud
        self.measurements = []

    # Likelihood fields model
    def sensor_model(self, state):
        # TODO: complete after the above methods are defined
        return 0.5

    # Particle filter algorithm (Monte Carlo localization)
    def particle_filter(self, states):
        # X_t-1 = states, u_t = motion, z_t = measurements
        # TODO: get motion (gaussian sample)

        # Get current measurements
        self.laser_scan()

        # TODO: First part: sample motion model and then measurement model for all particles
        state = Pose()
        state.position.x = 0
        state.position.y = 0
        print(self.sensor_model(state))
        # TODO: Second part: Resampling
        # TODO: Return particles
        pass

    # Main loop
    def run(self):
        # Store map with obstacles
        self.generate_map()

        # Start the particle filter iterations
        located = False
        particles = []  # TODO: Initial particles
        while not located:
            #particles = particle_filter(particles, motion, measurements)  # TODO: call particle_filter for this iteration
            # TODO: Complete the "draw_particles" method in "display_state.py" from the "visualization" package
            # TODO: Publish current particles as a PoseArray with particles_pub (with the above method completed they will be displayed)
            located = False  # TODO: STOPPING CRITERION: Check if the particles converge into a single point
            if located:  # The robot has successfully located itself
                self.speed_msg.linear.x = 0
                self.speed_msg.angular.z = 0
                self.cmd_vel_mux_pub.publish(self.speed_msg)
                """
                # This code will show the location found on the state image
                # Use it when the particle filter has finished and found a probable pose (x,y)
                location = Pose()
                location.position.x = the x value for the position found
                location.position.y = the y value for the position found
                self.location_pub.publish(location)
                """
            self.rate_obj.sleep()


# Run particle filter
if __name__ == '__main__':
    particle_filter = ParticleFilter()
