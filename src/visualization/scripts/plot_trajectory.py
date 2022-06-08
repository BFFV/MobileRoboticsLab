#!/usr/bin/env python3

import matplotlib.pyplot as plt
import rospy
from geometry_msgs.msg import PoseArray
from matplotlib.animation import FuncAnimation
from nav_msgs.msg import Odometry


# Dynamic trajectory visualizer
class Visualizer:
    def __init__(self):
        self.fig, self.ax = plt.subplots()
        self.real_ln, self.ln = plt.plot([], [], 'b', [], [], 'r')
        self.ln.set_label('real')
        self.real_ln.set_label('reference')
        self.ax.legend()
        self.x_data, self.y_data = [], []
        self.real_x, self.real_y = [], []

    def plot_init(self):
        self.ax.set_xlim(0, 6)
        self.ax.set_ylim(0, 3)
        return self.real_ln, self.ln

    def set_trajectory(self, msg):
        self.real_x = [p.position.x for p in msg.poses]
        self.real_y = [p.position.y for p in msg.poses]

    def odom_callback(self, msg):
        self.x_data.append(msg.pose.pose.position.x + 1)
        self.y_data.append(msg.pose.pose.position.y + 1)

    def update_plot(self, frame):
        self.real_ln.set_data(self.real_x, self.real_y)
        self.ln.set_data(self.x_data, self.y_data)
        return self.real_ln, self.ln

# Plot trajectory in real time
if __name__ == '__main__':
    rospy.init_node('plot_trajectory')
    vis = Visualizer()
    path_sub = rospy.Subscriber('/trajectory', PoseArray, vis.set_trajectory)
    ani = FuncAnimation(vis.fig, vis.update_plot, init_func=vis.plot_init, blit=True)
    rospy.sleep(3)
    odom_sub = rospy.Subscriber('/odom', Odometry, vis.odom_callback)
    plt.show(block=True)
