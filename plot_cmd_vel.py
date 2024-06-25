#!/usr/bin/env python

import rosbag
import matplotlib.pyplot as plt
import sys
from geometry_msgs.msg import Twist

def read_bag(bag_file):
    times = []
    linear_x = []
    linear_y = []
    angular_z = []

    # Open the bag file
    bag = rosbag.Bag(bag_file)

    try:
        # Read messages from the /cmd_vel topic
        for topic, msg, t in bag.read_messages(topics=['/cmd_vel']):
            times.append(t.to_sec())
            linear_x.append(msg.linear.x)
            linear_y.append(msg.linear.y)
            angular_z.append(msg.angular.z)
    finally:
        # Close the bag file
        bag.close()

    return times, linear_x, linear_y, angular_z

def plot_velocities(times, linear_x, linear_y, angular_z):
    plt.figure(figsize=(12, 8))

    plt.subplot(3, 1, 1)
    plt.plot(times, linear_x, label='Linear X')
    plt.xlabel('Time (s)')
    plt.ylabel('Linear X Velocity (m/s)')
    plt.legend()

    plt.subplot(3, 1, 2)
    plt.plot(times, linear_y, label='Linear Y')
    plt.xlabel('Time (s)')
    plt.ylabel('Linear Y Velocity (m/s)')
    plt.legend()

    plt.subplot(3, 1, 3)
    plt.plot(times, angular_z, label='Angular Z')
    plt.xlabel('Time (s)')
    plt.ylabel('Angular Z Velocity (rad/s)')
    plt.legend()

    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    if len(sys.argv) != 2:
        print("Usage: plot_cmd_vel.py <bag_file>")
        sys.exit(1)

    bag_file = sys.argv[1]
    times, linear_x, linear_y, angular_z = read_bag(bag_file)
    plot_velocities(times, linear_x, linear_y, angular_z)
