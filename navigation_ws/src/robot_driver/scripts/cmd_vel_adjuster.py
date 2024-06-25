#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from dynamic_reconfigure.server import Server
from robot_driver.cfg import CmdVelAdjusterConfig

class CmdVelAdjuster:
    def __init__(self):
        self.linear_x = 0.0
        self.linear_y = 0.0
        self.linear_z = 0.0
        self.angular_x = 0.0
        self.angular_y = 0.0
        self.angular_z = 0.0
        self.transmission_frequency = 10.0
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.srv = Server(CmdVelAdjusterConfig, self.reconfigure_callback)

    def reconfigure_callback(self, config, level):
        self.linear_x = config.linear_x
        self.linear_y = config.linear_y
        self.linear_z = config.linear_z
        self.angular_x = config.angular_x
        self.angular_y = config.angular_y
        self.angular_z = config.angular_z
        self.transmission_frequency = config.transmission_frequency
        return config

    def publish_velocities(self):
        rate = rospy.Rate(self.transmission_frequency)
        while not rospy.is_shutdown():
            msg = Twist()
            msg.linear.x = self.linear_x
            msg.linear.y = self.linear_y
            msg.linear.z = self.linear_z
            msg.angular.x = self.angular_x
            msg.angular.y = self.angular_y
            msg.angular.z = self.angular_z
            self.pub.publish(msg)
            rate = rospy.Rate(self.transmission_frequency)  # Update the rate in case it has changed
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('cmd_vel_adjuster')
    adjuster = CmdVelAdjuster()
    adjuster.publish_velocities()
