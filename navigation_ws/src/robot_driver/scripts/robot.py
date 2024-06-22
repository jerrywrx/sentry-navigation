#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import struct
from uart_communicator import UARTCommunicator
import config

class RobotDriver:
    def __init__(self):
        self.uart = UARTCommunicator(config)
        self.cmd_id = self.uart.cfg.CHASSIS_CMD_ID

    def cmd_callback(self, msg):
        rospy.loginfo("Received control speed!")
        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z
        
        data = {'vx': vx, 'vy': vy, 'vw': wz}
        self.uart.create_and_send_packet(self.cmd_id, data)
        rospy.loginfo(f"Sent velocities: vx={vx}, vy={vy}, wz={wz}")

    def start(self):
        rospy.init_node('robot_driver', anonymous=True)
        rospy.Subscriber("/cmd_vel", Twist, self.cmd_callback)
        
        rospy.loginfo("Robot driver node started, listening to /cmd_vel")
        rospy.spin()

if __name__ == '__main__':
    driver = RobotDriver()
    driver.start()
