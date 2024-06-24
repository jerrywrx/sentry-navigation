#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty

# Display instructions
msg = """
Control Your Robot!
---------------------------
Moving around:
   w
a  s  d
q : Rotate left
e : Rotate right
f : Stop
---------------------------
u/j : Increase/decrease max speeds by 10%
i/k : Increase/decrease only linear speed by 10%
o/l : Increase/decrease only angular speed by 10%
CTRL-C to quit
"""

# Key mappings
moveBindings = {
    'w': (1, 0, 0, 0),
    's': (-1, 0, 0, 0),
    'a': (0, 1, 0, 0),
    'd': (0, -1, 0, 0),
    'q': (0, 0, 0, 1),
    'e': (0, 0, 0, -1),
    'f': (0, 0, 0, 0),
}

speedBindings = {
    'u': (1.1, 1.1),
    'j': (0.9, 0.9),
    'i': (1.1, 1.0),
    'k': (0.9, 1.0),
    'o': (1.0, 1.1),
    'l': (1.0, 0.9),
}

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed, turn)

if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('teleop_keyboard')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    speed = 1.5
    turn = 1.5
    x = 0
    y = 0
    z = 0
    th = 0
    status = 0

    try:
        print(msg)
        print(vels(speed, turn))
        while not rospy.is_shutdown():
            key = getKey()
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                z = moveBindings[key][2]
                th = moveBindings[key][3]
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                turn = turn * speedBindings[key][1]
                print(vels(speed, turn))
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15
            else:
                x = 0
                y = 0
                z = 0
                th = 0
                if (key == '\x03'):
                    break

            twist = Twist()
            twist.linear.x = x * speed
            twist.linear.y = y * speed
            twist.linear.z = z * speed
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = th * turn
            pub.publish(twist)

    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        pub.publish(twist)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
