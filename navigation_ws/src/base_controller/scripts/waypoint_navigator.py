#!/usr/bin/env python
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib

# Define waypoints
waypoints = {
    'a': {'x': -3.791390895843506, 'y': -2.7204017639160156, 'oz': -0.9999955389682309, 'ow': 0.0029869790152065715},
    'b': {'x': -2.3178482055664062, 'y': 3.0225839614868164, 'oz': 0.7075646159501544, 'ow': 0.7066486497937362},
    'c': {'x': 1.0240206718444824, 'y': 3.1247355937957764, 'oz': -0.6980509641612888, 'ow': 0.716048078995744}
}

def send_goal(waypoint):
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = waypoint['x']
    goal.target_pose.pose.position.y = waypoint['y']
    goal.target_pose.pose.orientation.z = waypoint['oz']
    goal.target_pose.pose.orientation.w = waypoint['ow']

    client.send_goal(goal)
    wait = client.wait_for_result()

    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()

def main():
    rospy.init_node('waypoint_navigator')

    while not rospy.is_shutdown():
        user_input = input("Enter waypoint (a, b, c) or 'exit' to quit: ").strip().lower()

        if user_input == 'exit':
            break

        if user_input in waypoints:
            rospy.loginfo(f"Sending robot to waypoint {user_input}")
            result = send_goal(waypoints[user_input])
            if result:
                rospy.loginfo("Goal execution done!")
        else:
            rospy.loginfo("Invalid waypoint. Please enter a, b, c, or 'exit'.")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation interrupted.")
