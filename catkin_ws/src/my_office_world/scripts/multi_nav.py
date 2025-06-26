#!/usr/bin/env python
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

waypoints = {
    "room1": (2.13, -1.85),
    "room2": (3.20, 0.45),
    "room3": (-1.0, 2.0),
    "origin": (0.0, 0.0)
}

def move_to_goal(x, y, yaw=0.0):
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.w = 1.0
    client.send_goal(goal)
    client.wait_for_result()
    return client.get_result()

if __name__ == '__main__':
    rospy.init_node('multi_room_nav')
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    rospy.loginfo("Waiting for move_base server...")
    client.wait_for_server()
    rospy.loginfo("Connected to move_base server")

    while not rospy.is_shutdown():
        rospy.loginfo("Available rooms: room1, room2, room3")
        dest = raw_input("Enter destination room (or 'exit'): ").strip()
        if dest == 'exit':
            break
        if dest in waypoints:
            x, y = waypoints[dest]
            rospy.loginfo("Navigating to %s at (%s, %s)" % (dest, x, y))
            result = move_to_goal(x, y)
            if result:
                rospy.loginfo("Arrived at %s. Returning to origin..." % dest)
                ox, oy = waypoints["origin"]
                move_to_goal(ox, oy)
                rospy.loginfo("Returned to origin.")
            else:
                rospy.logwarn("Failed to reach %s" % dest)
        else:
            rospy.logwarn("Invalid room entered.")

