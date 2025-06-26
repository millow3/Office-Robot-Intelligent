#!/usr/bin/env python
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def move_to_goal(x, y):
    goal = WMoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.w = 1.0  # Facing forward

    client.send_goal(goal)
    client.wait_for_result()
    return client.get_result()

if __name__ == '__main__':
    rospy.init_node('go_to_point_node')
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    rospy.loginfo("Waiting for move_base server...")
    client.wait_for_server()
    rospy.loginfo("Connected to move_base")

    while not rospy.is_shutdown():
        try:
            x = float(input("Enter X coordinate: "))
            y = float(input("Enter Y coordinate: "))
            result = move_to_goal(x, y)
            if result:
                rospy.loginfo("Successfully reached destination.")
            else:
                rospy.logwarn("Failed to reach destination.")
        except KeyboardInterrupt:
            print("\nExiting...")
            break

