#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

def move():
    rospy.init_node('office_nav', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz
    move_cmd = Twist()

    # Move forward
    move_cmd.linear.x = 0.2
    move_cmd.angular.z = 0.0

    rospy.loginfo("Moving forward...")
    for _ in range(5):
        pub.publish(move_cmd)
        rate.sleep()

    # Stop
    move_cmd.linear.x = 0.0
    rospy.loginfo("Stopping.")
    pub.publish(move_cmd)

if __name__ == '__main__':
    try:
        move()
    except rospy.ROSInterruptException:
        pass

