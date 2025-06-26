#!/usr/bin/env python
import rospy
import actionlib
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# Global flag set by image callback
object_detected = False

def image_cb(msg):
    global object_detected
    # Convert ROS Image to OpenCV
    frame = bridge.imgmsg_to_cv2(msg, 'bgr8')
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # Detect faces (using Haar cascade as example)
    faces = face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5)
    object_detected = len(faces) > 0

def move_to(x, y, w=1.0):
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.w = w
    client.send_goal(goal)

if __name__ == '__main__':
    rospy.init_node('smart_nav')

    # Prepare face detector and CvBridge
    bridge = CvBridge()
    face_cascade = cv2.CascadeClassifier(
        cv2.data.haarcascades + 'haarcascade_frontalface_default.xml'
    )

    # Subscribe to camera
    rospy.Subscriber('/camera/rgb/image_raw', Image, image_cb)

    # Connect to move_base
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    rospy.loginfo("Waiting for move_base server...")
    client.wait_for_server()
    rospy.loginfo("Connected to move_base server")

    # Example target; replace with your waypoint
    target_x, target_y = 2.0, 1.0
    origin_x, origin_y = 0.0, 0.0

    # Send initial goal
    move_to(target_x, target_y)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if object_detected:
            rospy.logwarn("Object detected! Pausing navigation.")
            client.cancel_goal()
            # Wait until object moves away
            while object_detected and not rospy.is_shutdown():
                rate.sleep()
            rospy.loginfo("Resuming navigation to target.")
            move_to(target_x, target_y)
        rate.sleep()

