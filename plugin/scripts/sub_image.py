#!/usr/bin/env python3

# Import the necessary libraries
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# Define a callback function to convert the ROS message to an image and display it
def image_callback(ros_image):
    try:
        # Convert the ROS message to an image
        img = CvBridge().imgmsg_to_cv2(ros_image, "bgr8")

        # Display the image
        cv2.imshow("IP Camera Stream", img)
        cv2.waitKey(1)
    except CvBridgeError as e:
        print(e)

# Initialize the ROS node and the subscriber
rospy.init_node("ip_camera_subscriber", anonymous=True)
sub = rospy.Subscriber("/camera/image", Image, image_callback)

# Keep the ROS node running
rospy.spin()