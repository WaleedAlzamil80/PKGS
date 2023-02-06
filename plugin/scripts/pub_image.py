#!/usr/bin/env python3

# Import the necessary libraries
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# Define a callback function to convert the image and publish it to a ROS topic
def image_callback(img):
    try:
        # Convert the image to a ROS message
        ros_image = CvBridge().cv2_to_imgmsg(img, "bgr8")

        # Publish the image to a ROS topic
        pub.publish(ros_image)
    except CvBridgeError as e:
        print(e)

# Initialize the ROS node and the publisher
rospy.init_node("ip_camera_publisher", anonymous=True)
pub = rospy.Publisher("/camera/image", Image, queue_size=1)

# Open the IP camera stream
cap = cv2.VideoCapture("http://192.168.1.9:81/stream")

# Continuously read frames from the IP camera stream
while not rospy.is_shutdown():
    ret, frame = cap.read()
    if ret:
        # Call the callback function with the latest frame
        image_callback(frame)
