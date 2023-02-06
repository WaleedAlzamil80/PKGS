#!/usr/bin/env python3

import sys
import time

import rospy
import rostest

import tf2_py as tf2
import tf2_ros

from geometry_msgs.msg import *


rospy.init_node('file')
pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size = 10)

msg = PoseWithCovarianceStamped()

msg.header.seq = 0
msg.header.stamp.secs = 0
msg.header.stamp.nsecs = 0
msg.header.frame_id = "map"
msg.pose.pose.position.x = 0.0
msg.pose.pose.position.y = 0.0
msg.pose.pose.position.z = 0.0
msg.pose.pose.orientation.x = 0.0
msg.pose.pose.orientation.y = 0.0
msg.pose.pose.orientation.z = 0.0
msg.pose.pose.orientation.w = 1.0
msg.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787]
while not rospy.is_shutdown():
	pub.publish(msg)
 
'''
header: 
  seq: 0
  stamp: 
    secs: 1670171527
    nsecs: 958407776
  frame_id: "map"
pose: 
  pose: 
    position: 
      x: 0.3074305057525635
      y: -0.08783185482025146
      z: 0.0
    orientation: 
      x: 0.0
      y: 0.0
      z: -0.01879817717373822
      w: 0.9998232986557899
  covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787]

'''
