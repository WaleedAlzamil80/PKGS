#!/bin/env python3
import roslib
import rospy
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray

from FRRTStar import RRTStarPlanner
from Futils import * 
import numpy as np
import time 
import math

obs_exist = True

def pose_callback(pose_message):

    global x, y, yaw
    x = pose_message.x
    y = pose_message.y
    yaw = pose_message.theta

def wayPoints_callback(waypoints_msg):
    global waypoints
    waypoints = waypoints_msg.data


def RRTSTAR_ALGO(start_p, goal, size, obstacles, stepsize):
    # add the size (range of search)
    map = Map(size[0], size[1], size[2], size[3])
   
    # add obstacles
    for obs in obstacles:
        map.add_obstacle(CircleObstacle(pos=Point(obs[0], obs[1]), radius=obs[2])) 
   
    # instantiate an instance from rrtstarplanner
    rrtPlanner = RRTStarPlanner(map, epsilon=0.2, stepSize=stepsize)

    # set the position of the agent (vehicle)
    # we need to update this value from AMCL
    start = Point(start_p[0], start_p[1])
    
    # set the goal
    # we need to update this value, we need to choose a new goal
    end = Point(goal[0], goal[1])
    
    # plan 
    rrtPlanner.plan(start=start, target=end) # rrtplanner.finalpath
    for i in range(len(rrtPlanner.finalPath)): 
        
        ## go to controller
        velocity_message = Twist()
        X_goal,Y_goal = (rrtPlanner.finalPath[i].tuple()[0] /90.0, rrtPlanner.finalPath[i].tuple()[1] /90.0)
        K_linear = 5
        distance = math.sqrt((X_goal - x)**2 + (Y_goal - y)**2)
        linear_speed = distance * K_linear
        K_angular = 0.3
        desired_angle_goal = math.atan2((Y_goal - y),(X_goal - x))

        angular_speed = (desired_angle_goal - yaw)*K_angular
        velocity_message.linear.x = linear_speed
        velocity_message.angular.z = angular_speed
        velocity_publisher.publish(velocity_message)


if __name__=="__main__":

    rospy.init_node('Start1')
    velocity_publisher = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size = 10)
    pose_subscriber = rospy.Subscriber("/turtle1/pose", Pose, callback = pose_callback)

    time.sleep(2)
    while obs_exist :
        x_goal = float(input("Enter the x coordinate for the new position : "))
        y_goal = float(input("Enter the y coordinate for the new position : "))
        
        ## pass the required paramters to the rtt* algo.  then pass the result to go to point function 
        # Frm perception   
        obs =[
            (100, 100, 25),
            (200, 150, 12.5),
            (37, 200, 25)
        ]
        
        # From localization
        start = (x*45, y*45)
        
        # we need to specify how we will set the next point to go to when there are dynamic obstacles 
        # and also we need to determine the range of search maybe there will be some kind of mapping and transform
        goal = (x_goal*45, y_goal*45)
        size = (500, 0, 0, 500)
        
        # Do it
        RRTSTAR_ALGO(start, goal, size, obs, 5)

        char = input("press [Y] if you want to enter a new position Otherwise it'll exit : ")
        if(char != "Y"):
            break
