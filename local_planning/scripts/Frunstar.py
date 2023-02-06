 #!/bin/env python3
import roslib
import rospy
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray

from FRRTStar import RRTStarPlanner
from Futils import * 
import numpy as np
import time 
import math

# x = 0 
# y = 0
# yaw = 0
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
    # limits 
    #global x, y, xi, yi, x_goal, y_goal
    map = Map(size[0], size[1], size[2], size[3])
   
    # add obstacles
    for obs in obstacles:
        map.add_obstacle(CircleObstacle(pos=Point(obs[0], obs[1]), radius=obs[2])) 

    wayPoints_msg = Float64MultiArray()
   
    # instantiate an instance from rrtstarplanner
    rrtPlanner = RRTStarPlanner(map, epsilon=0.2, stepSize=stepsize)

    # check if there any dynamic obstacles nearby else publish the optimized curve
    # obs_exist = True
    # while map.is_open and obs_exist:

    # set the position of the agent (vehicle)
    # we need to update this value from AMCL
    start = Point(start_p[0], start_p[1])
    
    # set the goal
    # we need to update this value, we need to choose a new goal
    end = Point(goal[0], goal[1])
    
    # plan 
    rrtPlanner.plan(start=start, target=end)
    final_path = list()
    # if(dist((x,y), (x_goal,y_goal)) < 0.5):
    #     break
    for i in range(len(rrtPlanner.finalPath)): 
        # print([rrtPlanner.finalPath[i].tuple()[0] /90, rrtPlanner.finalPath[i].tuple()[1] /90])  
        final_path.append([rrtPlanner.finalPath[i].tuple()[0] /90, rrtPlanner.finalPath[i].tuple()[1] /90]) 
        # if(dist((x,y), (x_goal,y_goal)) < 0.5):
        #     break
        # go_to_point(rrtPlanner.finalPath[i].tuple()[0] /90, rrtPlanner.finalPath[i].tuple()[1]/90)

    final_path = np.array(final_path).reshape(-1, 2)       
    wayPoints_msg.data = final_path
    wayPoints_publisher.publish(wayPoints_msg) 
    # map.add_geometry(type='point', pos=start.tuple(), size=3, color=(100, 0, 0))
    # map.add_geometry(type='point', pos=end.tuple(), size=3, color=(0, 100, 0))
    
    # for node in rrtPlanner.nodeList:
    #     map.add_geometry(type='point', pos=node.pos.tuple())
    #     if node.parent is not None:
    #         map.add_geometry(type='line', start=node.parent.pos.tuple(), end=node.pos.tuple())
    
    
    # for i in range(len(rrtPlanner.finalPath)-1):
    #     map.add_geometry(type='line', start=rrtPlanner.finalPath[i].tuple(), end=rrtPlanner.finalPath[i+1].tuple(), color=(0, 100, 0))

    # map.render()

def go_to_point(X_goal, Y_goal):

    global x, y, yaw
    velocity_message = Twist()

    K_linear = 5
    distance = math.sqrt((X_goal - x)**2 + (Y_goal - y)**2)
    linear_speed = distance * K_linear
    K_angular = 3
    desired_angle_goal = math.atan2((Y_goal - y),(X_goal - x))

    angular_speed = (desired_angle_goal - yaw)*K_angular
    velocity_message.linear.x = linear_speed
    velocity_message.angular.z = angular_speed
    velocity_publisher.publish(velocity_message)


def dist(p1, p2):
    return np.sqrt(np.power(p2[0]-p1[0], 2) + np.power(p2[1] - p1[1], 2))

if __name__=="__main__":

    rospy.init_node('Start1')
    velocity_publisher = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size = 10)
    pose_subscriber = rospy.Subscriber("/turtle1/pose", Pose, callback = pose_callback)
    wayPoints_publisher = rospy.Publisher("/local_planning/way_points", Float64MultiArray, queue_size = 10)
    wayPoints_subscriper = rospy.Subscriber("/local_planning/way_points", Float64MultiArray, callback = wayPoints_callback)

    time.sleep(2)
    # check if there any dynamic obstacles nearby else publish the optimized curve
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
        start = (x, y)
        
        # we need to specify how we will set the next point to go to when there are dynamic obstacles 
        # and also we need to determine the range of search 
        # maybe there will be some kind of mapping and transform
        goal = (x_goal, y_goal)
        size = (500, 0, 0, 500)
        
        # Do it
        RRTSTAR_ALGO(start, goal, size, obs)

        char = input("press [Y] if you want to enter a new position Otherwise it'll exit : ")
        if(char != "Y"):
            break

