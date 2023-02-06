#!/usr/bin/env python3

import rospy
from numpy import ndarray
from local_planning.srv import servise, serviseResponse


def callback(req):
    
    # create response message
    response = serviseResponse()
    
    # Execute the task
    data = req.data
    response.success = True
    response.data = data
    
    # return your response    
    return response
    
class server_response:
   def __init__(self):
       
       rospy.init_node("/Server")
       self.srv = rospy.Service( ,service , self.srvCallback)

rospy.init_node("server")

rospy.Service("/local_planning_srv", servise, callback)

rospy.spin()
