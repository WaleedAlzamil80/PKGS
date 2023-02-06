#!/usr/bin/env python3

import rospy
from numpy import ndarray
from local_planning.srv import servise, serviseRequest


rospy.init_node("client")

# define the client and wait for a service
client = rospy.ServiceProxy("/local_planning_srv", servise)
client.wait_for_service()

# create a request message
request = serviseRequest()
request.data = ndarray()

# receive the response and store it
response = client(request)
