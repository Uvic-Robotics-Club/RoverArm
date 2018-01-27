#!/usr/bin/env python
import rospy
from RoverArm.srv import *
import geometry_msgs.msg

count = 2
def test_service_func(req):
    global count
    return_msg = point_to_angleResponse()
    return_msg.angles.base_angle = req.destination.x + count
    return_msg.angles.lower_angle = req.destination.y
    return_msg.angles.upper_angle = req.destination.z
    count = count+1
    return return_msg


def test_service_server():
    global count
    count = 1
    rospy.init_node('test_service_node')
    s = rospy.Service('test_service', point_to_angle, test_service_func)
    rospy.spin()

if __name__ == '__main__':
    test_service_server()
