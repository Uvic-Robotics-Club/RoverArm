#!/usr/bin/env python
import rospy
from RoverArm.srv import *
import geometry_msgs.msg
import sys,os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'Utilities/Robotic Arm Class'))
from Arm import DisplayArm
import numpy as np

arm = DisplayArm([0.001, 3, 2, 1.5],
					 units=0.3048,
					 q1=45.0/360*2*np.pi,
					 q2=60.0/360*2*np.pi,
					 q3=-30.0/360*2*np.pi,
					 q4=-30.0/360*2*np.pi)

def get_angle_from_point(req):
    global arm
    results = arm.inverse_kinematics([req.destination.x,req.destination.y,req.destination.z])
    return_msg = point_to_angleResponse()
    return_msg.angles.base_angle = results[0]
    return_msg.angles.lower_angle = results[1]
    return_msg.angles.upper_angle = results[2]

    return return_msg


def test_service_server():
    global arm
    rospy.init_node('arm_inverse_kinematics_service')
    s = rospy.Service('arm_ik', point_to_angle, get_angle_from_point)
    rospy.spin()

if __name__ == '__main__':
    test_service_server()
