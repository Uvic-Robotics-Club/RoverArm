#!/usr/bin/env python
import rospy
from RoverArm.srv import *
import geometry_msgs.msg
import sys,os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'Utilities/Robotic Arm Class'))
from Arm import DisplayArm
import numpy as np

arm = DisplayArm([0.001, 1, 1, 0.001],
					 units=1,
					 q1 = 90.0/360*2*np.pi,
					 q2 = 60.0/360*2*np.pi,
					 q3 = 30.0/360*2*np.pi,
					 q4 = 30.0/360*2*np.pi)

def get_angle_from_point(req):
    global arm
    results = arm.inverse_kinematics([req.destination.x,req.destination.y,req.destination.z])
    return_msg = point_to_angleResponse()
    return_msg.angles.base_angle = rad_to_deg(results[0])
    return_msg.angles.lower_angle = rad_to_deg(results[1])
    return_msg.angles.upper_angle = rad_to_deg(results[2])
    return return_msg
    
def rad_to_deg(angle):
    return (180.0/np.pi)*angle

def deg_to_rad(angle):
    return (np.pi/180.0)*angle

def get_points_from_angles(req):
    global arm
    [resultsx, resultsy, resultsz] = arm.forward_kinematics(deg_to_rad(req.angles.base_angle), deg_to_rad(req.angles.lower_angle), deg_to_rad(req.angles.upper_angle))
    return_msg = angles_to_pointsResponse()
    
    return_msg.origin.x = resultsx[0]
    return_msg.end_of_link_one.x = resultsx[1]
    return_msg.end_of_link_two.x = resultsx[2]
    return_msg.end_of_link_three.x = resultsx[3]
    
    return_msg.origin.y = resultsy[0]
    return_msg.end_of_link_one.y = resultsy[1]
    return_msg.end_of_link_two.y = resultsy[2]
    return_msg.end_of_link_three.y = resultsy[3]
    
    return_msg.origin.z = resultsz[0]
    return_msg.end_of_link_one.z = resultsz[1]
    return_msg.end_of_link_two.z = resultsz[2]
    return_msg.end_of_link_three.z = resultsz[3]
    
    return return_msg
    

def test_service_server():
    global arm
    rospy.init_node('arm_inverse_kinematics_service')
    s = rospy.Service('arm_ik', point_to_angle, get_angle_from_point)
    ss = rospy.Service('arm_fk', angles_to_points, get_points_from_angles)
    rospy.spin()

if __name__ == '__main__':
    test_service_server()
