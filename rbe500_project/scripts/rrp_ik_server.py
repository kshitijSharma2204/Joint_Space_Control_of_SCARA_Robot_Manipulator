#!/usr/bin/env python

from rbe500_project.srv import rrpIK
import rospy
import numpy as np 
from math import pi, cos, sin, atan2, acos, sqrt

def handle_rrpIK(req):
    ex=req.x
    ey=req.y
    ez=req.z
    joints=[]

    return inverse_kinematics(ex, ey, ez)

def rrpIK_Server():
    rospy.init_node('rrpIK_server')
    s = rospy.Service('rrpIK',rrpIK,handle_rrpIK)
    print("Computed the joint parameters for the given position")
    rospy.spin()

def inverse_kinematics(x,y,z):

	l0 = 0.05
	l1 = 0.45
	l2 = 0.425
	l3 = 0.345
	l4 = 0.11
    
	q3 = l0 + l1 - l4 - z
	value_c = ((x**2 + y**2 - l2**2 - l3**2)/(2*l2*l3))
	value_c = round(value_c, 2)
	value_s = sqrt(1-value_c**2) 
	theta2 = atan2(value_s, value_c)
    
	theta3 = atan2(y, x) - atan2(l3*value_s, l2 + l3*value_c)
    
	print(" Joint Values are ".center(100,"-"))
	joint1 = theta3
	joint2 = theta2
	joint3 = q3
	joints = [joint1,joint2,joint3]
	
	return joints


if __name__=="__main__":
    
    rrpIK_Server()

