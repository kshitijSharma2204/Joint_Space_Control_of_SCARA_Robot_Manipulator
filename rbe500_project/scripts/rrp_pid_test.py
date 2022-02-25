#!/usr/bin/env python

import math
from os import error

from matplotlib import colors
import rospy
from rbe500_project.srv import *
from rospy.timer import Rate
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from math import pi
import matplotlib.pyplot as plt
import numpy as np
import csv

current_j1 = 0
current_j2 = 0
current_j3 = 0
trajactory1 = list()
trajactory2 = list()
trajactory3 = list()

tol = 0.05
tol_dist = 0.001

fields = ['Sno','J1_SP','J1_CP']

heading = ["No.","J1_SP","J1_CP","J2_SP","J2_CP","J3_SP","J3_CP"]
np.savetxt('trajactory.csv' , np.array(heading), fmt='%s', delimiter=',')

class rrp_controller:
    def __init__(self, p= 0.0 , d = 0.0, i = 0.0, setpoint = 0.0, i_sum = 0.0,i_max = 500, i_min = -500):

        self.Kp = p
        self.Kd = d
        self.Ki = i
        self.setpoint = setpoint
        self.i_sum = i_sum 
        self.t0_error = 0
        self.error = 0
        

        self.i_max = i_max
        self.i_min = i_min

    def update(self, current_value):

        error1 = self.setpoint - current_value
        #print(error1)
        error2 = lim_angle(2*math.pi + error1)
        #print(error2)
        if (abs(error1)<abs(error2)):
            self.error = error1
        else:
            self.error = error2

        self.i_sum = self.i_sum + self.error

        if self.i_sum > self.i_max:
            self.i_sum = self.i_max
        elif self.i_sum < self.i_min:
            self.i_sum = self.i_min

        P_part = self.Kp * self.error
        D_part = self.Kd * (self.error - self.t0_error)
        I_part = self.i_sum * self.Ki

        self.t0_error = self.error

        PID_calc = P_part+D_part+I_part
        return PID_calc

    def setPID(self, P=0.0, I=0.0, D=0.0):
        self.Kp = P
        self.Kd = D
        self.Ki = I

    def setPoint(self, set_point):
        self.setpoint = set_point
        self.previous_error = 0
        self.Integrator = 0

    def get_error(self):
        return self.error

    
    def setIntegrator(self, Integrator, i_max, i_min):
        self.Integrator = Integrator
        self.i_max = i_max
        self.i_min = i_min

    



def main():

    global current_j1
    global current_j2
    global current_j3
    global trajactory1
    global trajactory2
    global trajactory3
    global tol
    global tol_dist

    p1 = [0, 0.77, 0.34]
    p2 = [-0.345, 0.425, 0.24]
    p3 = [-0.67, -0.245, 0.14]
    p4 = [0.77, 0.0, 0.39]

    pos = [ p4 ]

    joint1_c = rrp_controller()
    joint2_c = rrp_controller()
    joint3_c = rrp_controller()

    joint1_c.setPID(0.15,0.0001,25)
    joint2_c.setPID(1,0.0005,20)
    joint3_c.setPID(600,0.001,20)


    rospy.init_node("listner", anonymous=True)
    rospy.Subscriber("/rrp/joint_states", JointState, callback)
    
    pub1 = rospy.Publisher('/rrp/joint1_effort_controller/command', Float64, queue_size = 10)
    pub2 = rospy.Publisher('/rrp/joint2_effort_controller/command', Float64, queue_size = 10)
    pub3 = rospy.Publisher('/rrp/joint3_effort_controller/command', Float64, queue_size = 10)

    Rate = rospy.Rate(100)

    for i in pos:

        j = IK_client(i[0], i[1], i[2])

        joint1_c.setPoint(j.joint1)
        joint2_c.setPoint(j.joint2)
        joint3_c.setPoint(j.joint3)

        x = 0
        #fig,axis = plt.subplots(2,2)
        #print("errors: ", j.joint1 - lim_angle(current_j1), j.joint2 - lim_angle(current_j2), j.joint3 - lim_angle(current_j3))
        Reached = False

        while not Reached:
            rospy.Subscriber("rrp/joint_states", JointState, callback)

            joint1_c.update(current_j1)
            joint2_c.update(current_j2)
            joint3_c.update(current_j3)

            if abs(joint1_c.get_error()) > tol:

                pub1.publish((lim_effort(joint1_c.update(lim_angle(current_j1)))))
                pub2.publish(0)
                pub3.publish(0)
            
            elif abs(joint1_c.get_error()) < tol and abs(joint2_c.get_error()) > tol:

                pub2.publish((lim_effort(joint2_c.update((current_j2)))))
                pub1.publish(0)
                pub3.publish(0)
                
            elif abs(joint1_c.get_error()) < tol and abs(joint2_c.get_error()) < tol and abs(joint3_c.get_error()) > tol_dist:
                
                pub3.publish(joint3_c.update(current_j3))
                pub1.publish(0)
                pub2.publish(0)

            else:
                Reached = True
                pub1.publish(0)
                pub2.publish(0)
                pub3.publish(0)

            Rate.sleep()

## WHILE RUNNING DO NOT UNCOMMENT THESE LINE ELSE IT WILL TAKE LONG TO CONVERGE

            #axis[0,0].scatter(x,j.joint1,color="green",s=3)
            #axis[0,0].scatter(x,current_j1,color="red",s=1.5)
            #axis[0,1].scatter(x,j.joint2,color="green",s=3)
            #axis[0,1].scatter(x,current_j2,color="red",s=1.5)
            #axis[1,0].scatter(x,j.joint3,color="green",s=3)
            #axis[1,0].scatter(x,current_j3,color="red",s=1.5)

            rospy.loginfo("Joint1 - %s", current_j1)
            rospy.loginfo("")
            rospy.loginfo("Joint2 - %s", current_j2)
            rospy.loginfo("")
            rospy.loginfo("Joint3 - %s", current_j3)
            rospy.loginfo("")

            #plt.xlabel('Iterations')
            #plt.ylabel('Joint value')

            #plt.plot(x,current_j1,'.',color = 'blue')
            #plt.plot(x,joint1_c,'.',color = 'red')

## UNCOMMENT FOR TRAJACTORY CSV FILE CREATION
## COMMENTED AS CONVERGENCE TAKES LONG TIME 

            #trajactory1.append([x,j.joint1,current_j1])
            #trajactory2.append([x,j.joint2,current_j2])
            #trajactory3.append([x,j.joint3,current_j3])
            #trajactory_final = np.column_stack((trajactory1,trajactory2,trajactory3))
            #np.savetxt('trajactory.csv' , np.array(trajactory_final), fmt='%f', delimiter=',')

            x+=1

        plt.show()
        #print(trajactory1)

        #np.savetxt('trajactory1.csv' , np.array(trajactory1), fmt='%f', delimiter=',')
        #np.savetxt('trajactory2.csv' , np.array(trajactory2), fmt='%f', delimiter=',')
        #np.savetxt('trajactory3.csv' , np.array(trajactory3), fmt='%f', delimiter=',')
        
        pub1.publish(0)
        pub2.publish(0)
        pub3.publish(0)
        
        print("Reached position ")
        rospy.sleep(1)

##

def lim_pi(value):
    if value >math.pi:
        value-=math.pi
        
    elif value<-math.pi:
        value+=math.pi


def callback(data):
    global current_j1, current_j2, current_j3
    current_j1 = data.position[0]
    current_j2 = data.position[1]
    current_j3 = data.position[2]
    #rospy.loginfo(current_j1)
    #rospy.loginfo(data.position[0])

def IK_client(x,y,z):
    rospy.wait_for_service('rrpIK')
    try:
        joint = rospy.ServiceProxy('rrpIK',rrpIK)
        joints= joint(x,y,z)
        return joints

    except rospy.ServiceException as e:
        print("Service error : %s"%e)

def lim_effort(val):
    if val > 1:
        val = 1
    elif val < -1:
        val = -1
    return val

def lim_angle(angle):
    if angle >2*math.pi:
        return angle-2*math.pi
    elif angle<-2*math.pi:
        return angle+2*math.pi
    
    return angle
##

if __name__ == "__main__":

    main()