#!/usr/bin/env python

import time
import numpy as np
from matplotlib import pyplot as plt
from matplotlib import animation
from mpl_toolkits import mplot3d
from casadi import *
import acado
import rospy
import continuum_manipulator_3hole_v3
from std_msgs.msg import Float32


class mpc:

    def __init__(self):

        self.motor1 = rospy.Publisher('vel1', Float32, queue_size=10)
        self.motor2 = rospy.Publisher('vel2', Float32, queue_size=10)
        self.motor3 = rospy.Publisher('vel3', Float32, queue_size=10)
        self.motor4 = rospy.Publisher('vel4', Float32, queue_size=10)
        self.motor5 = rospy.Publisher('vel5', Float32, queue_size=10)
        self.motor6 = rospy.Publisher('vel6', Float32, queue_size=10)   

        self.T=10
        self.NX=7
        self.NU=4
        self.samplingTime = 0.05 
        self.x0 = np.zeros((1, self.NX))
        self.x0[:,2] = 0.48
        self.x0[:,1] = 0.0
        self.x0[:,0] = 0.0

        self.X = np.zeros((self.T+1,self.NX))
        self.U = np.zeros((self.T,self.NU))
        self.Y = np.zeros((self.T,self.NX))
        self.yN = np.zeros((1,3))

        self.Q = np.diag([250, 250, 250, 0.2, 0.2, 0.2, 0.2])  # state cost matrix
        self.Qf = np.diag([500, 500, 500])


        rospy.init_node('mpc', anonymous=True)
        rate = rospy.Rate(25)

        while not rospy.is_shutdown():

            rate.sleep()
            self.solve_mpc(np.array([0.2, 0.2, 0.3]))


    def solve_mpc(self, ref):

        self.Y[:,0] = ref[0]
        self.Y[:,1] = ref[1]
        self.Y[:,2] = ref[2]

        self.yN[:,0] = ref[0]
        self.yN[:,1] = ref[1]
        self.yN[:,2] = ref[2]

        self.X, self.U = acado.mpc(0, 1, self.x0,self.X,self.U,self.Y,self.yN, np.transpose(np.tile(self.Q,self.T)), self.Qf, 0)
        self.x0 = np.array([self.X[1,:]])

        self.u = self.U[0,:] #in terms of theta and phi

        sol1 = continuum.f_thetaphidot_to_angvel_motor_l1(self.u[0], self.u[2], 0.014, self.x0[0][3], self.x0[0][5], 0.05)   
        sol2 = continuum.f_thetaphidot_to_angvel_motor_l2(self.u[1], self.u[3], 0.014, self.x0[0][4], self.x0[0][6], 0.05)

        self.motor1.publish(sol1[0])
        self.motor2.publish(sol1[1])
        self.motor3.publish(sol1[2])
        self.motor4.publish(sol2[0])
        self.motor5.publish(sol2[1])
        self.motor6.publish(sol2[2])

        rospy.loginfo(sol1)
        rospy.loginfo(sol2)


if __name__ == "__main__":

    continuum = continuum_manipulator_3hole_v3.continuum_symbolic(2, [0.24, 0.24], 0.014)
    c = mpc()
    
