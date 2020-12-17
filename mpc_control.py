import time
import serial
import numpy as np 
from matplotlib import pyplot as plt 
import random 
import continuum_manipulator_3hole_v3
from casadi import *
import acado
import rospy
import continuum_manipulator_3hole_v3
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray

class mpc:

    def __init__(self):

        self.ser = serial.Serial(port='/dev/ttyUSB0', baudrate=9600,
            bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE, timeout=120)

        # print(self.ser)

        self.command_all("RUN") #initialise all the motors.
        self.command_all("MV") #initialise all motors to be in velocity mode.
        self.command_all("ADT=1000")

        self.cmd_vel = rospy.Publisher('cmd_vel', Float32MultiArray, queue_size=10)
        self.state_pub = rospy.Publisher('states', Float32MultiArray, queue_size=10)

        self.T=10
        self.NX=7
        self.NU=4
        self.samplingTime = 0.05 
        self.x0 = np.zeros((1, self.NX))
        self.x0[:,2] = 0.364
        self.x0[:,1] = 0.0
        self.x0[:,0] = 0.0

        self.X = np.zeros((self.T+1,self.NX))
        self.U = np.zeros((self.T,self.NU))
        self.Y = np.zeros((self.T,self.NX))
        self.yN = np.zeros((1,3))

        self.Q = np.diag([250, 250, 250, 0.2, 0.2, 0.3, 0.3])  # state cost matrix
        self.Qf = np.diag([500, 500, 500])


        rospy.init_node('mpc', anonymous=True)
        rate = rospy.Rate(25)

        init_time = time.time()

        while not rospy.is_shutdown():

            rate.sleep()
            r = 0.04
            om = 0.5
            xref = r*np.sin(om*(init_time-time.time()))
            yref = r*np.cos(om*(init_time-time.time()))
            self.solve_mpc(np.array([xref, yref, 0.32]))
            self.motor_callback_vel(self.sol)


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

        sol1 = continuum.f_thetaphidot_to_angvel_motor_l1(self.u[0], self.u[2], 0.014, self.x0[0][3], self.x0[0][5], 0.026)   
        sol2 = continuum.f_thetaphidot_to_angvel_motor_l2(self.u[1], self.u[3], 0.014, self.x0[0][4], self.x0[0][6], 0.026)

        self.sol = np.concatenate((sol1,sol2), axis=None)

        vel = Float32MultiArray()
        vel.data = self.sol

        state = Float32MultiArray()
        state.data = self.X[1,:]     

        self.cmd_vel.publish(vel)
        self.state_pub.publish(state)

        rospy.loginfo(state)
        rospy.loginfo(self.sol)

    def motor_callback_pos(self, data):

        self.pos = data.data 
        self.command_pos(self.pos[0], 1)
        self.command_pos(self.pos[1], 2)
        self.command_pos(self.pos[2], 3)
        self.command_pos(self.pos[3], 4)
        self.command_pos(self.pos[4], 5)
        self.command_pos(self.pos[5], 6)

    def motor_callback_vel(self, data):

        self.vel = data
        self.command_vel(self.vel[0], 1)
        self.command_vel(self.vel[1], 2)
        self.command_vel(self.vel[2], 3)
        self.command_vel(self.vel[3], 4)
        self.command_vel(self.vel[4], 5)
        self.command_vel(self.vel[5], 6)

    def command_all(self, line):

        assert type(line) == str 
        a = chr(128)
        c = chr(32)
        comm = a+str.encode(line)+c
        comm = bytes(comm)
        print(comm)
        self.ser.write(comm)

        return None 

    def command(self, line, motor_no):

        a = chr(128+motor_no)
        c = chr(32)
        comm = a+str.encode(line)+c
        comm = bytes(comm)
        print(comm)
        self.ser.write(comm)

        return None

    def command_vel(self, vel, motor_no):

        # print(type(vel))
        vel = (vel*65536.0/(4*np.pi))
        line = "VT=%d" %vel #vel is in terms of radians per second.
        self.command(line, motor_no)
        self.command("G", motor_no)

        return None

    def stop(self, motor_no):

        line = "X"
        self.command(line, motor_no)

    def return_to_origin(self):

        self.command_all("MP")
        self.command_all("VT=10000")
        self.command_all("PT=0")
        self.command_all("G")

    def set_origin(self):
        self.command_all("MT")
        self.command_all("T=2500")
        self.command_all("G")
        time.sleep(5)
        self.command_all("O=0")
        self.command_all("X")
        self.command_all("MP")
        self.command_all("PT=0")
        self.command_all("G")

    def command_pos(self, pos, motor_no):

        pos = pos*65536/(4*np.pi)
        line = "PT=%d" %pos
        self.command(line, motor_no)
        self.command("G", motor_no)

if __name__ == "__main__":

    continuum = continuum_manipulator_3hole_v3.continuum_symbolic(2, [0.182, 0.182], 0.014)
    a = mpc()