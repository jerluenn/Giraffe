import time
import serial
import numpy as np 
from matplotlib import pyplot as plt 
import random 
import continuum_manipulator_3hole_v3
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
import rospy

class SmartMotors:

    def __init__(self):

        self.ser = serial.Serial(port='/dev/ttyUSB0', baudrate=9600,
            bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE, timeout=120)

        print(self.ser)

        self.command_all("RUN") #initialise all the motors.
        self.command_all("MV") #initialise all motors to be in velocity mode.
        self.command_all("ADT=1000")
    

        rospy.init_node("motor", anonymous = True)

        self.mot_vel = rospy.Subscriber("cmd_vel", Float32MultiArray, self.motor_callback_vel)
        self.mot_pos = rospy.Subscriber("cmd_pos", Float32MultiArray, self.motor_callback_pos)

        rospy.spin()

    def motor_callback_pos(self, data):

        self.pos = data.data 
        self.command_pos(self.pos[0], 1)
        self.command_pos(self.pos[1], 2)
        self.command_pos(self.pos[2], 3)
        self.command_pos(self.pos[3], 4)
        self.command_pos(self.pos[4], 5)
        self.command_pos(self.pos[5], 6)

    def motor_callback_vel(self, data):

        self.vel = data.data 
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

    a = SmartMotors()
    # a.command_vel(1, 2)
    a.set_origin()
    # a.return_to_origin()
    initial = time.time()
    time.sleep(5)
    print(time.time() - initial)
    a.command_all("X")
