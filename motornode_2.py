import time
import serial
import numpy as np 
from matplotlib import pyplot as plt 
import random 
import continuum_manipulator_3hole_v3
from std_msgs.msg import Float32
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

        self.mot1 = rospy.Subscriber("vel1", Float32, self.mot1_callback)
        self.mot2 = rospy.Subscriber("vel2", Float32, self.mot2_callback)
        self.mot3 = rospy.Subscriber("vel3", Float32, self.mot3_callback)
        self.mot4 = rospy.Subscriber("vel4", Float32, self.mot4_callback)
        self.mot5 = rospy.Subscriber("vel5", Float32, self.mot5_callback)
        self.mot6 = rospy.Subscriber("vel6", Float32, self.mot6_callback)

        rospy.spin()

    
    def mot1_callback(self, data):

        self.vel1 = data.data
        self.command_vel(self.vel1, 1)
    
    def mot2_callback(self, data):

        self.vel2 = data.data
        self.command_vel(self.vel2, 2)

    def mot3_callback(self, data):

        self.vel3 = data.data
        self.command_vel(self.vel3, 3)

    def mot4_callback(self, data):

        self.vel4 = data.data
        self.command_vel(self.vel4, 4)

    def mot5_callback(self, data):

        self.vel5 = data.data
        self.command_vel(self.vel5, 5)
    
    def mot6_callback(self, data):

        self.vel6 = data

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

    def command_pos(self, pos, motor_no)

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





# ser = serial.Serial(port='ttyUSB0', baudrate=9600,
#             bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
#             stopbits=serial.STOPBITS_ONE, timeout=120)

# print("Serial Port Status Open: ",ser.isOpen())

# def read_integer():
#     currentString = ''
#     currentChar = None 
#     while (currentChar!=' ' and currentChar!='\n' and currentChar!='\r'):
#         currentChar = ser.read()
#         # currentChar = currentChar.decode()
#         # currentChar = unicode(currentChar, errors = 'ignore')
#         # print(currentString)
#         currentString += currentChar
#     try:
#         return (float(currentString)) 
#     except:
#         pass    


# initialtime = time.time()

# # print(initialtime)

# # ser.write(b'1RPA ')
# # print(read_integer())
# # print(read_integer())
# # ser.writelines('PRINT(ADDR)\n\r')
# # print(read_integer())
# # ser.write(b'0WAKE\n\r')
# # ser.write(b'0ZS\n\r')
# # ser.write(b'0EIGN(2)\n\r')
# # ser.write(b'0EIGN(3)\n\r')
# # ser.write(b'1MP\n\r')
# # ser.write(b'1ADT=100\n\r')
# # ser.write(b'1VT=100000\n\r')
# # ser.write(b'1PT=%d\n\r' % pos_d)
# # ser.write('1PT=100000\n\r')
# # time.sleep(1)
# # time.sleep(1)
# # ser.write(b"1G\n\r")
# # time.sleep(1)

# # ser.write(b" ")
# # ser.write(b"\n\r")
# # ser.write(b"1PRT=8000 ")
# # ser.write(b"G ")
# # print(chr(129 71 32))
# # x = chr()
# a = chr(128)
# b = chr(71)
# c = chr(32)
# aa = chr(130)
# # print(a_)
# cc_ = str.encode(a)+str.encode("RUN")+str.encode(c)
# # ser.write(cc_)
# b_ = str.encode(a)+str.encode("EIGN(2)")+str.encode(c)
# c_ = str.encode(a)+str.encode("EIGN(3)")+str.encode(c)
# a_ = str.encode(a)+str.encode("ZS")+str.encode(c)
# ser.write(b_)
# ser.write(c_)
# ser.write(a_)
# f_ = str.encode(a)+str.encode("MV")+str.encode(c)
# d_ = str.encode(a)+str.encode("VT=2000")+str.encode(c)
# l_ = str.encode(aa)+str.encode("VT=100000")+str.encode(c)
# e_ = str.encode(a)+str.encode("G ")
# g_ = str.encode(a)+str.encode("ADT=100")+str.encode(c)
# ser.write(d_)
# ser.write(f_)
# ser.write(g_)
# ser.write(l_)
# ser.write(e_)
# ser.write(b"1G ")


# while time.time() - initialtime < 5:

#     pass
#     # rpa = str.encode(a)+str.encode("RPA ")
#     # ser.write(rpa)
#     # print(read_integer())

# stop = str.encode(a)+str.encode("X ")

# ser.write(stop)
# print("End")
# ser.write(b"129 71 32")

# print("test")
# # ser.write(b'0ZS ')

# string = "0ZS "
# a = str.encode(string)
# print((a))
# ser.write(a)


# print("pos_d",pos_d)

# disp = []
# vel = []
# t = []

# while time.time() - initialtime < 10: 
#     ser.write(b'1RPA ')
#     displacement = read_integer()
#     # print('test')
#     print("displacement: ", displacement/4000, 'revs')
#     ser.write(b'1RVA ')
#     velocity = read_integer()
#     # print("velocity: ",  velocity/32768, 'rev/s')
#     t.append(time.time() - initialtime)
#     disp.append(displacement/4000)
#     vel.append(velocity/32768)
#     if displacement == pos_d:
#         break



# #1000 encoder counts is approximately pi/2 radians.

# ser.write(b'X\n\r')

# plt.plot(t, disp)
# plt.xlabel('t')
# plt.ylabel('disp (rev)')
# plt.show()

# plt.plot(t, vel)
# plt.xlabel('t')
# plt.ylabel('vel (rev/s)')
# plt.show()
