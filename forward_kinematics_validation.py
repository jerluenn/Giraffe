import time 
import numpy as np 
from matplotlib import pyplot as plt 
from matplotlib import animation 
from mpl_toolkits import mplot3d
from casadi import *
import rospy
import continuum_manipulator_3hole_v3 as cm 
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray

class validation:

    def __init__(self, q, testmode, pulley_radius):

        self.forward = rospy.Publisher('cmd_pos', Float32MultiArray, queue_size = 10)
        # self.pulley_radius = pulley_radius
        self.q = Float32MultiArray()
        self.q.data = q/pulley_radius
        self.testmode = testmode

        rospy.init_node('validate', anonymous=True)

        rate = rospy.Rate(25)

        while not rospy.is_shutdown():
            
            rate.sleep()
            self.publish()

    def publish(self):

        self.forward.publish(self.q)
        rospy.loginfo(self.q)


if __name__ == "__main__":

    #q is driven cables. 

    q = np.zeros(6)
    v = validation(q, True)
