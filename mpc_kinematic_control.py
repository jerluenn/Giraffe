from casadi import * 
import time
import numpy as np 
from matplotlib import pyplot as plt 
from matplotlib import animation 
from mpl_toolkits import mplot3d

class manipulator:

    def __init__(self, r_disc, r_thickness, n, disc_length):

        self.r_disc = r_disc
        self.l = disc_length
        self.n = n 
        self.thick = r_thickness
        self.q = np.zeros([2])
        self.theta = np.zeros(n)
        self.alpha = np.zeros(n)
        self.cable_lengths = np.zeros([self.n, 4])

    def forward_kinematics(self, displacement):

        """ Calculates the forward kinematics of a discrete continuum manipulator, alpha_config and theta_config must have r_n terms in it! 
        """

        self.map_q_to_length(displacement)
        self.map_length_to_config()

        transform = self.h_transform()

        joint_x = [0]
        joint_y = [0]
        joint_z = [0]
        end_coord = np.array([0, 0, 0, 1])

        for i in range(self.n):
            for k in reversed(range(i+1)):
                end_coord = np.dot(transform[k], end_coord)
            # end_coord = np.dot(transform[i],end_coord)
            joint_x.append(end_coord[0])
            joint_y.append(end_coord[1])
            joint_z.append(end_coord[2])
            if i == self.n - 1:
                pass
            else:
                end_coord = np.array([0, 0, 0, 1])

        return end_coord, joint_x, joint_y, joint_z

    def h_transform(self):

        homogeneous_transformation = []

        for i in range(self.n):

            T = np.array([[np.cos(self.theta[i]), np.sin(self.alpha[i])*np.sin(self.theta[i]), np.cos(self.alpha[i])*np.sin(self.theta[i]), (self.l*np.sin(self.theta[i])*np.cos(self.alpha[i]))/2],
            [0, np.cos(self.alpha[i]), -np.sin(self.alpha[i]), (-self.l/2)*np.sin(self.alpha[i])],
            [-np.sin(self.theta[i]), np.sin(self.alpha[i])*np.cos(self.theta[i]), np.cos(self.alpha[i])*np.cos(self.theta[i]), self.l/2 + self.thick + (self.l*np.cos(self.theta[i])*np.cos(self.alpha[i]))/2],
            [0,0,0,1]])
            
            homogeneous_transformation.append(T)

        return homogeneous_transformation

    def map_length_to_config(self):

        for i in range(len(self.cable_lengths)):

            self.theta[i] = (self.cable_lengths[i,2] - self.cable_lengths[i,0])/(2*self.r_disc)
            self.alpha[i] = (self.cable_lengths[i,1] - self.cable_lengths[i,3])/(2*self.r_disc)
        
        return None

    def map_q_to_length(self, displacement):

        """ Maps actuation state to cable lengths. """
        
        self.q = displacement

        for i in range(self.n):
            self.cable_lengths[i,0] = self.l - self.q[0]/self.n
            self.cable_lengths[i,1] = self.l - self.q[1]/self.n
            self.cable_lengths[i,2] = self.l + self.q[0]/self.n
            self.cable_lengths[i,3] = self.l + self.q[1]/self.n

        return None
 

class manipulator_symbolic:

    """Creates a manipulator in casadi's symbolic terms only."""

    def __init__(self, n):

        self.r_disc = MX.sym('r')
        self.l = MX.sym('l')
        self.n = n 
        self.thick = MX.sym('thick')
        self.q = MX.sym('q', 2)
        self.theta = MX.sym('theta', n)
        self.alpha = MX.sym('alpha', n)
        self.cable_lengths = MX.sym('cable_lengths', (n,4))
        self.q_dot = MX.sym('q_dot',2)

    def map_length_to_config(self):

        for i in range(self.n):

            self.theta[i] = (self.cable_lengths[i,2] - self.cable_lengths[i,0])/(2*self.r_disc)
            self.alpha[i] = (self.cable_lengths[i,1] - self.cable_lengths[i,3])/(2*self.r_disc)
        
        return None

    def map_q_to_length(self):

        """ Maps actuation state to cable lengths. """


        for i in range(self.n):
            self.cable_lengths[i,0] = self.l - self.q[0]/self.n
            self.cable_lengths[i,1] = self.l - self.q[1]/self.n
            self.cable_lengths[i,2] = self.l + self.q[0]/self.n
            self.cable_lengths[i,3] = self.l + self.q[1]/self.n

        return None
    
    def h_transform(self):

        homogeneous_transformation = []

        for i in range(self.n):

            T = MX.zeros(4, 4)
            k = 0 

            T_ = [[MX.cos(self.theta[i]), MX.sin(self.alpha[i])*MX.sin(self.theta[i]), MX.cos(self.alpha[i])*MX.sin(self.theta[i]), (self.l/2)*MX.sin(self.theta[i])*MX.cos(self.alpha[i])/2],
            [0, MX.cos(self.alpha[i]), -MX.sin(self.alpha[i]), -(self.l/2)*MX.sin(self.alpha[i])/2],
            [-MX.sin(self.theta[i]), MX.sin(self.alpha[i])*MX.cos(self.theta[i]), MX.cos(self.theta[i])*MX.cos(self.alpha[i]), self.l/2 + (self.l/2)*MX.cos(self.theta[i])*MX.cos(self.alpha[i])/2],
            [0,0,0,1]]


            for i in T_: 
                for j in range(len(i)):
                    T[k,j] = T_[k][j]     
                k += 1

            homogeneous_transformation.append(T) 

        return homogeneous_transformation

    def velocity_kinematics(self):

        """Determine x y z velocity using the Jacobian.""" 

        self.map_q_to_length()
        self.map_length_to_config()

        transform = self.h_transform()
        end_coord = MX([0, 0, 0, 1])

        for i in reversed(range(self.n)):

            end_coord = mtimes(transform[i],end_coord) 

        velocity = mtimes(jacobian(end_coord, self.q), self.q_dot)[0:3]

        return velocity

# m = manipulator(0.016, 0.004, 4, 0.028)
# end, xdata, ydata, zdata = m.forward_kinematics(np.array([0.0, -0.0]))
# print(end)
# fig = plt.figure()
# ax = plt.axes(projection='3d')
# ax.set_xlabel('x')
# ax.set_ylabel('y')
# ax.set_zlabel('z')
# ax.set_zlim3d(0, 0.2)
# ax.set_ylim3d(-0.1, 0.1)
# ax.set_xlim3d(-0.1, 0.1)
# ax.scatter3D(xdata, ydata, zdata, cmap='Greens')
# ax.plot3D(xdata, ydata, zdata)
# plt.show()

mpc = manipulator_symbolic(5)
V = mpc.velocity_kinematics()



