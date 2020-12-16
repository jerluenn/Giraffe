import time 
import numpy as np 
from matplotlib import pyplot as plt 
from matplotlib import animation 
from mpl_toolkits import mplot3d
from casadi import *

class continuum():

    def __init__(self, n, s, r_disc, q):
        """
        n is the number of levels.
        l is the array of of cable lengths.
        s is the length of the curvature.
        r_disc is the radius of the disc.
        this code is meant for 3 hole/level continuum manipulators.
        """

        self.n = n 
        self.s = s 
        self.r_disc = r_disc
        self.q = q
        self.l = np.zeros([self.n, 3])
        self.epsilon = 1e-10
        self.phi = np.zeros(self.n)
        self.theta = np.zeros(self.n)

        assert len(s) == n
        assert len(s) == q.ndim

    def f_q_to_length(self):

        if self.n == 1: 


            self.l[0,0] = self.s - self.q[0]
            self.l[0,1] = self.s - self.q[1]
            self.l[0,2] = self.s - self.q[2]

        else:

            for i in range(self.n):

                self.l[i,0] = self.s[i] - self.q[i,0]
                self.l[i,2] = self.s[i] - self.q[i,2]
                self.l[i,1] = self.s[i] - self.q[i,1]

    def f_length_to_phitheta_l1(self):

        l = self.l[0]

        self.phi[0] = np.arctan2(np.sqrt(3)*(l[1]+l[2]-2*l[0]) , (3*(l[1]-l[2] + self.epsilon)))
        self.theta[0] = (self.s[0]*2*(np.sqrt(l[0]**2 + l[1]**2 + l[2]**2 - l[0]*l[1] - l[0]*l[2] - l[1]*l[2])) + self.epsilon)/(self.r_disc*(l[0] + l[1] + l[2]))

    def f_length_to_phitheta_l2(self):

        l = self.l[1]

        self.phi[1] = np.arctan2(np.sqrt(3)*(l[0]+l[1]-2*l[2]) , (3*(l[0]-l[1] + self.epsilon))) - np.pi
        self.theta[1] = (self.s[1]*2*(np.sqrt(l[0]**2 + l[1]**2 + l[2]**2 - l[0]*l[1] - l[0]*l[2] - l[1]*l[2])) + self.epsilon)/(self.r_disc*(l[0] + l[1] + l[2]))

    def f_phitheta_to_xyz(self, theta, phi, s):

        transform = np.array([[np.cos(theta)*np.cos(phi)**2 + np.sin(phi)**2, np.cos(phi)*np.sin(phi)*(np.cos(theta) - 1), np.cos(phi)*np.sin(theta), (s*np.cos(phi)*(1 - np.cos(theta)))/theta],
                    [np.cos(phi)*np.sin(phi)*np.cos(theta) - np.cos(phi)*np.sin(phi), np.sin(phi)**2*np.cos(theta) + np.cos(phi)**2, np.sin(phi)*np.sin(theta), (s*np.sin(phi)*(1-np.cos(theta)))/theta],
                    [-np.cos(phi)*np.sin(theta), -np.sin(phi)*np.sin(theta), np.cos(theta), (np.sin(theta)*s)/theta],
                    [0, 0, 0, 1]]) 

        
        return transform

    def compute_fk(self, verbose = False):

        self.f_q_to_length()
        self.f_length_to_phitheta_l1()
        self.f_length_to_phitheta_l2()
        self.transforms = []

        for i in range(self.n):

            T = self.f_phitheta_to_xyz(self.theta[i], self.phi[i], self.s[i])
            self.transforms.append(T)

        for i in reversed(range(self.n)):

            if i == 0:
                break

            T = np.dot(self.transforms[i-1] , T)

        
        self.T = T
        
        if verbose == True:

            print("Actuation States: ", self.q)
            print("Actuation Cable Lengths:", self.l)
            print("Theta Angles:", self.theta*180/np.pi)
            print("Phi Angles:", self.phi*180/np.pi)
            # print("Homogeneous Transform Matrix", self.T)
            # print("Transformation Matrices:", self.T)
            print("Position:", self.T[:,3])

        return T

class continuum_symbolic():
        """
        n is the number of levels.
        l is the array of of cable lengths.
        s is the default length of the curvature.
        s_ is ANY point on the flexible rod.
        r_disc is the radius of the disc.
        this code is meant for 3 hole/level continuum manipulators.
        """

        def __init__(self, n, s, pull_rad):

            self.n = n
            self.r_disc = SX.sym('r_d')
            self.epsilon = 1e-10
            self.flag = 0
            self.theta = SX.sym('th', n)
            self.theta_q = SX.sym('th_q', n)
            self.phi = SX.sym('ph', n)
            self.phi_q = SX.sym('ph_q', n)
            self.s = SX.sym('s', n)
            self.q = SX.sym('q', (3, self.n))
            self.l = SX.sym('l', (3, self.n))
            self.vel_flag = 0
            self.theta_dot = SX.sym('th_dot', n)
            self.phi_dot = SX.sym('ph_dot', n)
            self.x = SX.sym('x', 3)
            self.s_numerical = SX(s)
            self.pulley_radius = pull_rad
            self.functions = []

        def f_q_to_length(self):

            functions = [] 

            if self.n == 1: 

                self.l[0,0] = self.s - self.q[0,0]
                self.l[1,0] = self.s - self.q[1,0]
                self.l[2,0] = self.s - self.q[2,0]

                functions.append(Function('f_qtolength', [self.s, self.q] , [self.l]))

            else:

                for i in range(self.n):

                    self.l[0,i] = self.s[i] - self.q[0,i]
                    self.l[1,i] = self.s[i] - self.q[1,i]
                    self.l[2,i] = self.s[i] - self.q[2,i]

                    functions.append(Function('f_qtolength', [self.s, self.q], [self.l]))

            return functions

        def f_length_to_phitheta_l1(self):

            l = self.l[:,0]

            self.phi_q[0] = arctan2(sqrt(3)*(l[1]+l[2]-2*l[0]), (3*(l[1]-l[2]) + self.epsilon))
            self.theta_q[0] = (self.s[0]*2*(sqrt(l[0]**2 + l[1]**2 + l[2]**2 - l[0]*l[1] - l[0]*l[2] - l[1]*l[2])) + self.epsilon)/(self.r_disc*(l[0] + l[1] + l[2]))

            self.functions.append(Function('f_lengthtophitheta', [self.s, self.q], [self.phi_q[0], self.theta_q[0]]))

            return None

        def f_length_to_phitheta_l2(self):

            l = self.l[:,1]

            self.phi_q[1] = arctan2(sqrt(3)*(l[0]+l[1]-2*l[2]), (3*(l[0]-l[1]) + self.epsilon)) - pi
            self.theta_q[1] = (self.s[1]*2*(sqrt(l[0]**2 + l[1]**2 + l[2]**2 - l[0]*l[1] - l[0]*l[2] - l[1]*l[2])) + self.epsilon)/(self.r_disc*(l[0] + l[1] + l[2]))

            self.functions.append(Function('f_lengthtophitheta', [self.s, self.q], [self.phi_q[1], self.theta_q[1]]))

            return None

        def f_phitheta_to_xyz(self, phi, theta, s):

            Transform = SX.eye(4)

            transform = [[cos(theta)*cos(phi)**2 + sin(phi)**2, cos(phi)*sin(phi)*(np.cos(theta) - 1), cos(phi)*np.sin(theta),  s*(-theta**3 + 12*theta)*cos(phi)/24],
                        [cos(phi)*sin(phi)*(cos(theta) - 1), sin(phi)**2*cos(theta) + cos(phi)**2, sin(phi)*sin(theta), s*(-theta**3 + 12*theta)*sin(phi)/24],
                        [-cos(phi)*sin(theta), -sin(phi)*sin(theta), cos(theta), s*(theta**4 - 20*theta**2 + 120)/120],
                        [0, 0, 0, 1]]

            for i in range(len(transform)):

                for k in range(4):

                    Transform[i,k] = transform[i][k]

            f_transform = Function('f_transform', [phi, theta, s], [Transform])

            return f_transform, Transform

        def f_phitheta_to_xyz_q(self, phi, theta, s):

            Transform = SX.eye(4)

            transform = [[cos(theta)*cos(phi)**2 + sin(phi)**2, cos(phi)*sin(phi)*(np.cos(theta) - 1), cos(phi)*np.sin(theta), (s*cos(phi)*(1 - cos(theta)))/(theta + self.epsilon)],
                        [cos(phi)*sin(phi)*(cos(theta) - 1), sin(phi)**2*cos(theta) + cos(phi)**2, sin(phi)*sin(theta), (s*sin(phi)*(1-cos(theta)))/(theta + self.epsilon)],
                        [-cos(phi)*sin(theta), -sin(phi)*sin(theta), cos(theta), (sin(theta)*s)/(theta + self.epsilon)],
                        [0, 0, 0, 1]]

            for i in range(len(transform)):

                for k in range(4):

                    Transform[i,k] = transform[i][k]

            f_transform = Function('f_transform', [self.q, self.s, self.r_disc], [Transform])

            return Transform, f_transform

        def compute_symbolic_fk(self):

            self.vel_flag = 1 #only when vel_flag is 1, velocity_fk can go through.
            self.transforms = []

            for i in range(self.n):

                transform = self.f_phitheta_to_xyz(self.phi[i], self.theta[i], self.s[i])[1]
                self.transforms.append(transform)

            for i in reversed(range(self.n)):

                if i == 0:
                    break

                Forward_Transform = mtimes(self.transforms[i-1], transform)

            if self.n == 1:

                Forward_Transform = transform

            f_transform = Function('f_transform', [self.s, self.theta, self.phi], [Forward_Transform])

            return Forward_Transform, f_transform

        def compute_fk(self):

            self.f_q_to_length()
            self.f_length_to_phitheta_l1()
            self.f_length_to_phitheta_l2()
            self.transforms_q = []

            for i in range(self.n):

                transform =  self.f_phitheta_to_xyz_q(self.phi_q[i], self.theta_q[i], self.s[i])[0]
                self.transforms_q.append(transform)

            for i in reversed(range(self.n)):

                if i == 0:
                    break

                Forward_Transform = mtimes(self.transforms_q[i-1], transform)

            if self.n == 1:

                Forward_Transform = transform

            f_transform = Function('f_transform', [self.q, self.s, self.r_disc], [Forward_Transform])

            return f_transform

        def velocity_fk(self):

            self.tip_position = self.compute_symbolic_fk()[0][0:3,3]
            self.tip_velocity = mtimes(jacobian(self.tip_position, vertcat(self.theta, self.phi)), vertcat(self.theta_dot, self.phi_dot))
            self.angles = vertcat(self.theta_dot, self.phi_dot)

            fx = vertcat(self.tip_velocity, self.angles)

            fx = Function('f_x', [self.x, self.theta, self.phi, self.s, self.theta_dot, self.phi_dot], [fx])


            return fx


        def create_integrator(self):  

            fx = self.velocity_fk()
            x = vertcat(self.x, self.theta, self.phi)
            # fx = vertcat(self.tip_velocity, self.theta_dot, self.phi_dot)
            u = vertcat(self.s, self.theta_dot, self.phi_dot)

            dae = {'x': x, 'p': u, 'ode': fx(self.x, self.theta, self.phi, self.s, self.theta_dot, self.phi_dot)}
            opts = {'tf': 0.05} #Interval Length
            I = integrator('I', 'rk', dae, opts)

            return I 
            
        def f_thetaphidot_to_angvel_motor_l1(self, theta_dot, phi_dot, disc_radius, theta, phi, pulley_radius):

            """" Mapping from theta dot and phi dot to motor steps for level 1."""

            q_dot1 = disc_radius*theta_dot*np.cos(-np.pi/2 + phi) - disc_radius*theta*phi_dot*np.sin(-np.pi/2 + phi)
            q_dot2 = disc_radius*theta_dot*np.cos(-phi + 2*np.pi/3 + np.pi/2) - disc_radius*theta*phi_dot*np.sin(phi - 2*np.pi/3 - np.pi/2)
            q_dot3 = disc_radius*theta_dot*np.cos(-phi + 4*np.pi/3 + np.pi/2) - disc_radius*theta*phi_dot*np.sin(phi - 4*np.pi/3 - np.pi/2)

            steps1 = (q_dot1/pulley_radius)
            steps2 = (q_dot2/pulley_radius)
            steps3 = (q_dot3/pulley_radius)

            return np.array([steps1, steps2, steps3 ])

        def f_thetaphidot_to_angvel_motor_l2(self, theta_dot, phi_dot, disc_radius, theta, phi, pulley_radius):

            """" Mapping from theta dot and phi dot to motor steps for level 2."""

            q_dot1 = disc_radius*theta_dot*np.cos(-np.pi/6 + phi) - disc_radius*theta*phi_dot*np.sin(-np.pi/6 + phi)
            q_dot2 = disc_radius*theta_dot*np.cos(-phi + 5*np.pi/6) - disc_radius*theta*phi_dot*np.sin(phi - 5*np.pi/6)
            q_dot3 = disc_radius*theta_dot*np.cos(-phi + 3*np.pi/2) - disc_radius*theta*phi_dot*np.sin(phi - 3*np.pi/2)

            steps1 = (q_dot1/pulley_radius)
            steps2 = (q_dot2/pulley_radius)
            steps3 = (q_dot3/pulley_radius)

            return np.array([steps1, steps2, steps3 ])

        def f_thetaphidot_to_q_dot_l2(self, theta_dot, phi_dot, disc_radius, theta, phi):

            """" Mapping from theta dot and phi dot to q_dot for level 2."""

            q_dot1 = disc_radius*theta_dot*np.cos(-np.pi/6 + phi) - disc_radius*theta*phi_dot*np.sin(-np.pi/6 + phi)
            q_dot2 = disc_radius*theta_dot*np.cos(-phi + 5*np.pi/6) - disc_radius*theta*phi_dot*np.sin(phi - 5*np.pi/6)
            q_dot3 = disc_radius*theta_dot*np.cos(-phi + 3*np.pi/2) - disc_radius*theta*phi_dot*np.sin(phi - 3*np.pi/2)

            return np.array([q_dot1, q_dot2, q_dot3])

        def f_thetaphidot_to_q_dot_l1(self, theta_dot, phi_dot, disc_radius, theta, phi):

            """" Mapping from theta dot and phi dot to q_dot for level 1."""

            q_dot1 = disc_radius*theta_dot*np.cos(-np.pi/2 + phi) - disc_radius*theta*phi_dot*np.sin(-np.pi/2 + phi)
            q_dot2 = disc_radius*theta_dot*np.cos(-phi + 2*np.pi/3 + np.pi/2) - disc_radius*theta*phi_dot*np.sin(phi - 2*np.pi/3 - np.pi/2)
            q_dot3 = disc_radius*theta_dot*np.cos(-phi + 4*np.pi/3 + np.pi/2) - disc_radius*theta*phi_dot*np.sin(phi - 4*np.pi/3 - np.pi/2)

            # q_dot1 = disc_radius*theta_dot*np.cos(phi) - disc_radius*theta*phi_dot*np.sin(phi)
            # q_dot2 = disc_radius*theta_dot*np.cos(-phi + 2*np.pi/3) - disc_radius*theta*phi_dot*np.sin(-phi + 2*np.pi/3)
            # q_dot3 = disc_radius*theta_dot*np.cos(phi + 2*np.pi/3) - disc_radius*theta*phi_dot*np.sin(phi + 2*np.pi/3)


            return np.array([q_dot1, q_dot2, q_dot3])

        def qdot(self, qdot_, u):

            return qdot_

        def f_thetaphi_to_q(self):

            for i in range(self.n):

                self.q[0,i] = self.theta[i]*self.r_disc*np.cos(-np.pi/2 + self.phi[i])
                self.q[1,i] = self.theta[i]*self.r_disc*np.cos(-self.phi[i] + 2*np.pi/3 + np.pi/2)
                self.q[2,i] = self.theta[i]*self.r_disc*np.cos(-self.phi[i] + 4*np.pi/3 + np.pi/2)
                

if __name__ == "__main__":

    a = continuum(2, np.array([[0.24], [0.24]]), 0.014, np.array([[0.0, 0.0, 0.0],[0.01, 0.0, 0.0]]))
    a.compute_fk(verbose = True)
    a.s = np.array([0.2, 0.2])
    a.compute_fk(verbose=True)



    # a = continuum_symbolic(2, [0.24, 0.24], 0.014)

    # integ = (a.create_integrator())
    # fw = a.compute_fk()
    # q = SX([[0.0,0.0],[0.0,0],[0.0,0.01]])
    # s = SX([0.24, 0.24])
    # r_disc = SX([0.014])

    # abc = a.compute_symbolic_fk()[1]
    # print(fw(q,s,r_disc)[:,3])

    # print(a.f_thetaphidot_to_q_dot(-0.02, 0, 0.014, 0, -np.pi/2))

    # a.theta = [0.2897518, -0.65611067] #answer should be 0.0129357, -0.0987073, 0.458767
    # a.phi = [0.65749038, -0.52255376]
    # a.r_disc = 0.014 
    # a.s = [0.24, 0.24]

    # a.f_thetaphi_to_q()
    # print(a.q)
    # print(fw(SX(a.q) , SX(a.s), SX(a.r_disc)))



