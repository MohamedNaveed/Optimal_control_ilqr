# Infinite horizon Trajectory generation for a pendulum system.
# Author: Mohamed Naveed Gul Mohamed
# email:mohdnaveed96@gmail.com
# Date: March 22 2023

import math as m

import numpy as np

T = 10

X0 = np.array([0,0])  #theta (rad), thetadot (rad/s)
Xg = np.array([180*m.pi/180,0]) 

class pendulum(object):

    def __init__(self, m=0.1, L=0.5, u_max=1, dt=0.1, nx=2, nu=1):

        self.m = m
        self.L = L
        self.u_max = u_max
        self.dt = dt
        self.nx = nx
        self.nu = nu
        self.g = 9.81

    def nonlinear_kinematics(self,state,U):


        state_n = np.zeros([0,0])

        state_n[0] = state[0] + state[1]*self.dt
        state_n[1] = state[1] - self.g*m.sin(state[0])*self.dt/self.L + U*self.dt/(self.m*self.L**2)

        state_n[0] = m.atan2(m.sin(state_n[0]),c.cos(state_n[0]))

        return state_n

class iLQR(object):

    def __init__(self, time_horizon, X0, Xg):

        '''
            Declare the matrices associated with the cost function
        '''

        self.Q = np.array([[1,0],[0,0.2]])
        self.Q_final = 900*np.array([[1,0],[0,0.1]])
        self.R = .01*np.ones((1,1))
        self.T = time_horizon
        self.X_0 = X0
        self.X_g = Xg

    def cost(self, x, u):
		
        '''
        Incremental cost in terms of state and controls
        '''
        return (((x - self.X_g).T @ self.Q) @ (x - self.X_g)) + (((u.T) @ self.R) @ u)

    def cost_final(self, x):
        '''
            Cost in terms of state at the terminal time-step
        '''
        return (((x - self.X_g).T @ self.Q_final) @ (x - self.X_g))
    
    def initialize_traj(self, path=None):

		'''
		Initial guess for the nominal trajectory by default is produced by zero controls
		'''
		if path is None:
			
			for t in range(0, self.T):
				self.U_p[t] = np.random.normal(0, 0.001, (self.n_u, 1))#np.random.normal(0, 0.01, self.n_u).reshape(self.n_u,1)#DM(array[t, 4:6])
			
			np.copyto(self.U_p_temp, self.U_p)
			
			self.forward_pass_sim()
			
			np.copyto(self.X_p_temp, self.X_p)


if __name__=="__main__":
      

    # Declare other parameters associated with the problem statement
	
	alpha = .9
	n_iterations = 10