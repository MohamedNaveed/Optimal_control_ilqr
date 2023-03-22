# Infinite horizon Trajectory generation for a cart-pole system.
# Author: Mohamed Naveed Gul Mohamed
# email:mohdnaveed96@gmail.com
# Date: March 22 2023

import casadi as c
import math as m

import numpy as np

T = 10

X0 = np.array([0,0,2*m.pi/180,0]) #x, xdot, theta (rad), thetadot (rad/s)
Xg = np.array([0,0,0,0])

class cart_pole(object):

    def __init__(self, M=1, m=0.01, L=0.6, u_max=1, dt=0.1, nx=4, nu=1):

        self.M = M
        self.m = m
        self.L = L
        self.u_max = u_max
        self.dt = dt
        self.nx = nx
        self.nu = nu
        self.x = c.MX.sym('x', self.nx,1)
        self.u = c.MX.sym('u', self.nu,1)
        self.g = 9.81

    def linear_process_model(self):

        h = c.MX(self.nx,self.nx)
        h[0,0] = 1; h[0,1] = self.dt; h[0,2] = 0; h[0,3] = 0;
        h[1,0] = 0; h[1,1] = 1; h[1,2] = 0; h[1,3] = 0;
        h[2,0] = 0; h[2,1] = 0; h[2,2] = 1; h[2,3] = self.dt;
        h[3,0] = 0; h[3,1] = 0; h[3,2] = self.g*self.dt/self.L; h[3,3] = 1;

        g = c.MX(self.nx,self.nu)
        g[0,0] = 0; g[1,0] = 1/self.M; g[2,0] = 0; g[3,0] = 1/(self.M*self.L)

        f = c.Function('f',[self.x,self.u],[c.mtimes(h,self.x) + c.mtimes(g,self.u)*self.dt])

        A = c.Function('A',[self.x,self.u],[c.jacobian(f(self.x,self.u),self.x)])
        B = c.Function('B',[self.x,self.u],[c.jacobian(f(self.x,self.u),self.u)])

        return f,A,B

    def nonlinear_model(self):

        h = c.MX(self.nx,1)
        x1 = self.x[0]
        x2 = self.x[1]
        theta1 = self.x[2]
        theta2 = self.x[3]
        u = self.u

        term1 = self.m/(self.M + self.m - self.m*(c.cos(theta1)**2)) #intermediate variable
        term2 = term1*(self.g*c.sin(theta1)*c.cos(theta1) - self.L*c.sin(theta1)*(theta2**2))

        h[0,0] = x2
        h[1,0] = term2

        h[2,0] = theta2
        h[3,0] = (self.g*c.sin(theta1)/self.L) + (c.cos(theta1)/self.L)*term2

        g = c.MX(self.nx,self.nu)
        g[0,0] = 0
        g[1,0] = term1*u/self.m
        g[2,0] = 0
        g[3,0] = (c.cos(theta1)/self.L)*term1*u/self.m

        f = c.Function('f',[self.x,self.u],[self.x + (h + g)*self.dt])

        A = c.Function('A',[self.x,self.u],[c.jacobian(f(self.x,self.u),self.x)])
        B = c.Function('B',[self.x,self.u],[c.jacobian(f(self.x,self.u),self.u)])

        return f,A,B

    def kinematics(self,state,U):

        #f,_,_ = self.linear_process_model()
        f,_,_ = self.nonlinear_model()

        state_n = f(state,U)

        state_n[2] = c.atan2(c.sin(state_n[2]),c.cos(state_n[2]))

        return state_n

class SSP(object):

    def __init__(self, time_horizon, X0, Xg, R, Q, Qf):

        self.T = time_horizon
        self.X0 = X0
        self.Xg = Xg
        #self.Xmin = Xmin
        #self.Xmax = Xmax
        self.R = R
        self.Q = Q
        self.Qf = Qf

    def solve_SSP(self, robot, X0, Ui, U_guess):

        opti = c.Opti()

        U = opti.variable(robot.nu*self.T,1)
        opti.set_initial(U, U_guess)

        opti.minimize(self.cost_func_SSP(robot, U, X0))

        #control constraints
        #opti.subject_to(U <= self.U_upper_bound(robot))
        #opti.subject_to(U >= self.U_lower_bound(robot))

        #state constraints
        #opti.subject_to(self.state_contraints(robot,U) > 0)

        opts = {}
        opts['ipopt.print_level'] = 0
        opti.solver('ipopt', opts)
        try:
                sol = opti.solve()
                U_opti = sol.value(U)

        except RuntimeError:
                U_opti = opti.debug.value(U)
                print("debug value used.")
        U_opti = c.reshape(U_opti,robot.nu,self.T)

        return U_opti

    def cost_func_SSP(self, robot, U, X0):

        cost = 0

        U = c.reshape(U, robot.nu, self.T)

        X = c.MX(robot.nx,self.T+1)
        X[:,0] = X0

        for i in range(self.T):

            cost = (cost + c.mtimes(c.mtimes(U[i].T,self.R),U[i]) +
                        c.mtimes(c.mtimes((self.Xg - X[:,i]).T,self.Q),(self.Xg - X[:,i])))

            X[:,i+1] = robot.kinematics(X[:,i],U[i])

        cost = cost + c.mtimes(c.mtimes((self.Xg - X[:,self.T]).T,self.Qf),(self.Xg - X[:,self.T]))

        return cost

    def U_upper_bound(self, robot):

        ones = c.DM.ones(robot.nu,self.T)
        ub = robot.u_max*ones[0,:]
        ub = c.reshape(ub,robot.nu*self.T,1)

        return ub

    def U_lower_bound(self, robot):

        ones = c.DM.ones(robot.nu,self.T)
        lb = -robot.u_max*ones[0,:]
        lb = c.reshape(lb,robot.nu*self.T,1)

        return lb


if __name__=='__main__':

    R = c.DM([[5]])
    Q = 5*c.DM([[2,0,0,0],[0,.2,0,0],[0,0,8,0],[0,0,0,0.3]])
    Qf = c.DM([[3,0,0,0],[0,3,0,0],[0,0,10,0],[0,0,0,3]])

    robot = cart_pole()

    control = SSP(T, X0, Xg, R, Q, Qf)

    U_guess = np.zeros((robot.nu,T))

    U_guess = np.reshape(U_guess,(robot.nu*T,1))

    print("Solving state space problem...")
    U_opti_ss = control.solve_SSP(robot,X0, np.zeros(robot.nu), U_guess)
    print("Done.")
    #print("Uss:",U_opti_ss)
    X_path_ss = np.zeros((robot.nx,T+1))
    X_path_ss[:,0] = X0

    for i in range(T):

        X_path_ss[:,i+1] = np.reshape(robot.kinematics(X_path_ss[:,i],U_opti_ss[:,i]),(robot.nx,)) #to make it compatible in dimension reshaping from (3,1) to 3

    for i in range(T):
        print("t=", i, "    X_nom:", X_path_ss[:,i], "  U nom:", U_opti_ss[:,i], "\n")

    print("U nom = ", U_opti_ss)