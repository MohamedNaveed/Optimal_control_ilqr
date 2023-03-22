import numpy as np


horizon = 30
state_dimemsion = 2
control_dimension = 1



Q = 0*np.array([[1,0],[0,0.2]])
Q_final = 900*np.array([[1,0],[0,0.1]])
R = .01*np.ones((1,1))


W_x_LQR = 10*np.eye(state_dimemsion)
W_u_LQR = 2*np.eye(control_dimension)
W_x_LQR_f = 100*np.eye(state_dimemsion)

feedback_samples_no = 10