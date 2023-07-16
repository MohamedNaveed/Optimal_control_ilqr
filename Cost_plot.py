import numpy as np
import matplotlib.patches as fill
import pylab
import math
import time
from random import uniform
import matplotlib.pyplot as plt

params = {'axes.labelsize':16,
            'font.size':12,
            'legend.fontsize':14,
            'xtick.labelsize':14,
            'ytick.labelsize':14,
            'text.usetex':True,
            'figure.figsize':[4.5,3]}
pylab.rcParams.update(params)

PLOT_MPC = True #False #
PLOT_DP = True


Noise_level = [0,10] #file index 1.6 - 64 1.0 - 54 .4-42

if __name__=='__main__':

    if PLOT_MPC:
        
        filename_mpc = "/home/naveed/Documents/Optimal_control_ilqr/mpc_1dcos_T200.csv"
        file_mpc = open(filename_mpc,"r")

        lines = file_mpc.read().splitlines()

        for i in range(len(lines)):
            data = lines[i].split(',')

            if data[0] != "epsilon":

                if i == 1:
                    epsilon_mpc = float(data[0])
                    cost_mpc = float(data[1])
                    cost_var_mpc = float(data[2])

                else:
                    epsilon_mpc = np.append(epsilon_mpc, float(data[0]))
                    cost_mpc = np.append(cost_mpc, float(data[1]))
                    cost_var_mpc = np.append(cost_var_mpc, float(data[2]))



        cost_std_mpc = np.sqrt(cost_var_mpc)

    
    if PLOT_DP:
        
        filename_dp = "/home/naveed/Documents/Optimal_control_ilqr/stochastic_hjb_1dcos_T300000_X200_processNoise_e0_10.csv"
        #filename_dp = "/home/naveed/Documents/Optimal_control_ilqr/dp_lqp_e0_2.csv"
        file_dp = open(filename_dp,"r")

        lines = file_dp.read().splitlines()

        for i in range(len(lines)):
            data = lines[i].split(',')

            if data[0] != "epsilon":

                if i == 1:
                    epsilon_dp = float(data[0])
                    cost_dp = float(data[1])
                    cost_var_dp = float(data[2])

                else:
                    epsilon_dp = np.append(epsilon_dp, float(data[0]))
                    cost_dp = np.append(cost_dp, float(data[1]))
                    cost_var_dp = np.append(cost_var_dp, float(data[2]))



        cost_std_dp = np.sqrt(cost_var_dp)

    

    pylab.grid(alpha=0.2)

    if PLOT_DP and PLOT_MPC:

        Min_cost = 1 #min(cost_dp[0], cost_mpc[0])

   
    if PLOT_DP:
    
        pylab.plot(epsilon_dp[Noise_level[0]:Noise_level[1]+1],
                    cost_dp[Noise_level[0]:Noise_level[1]+1]/Min_cost,linewidth=3,marker='.',markersize=10,color='r')

    if PLOT_MPC:

        pylab.plot(epsilon_mpc[Noise_level[0]:Noise_level[1]+1],
                    cost_mpc[Noise_level[0]:Noise_level[1]+1]/Min_cost,linestyle='--',linewidth=3,marker='.',markersize=10,color='b')



    if PLOT_DP and PLOT_MPC:
        legend = pylab.legend(["HJB-FD", "MPC-SH"],loc=2)

    if PLOT_DP:
        pylab.fill_between(epsilon_dp[Noise_level[0]:Noise_level[1]+1],
                        (cost_dp[Noise_level[0]:Noise_level[1]+1]-cost_std_dp[Noise_level[0]:Noise_level[1]+1])/Min_cost,
                        (cost_dp[Noise_level[0]:Noise_level[1]+1]+cost_std_dp[Noise_level[0]:Noise_level[1]+1])/Min_cost,
                        alpha=0.25,linewidth=0,color='r')
        
    if PLOT_MPC:
        pylab.fill_between(epsilon_mpc[Noise_level[0]:Noise_level[1]+1],
                        (cost_mpc[Noise_level[0]:Noise_level[1]+1]-cost_std_mpc[Noise_level[0]:Noise_level[1]+1])/Min_cost,
                        (cost_mpc[Noise_level[0]:Noise_level[1]+1]+cost_std_mpc[Noise_level[0]:Noise_level[1]+1])/Min_cost,
                        alpha=0.25,linewidth=0,color='b')

    #pylab.ylim(-1.0,1100)
    #pylab.xlim(-0.01,1.0)

    pylab.xlabel(r'epsilon')
    pylab.ylabel(r'Cost incurred ($J$)')
    #pylab.ylabel(r'$J/\bar{J}$')
    #pylab.title('Cost vs error percentage for 3 agent(s) ')
    #pylab.show()
    pylab.savefig('/home/naveed/Dropbox/Research/Manuscripts/TAC22/plots/'+'cost_1dcos_mpcT200_low_noise.pdf', format='pdf',bbox_inches='tight',pad_inches = 0.02) #1- dp, 2- dp replan, 3 - MPC, 4 - MPC_fast

    if PLOT_MPC:
        file_mpc.close()

    if PLOT_DP:
        file_dp.close()
