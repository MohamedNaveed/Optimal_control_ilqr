from __future__ import division

import numpy as np
import matplotlib.patches as fill
#import pylab
import math
import time
from random import uniform
import matplotlib.pyplot as pylab
import matplotlib.colors as mcolors

params = {'axes.labelsize':16,
            'font.size':12,
            'legend.fontsize':14,
            'xtick.labelsize':14,
            'ytick.labelsize':14,
            'text.usetex':True,
            'figure.figsize':[4.5,3]}
pylab.rcParams.update(params)


PLOT_MPC = True #True #False #
PLOT_TLQR = True
PLOT_TLQR_REPLAN = False
PLOT_PFC = False
PLOT_MPC_FAST2 = False

Noise_level = [0,100] #file index 1.6 - 64 1.0 - 54 .4-42

epsilonList = [0.01, 0.02, 0.03, 0.05, 0.06, 0.07, 0.08, 0.09, 0.1, 0.15, 0.2]

if __name__=='__main__':

    #filenames
    if PLOT_MPC:
        #filename_mpc = "/home/naveed/Dropbox/Research/Data/Decoupling_RAL20/MPC_files/MPC_3_agent_wo_obs.csv"
        #filename_mpc = "/home/naveed/Dropbox/Research/Data/Decoupling_RAL20/MPC_files/MPC_1_agent_hprc_wo_obs.csv"
        #filename_mpc = "/home/naveed/Dropbox/Research/Data/WAFR20/car_w_trailers/MPC_1_wo_obs_modified1.csv"
        #filename_mpc = "/home/naveed/Dropbox/Research/Data/WAFR20/Quad/MPC_1_wo_obs_modified1.csv"
        #filename_mpc = "/home/naveed/Dropbox/Research/Data/WAFR20/car_w_trailers/MPC_1_wo_obs_modified1.csv"
        #filename_mpc = "/home/naveed/Documents/Continuous_time_Optimal_control_oe4/Data/car/MPC_car.csv"
        #filename_mpc = "/home/naveed/Documents/Continuous_time_Optimal_control_oe4/Data/cartpole/MPC_cartpole_d.csv"
        filename_mpc = "/home/naveed/Documents/Optimal_control_ilqr/mpc_cartpole_e001_02_T30.csv"
        #filename_mpc = "/home/naveed/Dropbox/Research/Data/ACC22/mpc_1d_polynomial_T50_processNoise_Mar23.csv"
        #filename_mpc = "/home/naveed/Documents/Dynamic_programming/Data/stochastic_lqr_lqp_T300000_X200_processNoise_e0_20.csv"

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
                    if float(data[0]) in epsilonList:
                        epsilon_mpc = np.append(epsilon_mpc, float(data[0]))
                        cost_mpc = np.append(cost_mpc, float(data[1]))
                        cost_var_mpc = np.append(cost_var_mpc, float(data[2]))



        cost_std_mpc = np.sqrt(cost_var_mpc)

    if PLOT_PFC:
        filename_pfc = "/home/naveed/Dropbox/Research/Data/CDC21/tpfc_1d_polynomial_T50_processNoise_Mar24.csv" #change
        file_pfc = open(filename_pfc,"r")

        lines = file_pfc.read().splitlines()

        for i in range(len(lines)):
            data = lines[i].split(',')

            if data[0] != "epsilon":

                if i == 1:
                    epsilon_pfc = float(data[0])
                    cost_pfc = float(data[1])
                    cost_var_pfc = float(data[2])

                else:
                    epsilon_pfc = np.append(epsilon_pfc, float(data[0]))
                    cost_pfc = np.append(cost_pfc, float(data[1]))
                    cost_var_pfc = np.append(cost_var_pfc, float(data[2]))



        cost_std_pfc = np.sqrt(cost_var_pfc)

    if PLOT_MPC_FAST2:
        #filename_mpc_fast2 = "/home/naveed/Dropbox/Research/Data/Decoupling_RAL20/MPC_files/MPC_fast_3_agent_hprc_wo_obs_c7_p7.csv"
        #filename_mpc_fast2 = "/home/naveed/Dropbox/Research/Data/Decoupling_RAL20/MPC_files/MPC_fast_1_agent_hprc_wo_obs_c7_p7.csv"
        #filename_mpc_fast2 = "/home/naveed/Dropbox/Research/Data/CDC21/stochastic_dp_theoretical.csv"
        #filename_mpc_fast2 = "/home/naveed/Dropbox/Research/Data/WAFR20/car_w_trailers/MPC_SH_1_wo_obs_modified1.csv"
        #filename_mpc_fast2 = "/home/naveed/Dropbox/Research/Data/WAFR20/Quad/MPC_SH_1_wo_obs_modified1.csv"
        #filename_mpc_fast2 = "/home/naveed/Documents/Continuous_time_Optimal_control_oe4/Data/car/MPC_SH_car.csv"
        filename_mpc_fast2 = "/home/naveed/Documents/Continuous_time_Optimal_control_oe4/Data/cartpole/MPCSH_cartpole_d1.csv"
        file_mpc_fast2 = open(filename_mpc_fast2,"r")

        lines = file_mpc_fast2.read().splitlines()

        for i in range(len(lines)):
            data = lines[i].split(',')

            if data[0] != "epsilon":

                if i == 1:
                    epsilon_mpc_fast2 = float(data[0])
                    cost_mpc_fast2 = float(data[1])
                    cost_var_mpc_fast2 = float(data[2])

                else:
                    if float(data[0]) in epsilonList:
                        epsilon_mpc_fast2 = np.append(epsilon_mpc_fast2, float(data[0]))
                        cost_mpc_fast2 = np.append(cost_mpc_fast2, float(data[1]))
                        cost_var_mpc_fast2 = np.append(cost_var_mpc_fast2, float(data[2]))



        cost_std_mpc_fast2 = np.sqrt(cost_var_mpc_fast2)

    if PLOT_TLQR:
        #filename_tlqr = "/home/naveed/Dropbox/Research/Data/Decoupling_RAL20/T_LQR_files/TLQR_data_3_agent_wo_obs.csv"
        #filename_tlqr = "/home/naveed/Dropbox/Research/Data/Decoupling_RAL20/T_LQR_files/TLQR_data_1_agent_hprc_wo_obs.csv"
        #filename_tlqr = "/home/naveed/Dropbox/Research/Data/ACC22/stochastic_dp_1d_T50_X200_processNoise_eAll.csv"
        #filename_tlqr = "/home/naveed/Dropbox/Research/Data/WAFR20/car_w_trailers/TLQR_1_wo_obs_modified1.csv"
        #filename_tlqr = "/home/naveed/Dropbox/Research/Data/WAFR20/Quad/TLQR_1_wo_obs_modified1.csv"
        #filename_tlqr = "/home/naveed/Documents/Continuous_time_Optimal_control_oe4/Data/car/TPFC_car.csv"
        #filename_tlqr = "/home/naveed/Documents/Continuous_time_Optimal_control_oe4/Data/cartpole/TPFC_cartpole_d.csv"
        filename_tlqr = "/home/naveed/Documents/Optimal_control_ilqr/tpfc_cartpole_e001_02_T30.csv"
        #filename_tlqr = "/home/naveed/Documents/Dynamic_programming/Data/stochastic_dp_lqp_T300000_X200_processNoise_e0_20.csv"

        file_tlqr = open(filename_tlqr,"r")

        lines = file_tlqr.read().splitlines()

        for i in range(len(lines)):
            data = lines[i].split(',')

            if data[0] != "epsilon":

                if i == 1:
                    epsilon_tlqr = float(data[0])
                    cost_tlqr = float(data[1])
                    cost_var_tlqr = float(data[2])

                else:
                    if float(data[0]) in epsilonList:
                        epsilon_tlqr = np.append(epsilon_tlqr, float(data[0]))
                        cost_tlqr = np.append(cost_tlqr, float(data[1]))
                        cost_var_tlqr = np.append(cost_var_tlqr, float(data[2]))



        cost_std_tlqr = np.sqrt(cost_var_tlqr)

    if PLOT_TLQR_REPLAN:
        #filename_tlqr_r = "/home/naveed/Dropbox/Research/Data/Decoupling_RAL20/T_LQR_files/TLQR_replan_1_agent_hprc_wo_obs_02.csv"
        #filename_tlqr_r = "/home/naveed/Dropbox/Research/Data/Decoupling_RAL20/T_LQR_files/TLQR_replan_3_agent_hprc_wo_obs_02.csv"
        #filename_tlqr_r = "/home/naveed/Dropbox/Research/Data/CDC21/stochastic_dp_1dCos_T60000_X50_processNoise_e0.csv"
        #filename_tlqr_r = "/home/naveed/Dropbox/Research/Data/WAFR20/car_w_trailers/TLQR2_1_wo_obs_modified1_hprc.csv"
        #filename_tlqr_r = "/home/naveed/Dropbox/Research/Data/WAFR20/Quad/TLQR2_1_wo_obs_modified1.csv"
        #filename_tlqr_r = "/home/naveed/Documents/Continuous_time_Optimal_control_oe4/Data/car/TPFC2_car.csv"
        #filename_tlqr_r = "/home/naveed/Documents/Continuous_time_Optimal_control_oe4/Data/cartpole/TPFC2_cartpole_d1.csv"
        filename_tlqr_r = "/home/naveed/Documents/Optimal_control_ilqr/tpfc_car_e001_02_T30.csv"

        file_tlqr_r = open(filename_tlqr_r,"r")

        lines = file_tlqr_r.read().splitlines()

        for i in range(len(lines)): #len(lines)
            data = lines[i].split(',')

            if data[0] != "epsilon":

                if i == 1:
                    epsilon_tlqr_r = float(data[0])
                    cost_tlqr_r = float(data[1])
                    cost_var_tlqr_r = float(data[2])

                else:
                    if float(data[0]) in epsilonList:
                        epsilon_tlqr_r = np.append(epsilon_tlqr_r, float(data[0]))
                        cost_tlqr_r = np.append(cost_tlqr_r, float(data[1]))
                        cost_var_tlqr_r = np.append(cost_var_tlqr_r, float(data[2]))



        cost_std_tlqr_r = np.sqrt(cost_var_tlqr_r)



    pylab.grid(alpha=0.2)

    #min cost
    if PLOT_TLQR and PLOT_TLQR_REPLAN and PLOT_MPC and PLOT_PFC and PLOT_MPC_FAST2 :

        Min_cost = min(cost_tlqr[0], cost_mpc[0], cost_tlqr_r[0],cost_pfc[0], cost_mpc_fast2[0] )

    elif PLOT_TLQR_REPLAN and PLOT_MPC and PLOT_PFC and PLOT_MPC_FAST2 :

        Min_cost = min(cost_mpc[0], cost_tlqr_r[0],cost_pfc[0], cost_mpc_fast2[0] )

    elif PLOT_TLQR and PLOT_TLQR_REPLAN and PLOT_MPC and PLOT_MPC_FAST2 :

        Min_cost = min(cost_tlqr[0], cost_mpc[0], cost_tlqr_r[0], cost_mpc_fast2[0] )

    elif PLOT_TLQR_REPLAN and PLOT_MPC and PLOT_MPC_FAST2 :

        Min_cost = min(cost_mpc[0], cost_tlqr_r[0], cost_mpc_fast2[0] )

    elif PLOT_TLQR and PLOT_MPC and PLOT_PFC :

        Min_cost = min(cost_tlqr[0], cost_mpc[0],cost_pfc[0])

    elif PLOT_MPC and PLOT_MPC_FAST2  :

        Min_cost = min(cost_mpc[0], cost_mpc_fast2[0] )

    elif PLOT_TLQR and PLOT_TLQR_REPLAN and PLOT_MPC:

        Min_cost = min(cost_tlqr[0], cost_mpc[0], cost_tlqr_r[0])

    elif PLOT_TLQR and PLOT_TLQR_REPLAN:
        Min_cost = min(cost_tlqr[0], cost_tlqr_r[0])

    elif PLOT_TLQR and PLOT_MPC:
        Min_cost = min(cost_tlqr[0], cost_mpc[0])

    elif PLOT_TLQR and PLOT_PFC:
        Min_cost = min(cost_tlqr[0], cost_pfc[0])

    elif PLOT_MPC:
        Min_cost = cost_mpc[0]

    elif PLOT_TLQR:
        Min_cost = cost_tlqr[0]

    Min_cost = 1 #normalization
    epsilon_scale_factor = 3

    #plotting
    if PLOT_TLQR:
        pylab.fill_between(epsilon_scale_factor*epsilon_tlqr[Noise_level[0]:Noise_level[1]+1],
                        (cost_tlqr[Noise_level[0]:Noise_level[1]+1]-cost_std_tlqr[Noise_level[0]:Noise_level[1]+1])/Min_cost,
                        (cost_tlqr[Noise_level[0]:Noise_level[1]+1]+cost_std_tlqr[Noise_level[0]:Noise_level[1]+1])/Min_cost,
                        alpha=0.25,linewidth=0,color='r')

        pylab.plot(epsilon_scale_factor*epsilon_tlqr[Noise_level[0]:Noise_level[1]+1],
                    cost_tlqr[Noise_level[0]:Noise_level[1]+1]/Min_cost,
                    linewidth=4,marker='',markersize=10,color='tab:orange', label=r"T-PFC")

    if PLOT_PFC:
        # pylab.fill_between(epsilon_pfc[Noise_level[0]:Noise_level[1]+1],
        #                 (cost_pfc[Noise_level[0]:Noise_level[1]+1] - cost_std_pfc[Noise_level[0]:Noise_level[1]+1])/Min_cost,
        #                 (cost_pfc[Noise_level[0]:Noise_level[1]+1] + cost_std_pfc[Noise_level[0]:Noise_level[1]+1])/Min_cost,
        #                 alpha=0.25,linewidth=0,color='k')

        pylab.plot(epsilon_pfc[Noise_level[0]:Noise_level[1]+1], cost_pfc[Noise_level[0]:Noise_level[1]+1]/Min_cost,
                   linewidth=3,linestyle='--',marker='.',markersize=10, color='#34DD03')

    if PLOT_MPC:
        pylab.fill_between(epsilon_scale_factor*epsilon_mpc[Noise_level[0]:Noise_level[1]+1],
                        (cost_mpc[Noise_level[0]:Noise_level[1]+1]-cost_std_mpc[Noise_level[0]:Noise_level[1]+1])/Min_cost,
                        (cost_mpc[Noise_level[0]:Noise_level[1]+1]+cost_std_mpc[Noise_level[0]:Noise_level[1]+1])/Min_cost,
                        alpha=0.35,linewidth=0,color='b')

        pylab.plot(epsilon_scale_factor*epsilon_mpc[Noise_level[0]:Noise_level[1]+1],
                    cost_mpc[Noise_level[0]:Noise_level[1]+1]/Min_cost,
                    linewidth=3,linestyle='dotted',marker='.',markersize=10,color='b',label=r"MPC-SH")

    if PLOT_MPC_FAST2:
        pylab.fill_between(epsilon_mpc_fast2[Noise_level[0]:Noise_level[1]+1],
                         (cost_mpc_fast2[Noise_level[0]:Noise_level[1]+1] - cost_std_mpc_fast2[Noise_level[0]:Noise_level[1]+1])/Min_cost,
                         (cost_mpc_fast2[Noise_level[0]:Noise_level[1]+1] + cost_std_mpc_fast2[Noise_level[0]:Noise_level[1]+1])/Min_cost,
                         alpha=0.25,linewidth=0,color='m')

        pylab.plot(epsilon_mpc_fast2[Noise_level[0]:Noise_level[1]+1], cost_mpc_fast2[Noise_level[0]:Noise_level[1]+1]/Min_cost,
                   linewidth=3,linestyle='dotted',marker='.',markersize=10,color='m')

    if PLOT_TLQR_REPLAN:
        pylab.fill_between(epsilon_tlqr_r[Noise_level[0]:Noise_level[1]+1],
                        (cost_tlqr_r[Noise_level[0]:Noise_level[1]+1] - cost_std_tlqr_r[Noise_level[0]:Noise_level[1]+1])/Min_cost,
                        (cost_tlqr_r[Noise_level[0]:Noise_level[1]+1] + cost_std_tlqr_r[Noise_level[0]:Noise_level[1]+1])/Min_cost,
                        alpha=0.25,linewidth=0,color='#34DD03')

        pylab.plot(epsilon_tlqr_r[Noise_level[0]:Noise_level[1]+1],cost_tlqr_r[Noise_level[0]:Noise_level[1]+1]/Min_cost,
                   linewidth=3,linestyle='--',marker='.',markersize=10,color='#34DD03')

    ##legends
    if PLOT_TLQR and PLOT_MPC and PLOT_TLQR_REPLAN and PLOT_PFC and PLOT_MPC_FAST2:
        #legend = pylab.legend(["T-LQR", "MPC", "MPC-SH", "T-PFC","T-LQR2"]) #change
        legend = pylab.legend([r"DP",r"DP $\epsilon = 0.0$", r"DP $\epsilon = 0.5$",r"DP $\epsilon = 1.0$",   r"MPC"]) #change

    elif PLOT_TLQR and PLOT_MPC and PLOT_PFC and PLOT_MPC_FAST2:
        #legend = pylab.legend(["T-LQR", "MPC", "MPC-SH", "T-PFC","T-LQR2"]) #change
        legend = pylab.legend([r"Stochastic - DP",r"Stochastic - Theoretical",r"Linear Feedback $O(\epsilon^4)$",r"Deterministic Nonlinear - MPC $O(\epsilon^4)$"]) #change

    elif PLOT_TLQR_REPLAN and PLOT_MPC and PLOT_PFC and PLOT_MPC_FAST2 :
        legend = pylab.legend(["T-PFC", "MPC", "MPC-FH", "T-PFC2"],loc=2)

    elif PLOT_TLQR and PLOT_MPC and PLOT_TLQR_REPLAN and PLOT_MPC_FAST2:
        legend = pylab.legend(["T-PFC", "MPC", "MPC-FH", "T-PFC2"])
        #pass

    elif PLOT_MPC and PLOT_TLQR_REPLAN and PLOT_MPC_FAST2:
        legend = pylab.legend(["Std MPC cost", "SH MPC cost", "T-LQR Replan cost"],loc=2)

    elif PLOT_TLQR and PLOT_MPC and PLOT_PFC:
        legend = pylab.legend(["T-LQR","T-PFC", "MPC" ],loc=2) #change

    elif PLOT_TLQR and PLOT_MPC and PLOT_TLQR_REPLAN:
        legend = pylab.legend([r"DP", r"DP $\epsilon = 0.0$", "MPC"],loc=2)

    elif PLOT_TLQR and PLOT_MPC:
        #legend = pylab.legend(["DP", "MPC"],loc=2)
        #legend = pylab.legend([r"Stochastic - DP",r"Deterministic - Nonlinear MPC"],loc='upper left') #change
        legend = pylab.legend(loc='upper left') 
        pass
    elif PLOT_TLQR and PLOT_TLQR_REPLAN:
        legend = pylab.legend(["T-LQR cost", "T-LQR Replan cost"],loc=2)

    elif PLOT_TLQR and PLOT_PFC:
        legend = pylab.legend(["T-LQR", "T-PFC"],loc=2)

    elif PLOT_MPC and PLOT_MPC_FAST2:
        legend = pylab.legend(["Std MPC cost", "SH MPC"],loc=2)

    elif PLOT_TLQR:
        legend = pylab.legend(["DP"],loc=2)

    elif PLOT_MPC:
        legend = pylab.legend(["MPC cost"],loc=2)

    elif PLOT_TLQR_REPLAN:
        legend = pylab.legend(["MT-LQR Replan cost"],loc=2)

    #pylab.ylim(50,175)
    #pylab.ylim(600,1150)
    pylab.ylim(1500,4000)
    #pylab.xlim(-0.01,1.0)
    pylab.xlim(-0.01,0.61)
    pylab.grid(alpha=0.2)
    #pylab.xlim(-.05,1.6)

    #regimes
    #pylab.plot([0.2,0.2],[0,16],'k-',linewidth=2)
    #pylab.plot([1.25,1.25],[0,16],'k-',linewidth=2)

    #plt.text(0.05,1.5,'Low noise',rotation=90,rotation_mode='anchor',fontsize=18)
    #plt.text(0.25,1.5,'Medium noise',rotation=90,rotation_mode='anchor',fontsize=18)
    #plt.text(1.35,5,'High noise',rotation=90,rotation_mode='anchor',fontsize=18)
    #frame = legend.get_frame()
    #frame.set_facecolor('0.9')
    #frame.set_edgecolor('0.9')

    pylab.xlabel(r'epsilon')
    #pylab.ylabel(r'$J/\bar{J}$')
    pylab.ylabel(r'Cost incurred ($J$)')
    #pylab.title('Cost vs error percentage for 3 agent(s) ')

    pylab.savefig('/home/naveed/Documents/Optimal_control_ilqr/plots/'+'cartpole_ilqr.pdf', format='pdf',bbox_inches='tight',pad_inches = 0.02) #1- TLQR, 2- TLQR replan, 3 - MPC, 4 - MPC_fast
    #pylab.savefig('/home/naveed/Dropbox/Research/Data/AISTATS/'+'cost_car_LQR_PFC_comp.pdf', format='pdf',bbox_inches='tight',pad_inches = 0.02)
    if PLOT_MPC:
        file_mpc.close()

    if PLOT_PFC:
        file_pfc.close()

    if PLOT_MPC_FAST2:
        file_mpc_fast2.close()

    if PLOT_TLQR:
        file_tlqr.close()

    if PLOT_TLQR_REPLAN:
        file_tlqr_r.close()
