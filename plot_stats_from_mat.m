% get statistics from mat file.
clear;
clc;
SAVE_PLOT = false;
load("pendulum_exp_28_1_30_X0_0.mat"); %load("pendulum_exp_29_T_1_30_X0_75.mat");
%load("cartpole_exp_27_par_1_30.mat");%"cartpole_exp_27_1_30_X0_45_newini.mat");
%infinite_horizon = load("pendulum_exp_29_T150_X0_75.mat");
%infinite_horizon = load("cartpole_exp_28_T150_X0_0.mat");%"cartpole_exp_28_T150_X0_45.mat");
infinite_horizon.total_cost = 0;

%{
plot_trajectory(x_nom, u_nom, T, 0, model.name); %ilqr trajectory
figure;
semilogy(1:length(cost),cost,'LineWidth',2);% cost convergence
plot_trajectory(X, U, T, T_term, model.name); % full trajectory
%% plot cost vs timesteps

figure;
semilogy(0:length(cost_timestep)-1, cost_timestep, 'LineWidth', 3);
xlabel('time-steps');
ylabel('cost incurred');
title('Cost incurred at every time step.')
%}

%%

plot_cost_metrics(T_list, cost_ilqr+exp_CTG_vec, total_cost, exp_CTG_vec, ...
    true_CTG_vec, norm_final_state_error, infinite_horizon, SAVE_PLOT); %cost vs T


