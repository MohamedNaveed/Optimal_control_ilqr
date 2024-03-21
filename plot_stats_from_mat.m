% get statistics from mat file.
clear;
clc;
load("data/car_with_terminal_cost_x0_1.mat"); %load("pendulum_exp_29_T_1_30_X0_75.mat");
SAVE_PLOT = true;
%load("cartpole_exp_27_par_1_30.mat");%"cartpole_exp_27_1_30_X0_45_newini.mat");
%infinite_horizon = load('pendulum_exp_30_T150_X0_0.mat');%load("pendulum_exp_29_T150_X0_75.mat");
%infinite_horizon = load("cartpole_exp_28_T150_X0_0.mat");%"cartpole_exp_28_T150_X0_45.mat");
%infinite_horizon.total_cost = 0;

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
n_idx = length(T_list) -1;
plot_cost_metrics(T_list(1:n_idx), cost_ilqr(1:n_idx)+exp_CTG_vec(1:n_idx), total_cost, exp_CTG_vec(1:n_idx), ...
    true_CTG_vec, norm_final_state_error(1:n_idx), infinite_horizon, SAVE_PLOT); %cost vs T


