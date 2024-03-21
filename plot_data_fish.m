% get statistics from mat file.
clear;
clc;
load("data/swimmer_case1.mat"); %load("pendulum_exp_29_T_1_30_X0_75.mat");
SAVE_PLOT = true;

norm_final_state_error = zeros(length(T_list));
infinite_horizon = 0;
true_CTG_vec = zeros(length(T_list));

%%
n_idx = length(T_list);
plot_cost_metrics(T_list(1:n_idx), total_cost(1:n_idx), 0, terminal_cost(1:n_idx), ...
    true_CTG_vec, norm_final_state_error(1:n_idx), infinite_horizon, SAVE_PLOT); %cost vs T


