% get statistics from mat file.
clear;
clc;
load("pendulum_exp_28_1_30_X0_75.mat");

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
plot_cost_metrics(T_list, cost_ilqr, total_cost, exp_CTG_vec, true_CTG_vec); %cost vs T



%% norm of state error 
for i=1:length(T_list)

    norm_error(i) = norm(norm_final_state_error(i));

end
%%
figure;
plot(T_list, norm_error,'b','Marker','o','LineWidth', 3);
xlabel('Finite horizon T');
ylabel('L2 norm of error');
title('Error in the final state of finite horizon controller');