% get statistics from mat file.
clear;
clc;
%load("cartpole_init_guess_T30.mat");
%plot_trajectory(x_nom, u_nom, T, 0, model.name); %ilqr trajectory
figure;
semilogy(1:length(cost),cost,'LineWidth',2);% cost convergence
plot_trajectory(X, U, T, T_term, model.name); % full trajectory
%% plot cost vs timesteps

figure;
semilogy(0:length(cost_timestep)-1, cost_timestep, 'LineWidth', 3);
xlabel('time-steps');
ylabel('cost incurred');
title('Cost incurred at every time step.')
%%
plot_cost_metrics(T_vec, cost_ilqr, total_cost, exp_CTG_vec, true_CTG_vec); %cost vs T
T_vec
ilqr_final_state_error
cost_ilqr
total_cost
exp_CTG_vec
true_CTG_vec


%% norm of state error 
for i=1:length(T_vec)

    norm_error(i) = norm(ilqr_final_state_error(:,i));

end
%%
figure;
plot(T_vec, norm_error,'b','Marker','o','LineWidth', 3);
xlabel('Finite horizon T');
ylabel('L2 norm of error');
title('Error in the final state of finite horizon controller');