%% Infinite horizon main
clear;clc;
load('cartpole_init_guess_T30_X0_45_U.mat');
SAVE_file = false;
model = model_register('cartpole');
u_guess_from_file = U;%u_nom;
%test_cartpole(model, model.Xg);

%% Terminal controller 

[K,S,e] = dlqr(model.A, model.B, model.Q, model.R); % neglected half in matlab implementation doesn't matter
total_time = 150;
maxIte = 10;

%% iterate over every T
T_list = 30;

cost_ilqr = zeros(1,length(T_list));
total_cost = zeros(1,length(T_list));
exp_CTG_vec = zeros(1,length(T_list));
true_CTG_vec = zeros(1,length(T_list));
ilqr_final_state_error = zeros(model.nx,length(T_list));
norm_final_state_error = zeros(1,length(T_list));

M = length(T_list); % number of nodes. 

for iT = 1:length(T_list)
%parfor (iT = 1:length(T_list), M)

% ILQR model-based (finite horizon controller.)

T = T_list(iT);
Q_ilqr = model.Q;
R_ilqr = model.R; 
Q_T = S; 

if T>length(u_guess_from_file)
    u_guess = [u_guess_from_file, zeros(model.nu,T-length(u_guess_from_file))];
else
    u_guess = u_guess_from_file(model.nu,1:T);
end


[x_nom, u_nom, cost] = ILQR(model, model.X0, model.Xg, u_guess, T,...
                            Q_ilqr, R_ilqr, Q_T, maxIte);

cost_timestep = calc_cost(x_nom, u_nom, model.Xg, T, Q_ilqr, R_ilqr, Q_T, model.name);

%% plot cost convergence
plot_trajectory(x_nom, u_nom, T, 0, model.name);
figure;
semilogy(1:length(cost),cost,'LineWidth',2);

%% Cost to go estimated
state_err = compute_state_error(x_nom(:,T+1), model.Xg, model.name);

CTG_est = 0.5*state_err'*S*state_err;
fprintf('Estimated CTG: %f \n', CTG_est);

%% Terminal controller. 

T_term = total_time -T;

x_term = zeros(model.nx,T_term+1);
u_term = zeros(model.nu,T_term);
x_term(:,1) = x_nom(:,T+1);
cost_term = 0;

for t = 1:T_term

    state_err = compute_state_error(x_term(:,t), model.Xg, model.name);
    
    u_term(:,t) = -K*state_err;
    
    cur_cost = 0.5*state_err'*model.Q*state_err + 0.5*u_term(:,t)'*model.R*u_term(:,t);
    cost_term = cost_term + cur_cost;
                            
    x_term(:,t+1) = model.state_prop(t, x_term(:,t), u_term(:,t), model);

    cost_timestep = [cost_timestep, cur_cost];
       
end
fprintf('True CTG: %f \n', cost_term);

%% full trajectory.
X = [x_nom, x_term(:,2:end)];
U = [u_nom, u_term];
plot_trajectory(X, U, T, T_term, model.name);

%% plot cost vs timesteps

figure;
semilogy(0:length(cost_timestep)-1, cost_timestep, 'LineWidth', 3);
xlabel('time-steps');
ylabel('cost incurred');
title('Cost incurred at every time step.')

%% Cost

cost_ilqr(iT) = sum(cost_timestep(1:T));
total_cost(iT) = sum(cost_timestep);
exp_CTG_vec(iT) = CTG_est;
true_CTG_vec(iT) = cost_term;
ilqr_final_state_error(:,iT) = compute_state_error(x_nom(:,T+1), model.Xg, model.name);
norm_final_state_error(iT) = norm(ilqr_final_state_error(:,iT));

end

%% plot metrics

plot_cost_metrics(T_list, cost_ilqr, total_cost, exp_CTG_vec, true_CTG_vec);

ilqr_final_state_error
cost_ilqr
total_cost
exp_CTG_vec
true_CTG_vec

if SAVE_file
    save("cartpole_exp_27_T30_X0_45.mat");
end