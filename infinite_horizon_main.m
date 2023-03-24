%% Infinite horizon main
clear;clc;
load('pendulum_init_guess_T10.mat');

model = model_register('pendulum');

%test_pendulum(Xg);

%% Terminal controller 

[K,S,e] = dlqr(model.A, model.B, model.Q, model.R); % neglected half in matlab implementation doesn't matter


inc = 1; 
for T = 20

%% ILQR model-based (finite horizon controller.)
%{
Q_ilqr = model.Q;
R_ilqr = model.R; 
Q_T = S; 

if T>length(u_nom)
    u_guess = [u_nom, zeros(1,T-length(u_nom))];
else
    u_guess = u_nom(1:T);
end

[x_nom, u_nom, cost] = ILQR(model, model.X0, model.Xg, u_guess, T, Q_ilqr, R_ilqr, Q_T);

cost_timestep = calc_cost(x_nom, u_nom, model.Xg, T, Q_ilqr, R_ilqr, Q_T);

plot_trajectory(x_nom, u_nom, T);

%% plot cost convergence
%figure;
%semilogy(1:length(cost),cost,'LineWidth',2);

%%
disp('Cost to go estimated');
state_err = (x_nom(:,T+1) - model.Xg);
state_err(1) = atan2(sin(state_err(1)),cos(state_err(1)));

CTG_est = 0.5*state_err'*S*state_err;
fprintf('Estimated CTG: %f \n', CTG_est);
%}
%% Terminal controller. 

T_term = 100;

x_term = zeros(model.nx,T_term+1);
u_term = zeros(model.nu,T_term);
x_term(:,1) = model.X0;%x_nom(:,T+1);
cost_term = 0;

for t = 1:100
    
    state_err = (x_term(:,t) - model.Xg);
    state_err(1) = atan2(sin(state_err(1)),cos(state_err(1)));
    
    u_term(:,t) = -K*state_err;
    
    cur_cost = 0.5*state_err'*model.Q*state_err + 0.5*u_term(:,t)'*model.R*u_term(:,t);
    cost_term = cost_term + cur_cost;
                            
    x_term(:,t+1) = pendulum_nl_state_prop(t, x_term(:,t), u_term(:,t));

    %cost_timestep = [cost_timestep, cur_cost];
       
end
fprintf('True CTG: %f \n', cost_term);

%% full trajectory.
X = [x_term(:,1:end)];
U = [u_term];
plot_trajectory(X, U,T_term);

%% plot cost vs timesteps
%{
figure;
semilogy(0:length(cost_timestep)-1, cost_timestep, 'LineWidth', 3);
xlabel('time-steps');
ylabel('cost incurred');
title('Cost incurred at every time step.')
%}
%% Cost

cost_ilqr(inc) = sum(cost_timestep(1:T+1));
total_cost(inc) = sum(cost_timestep);
T_vec(inc) = T;
exp_CTG_vec(inc) = CTG_est;
true_CTG_vec(inc) = cost_term;

inc = inc + 1;
end

%% plot metrics

plot_cost_metrics(T_vec, cost_ilqr, total_cost, exp_CTG_vec, true_CTG_vec);