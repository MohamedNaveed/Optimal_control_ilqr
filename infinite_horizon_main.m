%% Infinite horizon main
clear;clc;
%load('pendulum_init_guess_T10_U.mat');
%load('u_guess_unicycle_discounted.mat');
SAVE_file = false;
model = model_register('car');
model.name
disp('initial state');
model.X0
disp('discount factor:')
model.beta = 1;
model.beta
model.q = 1; %for fully observed. 
if exist('u_nom', 'var')
    u_guess_from_file = u_nom;
else
    u_guess_from_file = zeros(model.nu,1);
end
%test_cartpole(model, model.Xg);

%% Terminal controller 

if rank(ctrb(model.A, model.B)) == model.nx
    [K,S,e] = dlqr(model.A, model.B, model.Q, model.R); % neglected half in matlab implementation doesn't matter
else
    %when the system is uncontrollable around the origin.
    S = model.Qf; %zeros(model.nx,model.nx); % 
    K = zeros(model.nu,model.nx);
end

total_time = 100;
maxIte = 1000;

%% iterate over every T
T_list = [30];


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
    T = T_list(iT)
    Q_ilqr = model.Q;
    R_ilqr = model.R; 
    
    if T == total_time
        Q_T = zeros(size(S));
    else
        Q_T = S;
    end
    
    
    if T>length(u_guess_from_file)
        u_guess = [u_guess_from_file, zeros(model.nu,T-size(u_guess_from_file,2))];
    else
        u_guess = u_guess_from_file(:,1:T);
    end
    
    
    [x_nom, u_nom, cost] = ILQR(model, model.X0, model.Xg, u_guess, T,...
                                Q_ilqr, R_ilqr, Q_T, maxIte);
    %%
    [cost_timestep, cost_to_go, time_inflection] = calc_cost(x_nom, u_nom, ...
                            model.Xg, T, Q_ilqr, R_ilqr, Q_T,...
                            model.name, model.beta);
    fprintf('Time of inflection: %d\n', time_inflection)

    %% plot cost convergence
    %{
    plot_trajectory(x_nom, u_nom, T, 0, model.name);
    figure;
    semilogy(1:length(cost),cost,'LineWidth',2);
    xlabel('ILQR Iterations')
    ylabel('Cost')
    title('Cost vs iterations')
    %}
    %% Cost to go estimated
    state_err = compute_state_error(x_nom(:,T+1), model.Xg, model.name);
    
    CTG_est = (model.beta^(T+1))*0.5*state_err'*Q_T*state_err;
    fprintf('Terminal Cost with beta: %f \n', CTG_est);

    phi_x = 0.5*state_err'*Q_T*state_err;
    fprintf('Terminal Cost: %f \n', phi_x);
    
    
    %% Terminal controller. 
    
    T_term = 0;%total_time -T;
    
    x_term = zeros(model.nx,T_term+1);
    u_term = zeros(model.nu,T_term);
    x_term(:,1) = x_nom(:,T+1);
    cost_term = 0;
    if T_term > 0
        for t = 1:T_term
    
            state_err = compute_state_error(x_term(:,t), model.Xg, model.name);
    
            u_term(:,t) = -K*state_err;
    
            cur_cost = 0.5*state_err'*model.Q*state_err + 0.5*u_term(:,t)'*model.R*u_term(:,t);
            cost_term = cost_term + cur_cost;
    
            x_term(:,t+1) = model.state_prop(t, x_term(:,t), u_term(:,t), model);
    
            cost_timestep = [cost_timestep, cur_cost];
    
        end
    
        fprintf('True CTG: %f \n', cost_term);
    end
    %% full trajectory.
    
    X = [x_nom, x_term(:,2:end)];
    U = [u_nom, u_term];
    plot_trajectory(X, U, T, T_term, model.name,model.dt,model.Xg);
    

    %% plot cost vs timesteps
    
    figure;
    semilogy(0:length(cost_timestep)-1, cost_timestep, 'LineWidth', 3);
    xlabel('time-steps');
    ylabel('cost incurred');
    title('Cost incurred at every time step.')

    figure;
    semilogy(0:length(cost_to_go)-1, cost_to_go, 'LineWidth', 3);
    xlabel('time-steps');
    yline(0.1, 'LineWidth', 3);
    xline(time_inflection, 'LineWidth', 3)
    ylabel('cost-to-go');
    grid on;
    title('Cost to go at every time step.')
    
    %% Cost
    
    cost_ilqr(iT) = sum(cost_timestep(1:T));
    total_cost(iT) = sum(cost_timestep);
    exp_CTG_vec(iT) = CTG_est;
    true_CTG_vec(iT) = cost_term;
    ilqr_final_state_error(:,iT) = compute_state_error(x_nom(:,T+1), model.Xg, model.name);
    norm_final_state_error(iT) = norm(ilqr_final_state_error(:,iT));

end

%% plot metrics
infinite_horizon = 0; %temporary
SAVE_PLOT = false;
%plot_cost_metrics(T_list, cost_ilqr, total_cost, exp_CTG_vec, true_CTG_vec,...
%    norm_final_state_error, infinite_horizon, SAVE_PLOT);

ilqr_final_state_error
cost_ilqr
total_cost
exp_CTG_vec
true_CTG_vec

if SAVE_file
    save("unicycle_with_terminal_cost.mat");
end