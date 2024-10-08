% ILQR sample

clear;clc;

model = model_register('cartpole');
model.name
fprintf('initial state = %d \n', model.X0);
fprintf('final state = %d \n', model.Xg);

fprintf('Horizon = %d \n', model.horizon);
fprintf('dt = %d \n', model.dt);

problem = 'full state'; % 'full state' | 'output' %type of cost
maxIte = 100;

% open loop ilqr
if strcmp(problem,'output')
    
    load('cartpole_output_T30.mat');
    %load('podilqr_cartpole_output_T30_1.mat');
    %u_guess = u_nom;
    %u_guess = zeros(model.nu, model.horizon);
    %u_guess(:,1) = u_nom(:,1);
    model.Q = 10*eye(model.nz);  % cost only on outputs
    model.Qf = 1000*eye(model.nz); % cost only on outputs
    
    Q_X = model.C'*model.Q*model.C; % converting to the equivalent Q matrix for state x
    Qf_X = model.C'*model.Qf*model.C;
    
    [x_nom, u_nom, cost, x_trajs] = ILQR(model, model.X0, model.Xg, u_guess, model.horizon,...
                                Q_X, model.R, Qf_X, maxIte); %trajectory optimization using iLQR.

else
    model.q = 1;
    %load('sqp_sol.mat');
    if strcmp(model.name,'cartpole')
        load('data/cartpole_init_guess_T30.mat');
        u_guess = u_nom;
    
    elseif strcmp(model.name,'car')
        %load('data/car_initial_guess_straight_wobs.mat')
        %u_guess = u_nom;
        u_guess = zeros(model.nu, model.horizon);
        u_guess(2,:) = -0.04*ones(model.horizon,1);
        u_guess(1,:) = -1*ones(model.horizon,1);
    else
        u_guess = zeros(model.nu, model.horizon);
    end
    
    u_guess = zeros(model.nu, model.horizon);
    x_guess = gen_traj(model.X0, u_guess, model);
    
    plot_trajectory(x_guess, u_guess, model.horizon,0,model.name,model.dt, model.Xg);

    %% call ILQR
    tic;
    [x_nom, u_nom, cost, ~, x_traj_ite_ilqr] = ILQR(model, model.X0, model.Xg, u_guess, model.horizon,...
                            model.Q, model.R, model.Qf, maxIte); %trajectory optimization using iLQR.
    toc;
end

%% plotting
plot_trajectory(x_nom, u_nom, model.horizon,0,model.name,model.dt, model.Xg);
