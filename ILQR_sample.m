% ILQR sample

clear;clc;

model = model_register('cartpole');
model.name
fprintf('initial state = %d \n', model.X0);
fprintf('final state = %d \n', model.Xg);

fprintf('Horizon = %d \n', model.horizon);
fprintf('dt = %d \n', model.dt);

problem = 'output feed'; % 'full state' | 'output' %type of cost
% open loop ilqr
load('cartpole_ilqr_fullyobserved_T30.mat');
%u_guess = zeros(model.nu, model.horizon);
u_guess = u_nom;
maxIte = 100;


if problem == 'output'
    model.Q = 0.1*eye(model.nz);  % cost only on outputs
    model.Qf = 100*eye(model.nz); % cost only on outputs
    
    Q_X = model.C'*model.Q*model.C; % converting to the equivalent Q matrix for state x
    Qf_X = model.C'*model.Qf*model.C;
    
    [x_nom, u_nom, cost] = ILQR(model, model.X0, model.Xg, u_guess, model.horizon,...
                                Q_X, model.R, Qf_X, maxIte); %trajectory optimization using iLQR.

else
    
    [x_nom, u_nom, cost] = ILQR(model, model.X0, model.Xg, u_guess, model.horizon,...
                            model.Q, model.R, model.Qf, maxIte); %trajectory optimization using iLQR.
end

%% 
plot_trajectory(x_nom, u_nom, model.horizon,0,model.name,model.dt);
