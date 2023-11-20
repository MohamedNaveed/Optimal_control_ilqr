% POD-ILQR 

clear;clc;

model = model_register('cartpole');
model.name
fprintf('initial state = %d \n', model.X0);
fprintf('final state = %d \n', model.Xg);

fprintf('Horizon = %d \n', model.horizon);
fprintf('dt = %d \n', model.dt);

load('cartpole_output_T30.mat');
%load('podilqr_cartpole_output_T30_1.mat');
u_guess = u_nom;
%u_guess = zeros(model.nu, model.horizon);
%u_guess(:,1) = u_nom(:,1);

maxIte = 200;

%%
model.Q = 10*eye(model.nz); 
model.Qf = 1000*eye(model.nz);

% converting to the equivalent Q matrix for state Z
Q_Z = model.CC'*model.Q*model.CC;
Qf_Z = model.CC'*model.Qf*model.CC;

[x_nom, Z_nom, u_nom,  delta_u, delta_z, cost, At, Bt] = POD_ILQR(model,...
                            model.X0, model.Xg, u_guess, model.horizon,...
                            Q_Z, model.R, Qf_Z, maxIte);

%% 
plot_trajectory(x_nom, u_nom, model.horizon,0,model.name,model.dt);
