% mpc using ilqr

clear;clc;

model = model_register('1dcos');
model.name
fprintf('initial state = %d \n', model.X0);
fprintf('final state = %d \n', model.Xg);

fprintf('Horizon = %d \n', model.horizon);
fprintf('dt = %d \n', model.dt);

%% ilqr

%u_guess = zeros(model.nu, model.horizon);
load('u_guess_1dcos_T200.mat');
u_guess = u_nom;
maxIte = 200;

T = model.horizon;

[x_nom, u_nom, cost] = ILQR(model, model.X0, model.Xg, u_guess, T,...
                            model.Q, model.R, model.Qf, maxIte);
                        
