% POD-ILQR 

clear;clc;

model = model_register('cartpole');
model.name
fprintf('initial state = %d \n', model.X0);
fprintf('final state = %d \n', model.Xg);

fprintf('Horizon = %d \n', model.horizon);
fprintf('dt = %d \n', model.dt);

u_guess = zeros(model.nu, model.horizon);
maxIte = 200;


model.Q = 0.1*eye(model.nz); 
model.Qf = 100*eye(model.nz);
Q_Z = model.CC'*model.Q*model.CC;% converting to the equivalent Q matrix for state Z
Qf_Z = model.CC'*model.Qf*model.CC;

[Z_nom, u_nom, cost] = POD_ILQR(model, model.X0, model.Xg, u_guess, model.horizon,...
                            Q_Z, model.R, Qf_Z, maxIte);
