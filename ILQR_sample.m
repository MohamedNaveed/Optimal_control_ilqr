% ILQR sample

clear;clc;

model = model_register('cartpole');
model.name
fprintf('initial state = %d \n', model.X0);
fprintf('final state = %d \n', model.Xg);

fprintf('Horizon = %d \n', model.horizon);
fprintf('dt = %d \n', model.dt);


% open loop ilqr
u_guess = zeros(model.nu, model.horizon);
maxIte = 200;

[x_nom, u_nom, cost] = ILQR(model, model.X0, model.Xg, u_guess, model.horizon,...
                            model.Q, model.R, model.Qf, maxIte); %trajectory optimization using iLQR.



%% 
plot_trajectory(x_nom, u_nom, model.horizon,0,model.name,model.dt);
