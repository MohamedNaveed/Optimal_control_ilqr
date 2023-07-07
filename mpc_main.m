% mpc using ilqr

clear;clc;

model = model_register('1dcos');
model.name
fprintf('initial state = %d \n', model.X0);
fprintf('final state = %d \n', model.Xg);

fprintf('Horizon = %d \n', model.horizon);
fprintf('dt = %d \n', model.dt);

%% MPC using ilqr solver

%u_guess = zeros(model.nu, model.horizon);
load('u_guess_1dcos_T200.mat');

maxIte = 200;
X0 = model.X0;
cost_mpd = 0;
for t = 0:model.horizon-1

    T = model.horizon - t;

    u_guess = u_nom;
    
    [x_nom, u_nom, cost] = ILQR(model, X0, model.Xg, u_guess, T,...
                            model.Q, model.R, model.Qf, maxIte);

    state_err = compute_state_error(x_new(:,i), xg, model.name);

            cost_new = cost_new + (0.5*state_err'*Q*state_err + ... 
                                    0.5*u_new(:,i)'*R*u_new(:,i));
    cost_mpc = 

    X_0 = 
end                        
%% 
plot_trajectory(x_nom, u_nom,model.horizon,0,model.name,model.dt);