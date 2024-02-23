clear;clc;

model = model_register('cartpole');
model.name
fprintf('initial state = %d \n', model.X0);
fprintf('final state = %d \n', model.Xg);
load('cartpole_full_state_T30_feb1.mat')

T = model.horizon;
X_ode45= zeros(model.nx, T+1);
X_ode45(:,1) = model.X0;

X_euler= zeros(model.nx, T+1);
X_euler(:,1) = model.X0;

for i = 1:T
    X_ode45(:,i+1) = cartpole_nl_state_prop(i, X_ode45(:,i), u_nom(:,i), model, 0,'ode45');

    X_euler(:,i+1) = cartpole_nl_state_prop(i, X_euler(:,i), u_nom(:,i), model, 0);
end

%% plotting
plot_trajectory(X_ode45-X_euler, u_nom, model.horizon,0,model.name,model.dt);
