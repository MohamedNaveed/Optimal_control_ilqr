%% Infinite horizon main
clear;clc;
load('pendulum_init_guess_T30.mat');
T = 30;

X0 = [0;0];  %theta (rad), thetadot (rad/s)
Xg = [180*pi/180;0];

model = pendulum_model();

%test_pendulum(Xg);

%% Terminal controller 
U_term = 0;
[A,B] = pendulum_A_B(Xg, U_term);

Q = eye(model.nx);
R = eye(model.nu);

[K,S,e] = dlqr(A,B,Q,R);

%% ILQR model-based (finite horizon controller.)

u_guess = [u_nom, zeros(1,T-length(u_nom))]; 
[x_nom, u_nom, cost] = ILQR(model, X0, Xg, u_guess, T, S);

plot_trajectory(x_nom, u_nom, T);

%% plot cost convergence
figure;
semilogy(1:length(cost),cost,'LineWidth',2);

%%
disp('Cost to go estimated');
state_err = (x_nom(:,T+1) - Xg);
state_err(1) = atan2(sin(state_err(1)),cos(state_err(1)));

CTG_est = 0.5*state_err'*S*state_err;
fprintf('Estimated CTG: %f \n', CTG_est);
%% Terminal controller. 

T_term = 100;

x_term = zeros(model.nx,T_term+1);
u_term = zeros(model.nu,T_term);
x_term(:,1) = x_nom(:,T+1);
cost_term = 0;

for t = 1:100
    
    state_err = (x_term(:,t) - Xg);
    state_err(1) = atan2(sin(state_err(1)),cos(state_err(1)));
    
    u_term(:,t) = -K*state_err;
        
    cost_term = cost_term + 0.5*state_err'*Q*state_err + ... 
                                0.5*u_term(:,t)'*R*u_term(:,t);
                            
    x_term(:,t+1) = pendulum_nl_state_prop(t, x_term(:,t), u_term(:,t));
       
end
fprintf('True CTG: %f \n', cost_term);

%% full trajectory.
X = [x_nom, x_term(:,2:end)];
U = [u_nom, u_term];
plot_trajectory(X, U,T+T_term);
