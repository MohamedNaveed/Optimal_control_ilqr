% mpc using ilqr

clear;clc;

model = model_register('cartpole');
model.name
model.q = 1; %full state
fprintf('initial state = %f \n', model.X0);
fprintf('final state = %f \n', model.Xg);

fprintf('Horizon = %f \n', model.horizon);
fprintf('dt = %f \n', model.dt);

%{
% open loop ilqr
%load('u_guess_1dcos_T150.mat');
%u_guess = u_nom;
u_guess = zeros(model.nu, model.horizon);

maxIte = 400;

[x_nom, u_nom, cost] = ILQR(model, model.X0, model.Xg, u_guess, model.horizon,...
                            model.Q, model.R, model.Qf, maxIte); %trajectory optimization using iLQR.

%}
%% MPC using ilqr solver

epsilon = 0.01;

maxIte = 100;

x_mpc = zeros(model.nx, model.horizon+1);
u_mpc = zeros(model.nu, model.horizon);
x_mpc(:,1) = model.X0;
%load('data/car_u_guess_T30.mat');
load('data/cartpole_init_guess_T30.mat')
%u_guess = zeros(model.nu, model.horizon);
%u_nom = u_guess
cost_mpc = 0;
K = zeros(model.nu,model.nx,model.horizon);
x_nom = zeros(model.nx,model.horizon);
tic;
for t = 0:model.horizon-1
    
    fprintf('t= %d \n',t);
    T = model.horizon - t;
    
    u_guess = u_nom(:,end-T+1:end);
    
    [x_nom, u_nom, cost, K] = ILQR(model, x_mpc(:,t+1), model.Xg, u_guess, T,...
                            model.Q, model.R, model.Qf, maxIte, K, x_nom);%trajectory optimization using iLQR.
    
    u_mpc(:,t+1) = u_nom(:,1);
    
    state_err = compute_state_error(x_nom(:,1), model.Xg, model.name);

    cost_mpc = cost_mpc + (0.5*state_err'*model.Q*state_err + ... 
                                    0.5*u_mpc(:,t+1)'*model.R*u_mpc(:,t+1)); %incremental cost
                                
    x_mpc(:,t+2) = model.state_prop(t+1, x_mpc(:,t+1), u_mpc(:,t+1), model, epsilon); % state propagate.
 
end

time_taken = toc;
state_err = compute_state_error(x_mpc(:,model.horizon+1), model.Xg, model.name);
cost_mpc = cost_mpc + (0.5*state_err'*model.Qf*state_err); %terminal cost
fprintf('Time taken %d \n', time_taken);



%% 
plot_trajectory(x_mpc, u_mpc, model.horizon,0,model.name,model.dt);
%%
%{
load('/home/naveed/Documents/Dynamic_programming/Data/dp_hjb_1dcos_n_200_N_300000_epsi0_1.mat');
figure;
timesteps1 = (0:model.horizon)*model.dt;
timesteps2 = double(0:N)*dt;
idxs = double((0:1500:N)) + 1;
plot(timesteps1, x_mpc, 'b','LineWidth',2,'DisplayName','MPC');
hold on;
plot(timesteps2, x_dp,'--r', 'LineWidth',2,'DisplayName','DP');
ylabel('state');
xlabel('Time steps');
legend();

figure;
plot(timesteps1(1:model.horizon), u_mpc, 'b', 'LineWidth',2,'DisplayName','MPC');
hold on;
plot(timesteps2(1:end-1), u_dp, '--r', 'LineWidth',2,'DisplayName','DP');
ylabel('u');
xlabel('Time steps');
legend();
%}

%% plot ensemble of trajectories
%{
fig = figure;
timesteps = (0:model.horizon)*model.dt;
plot(timesteps, x_data,'LineWidth',2);
ylabel('state');
xlabel('time');
grid on;
ylim([-3,3]);
SAVE_PLOT = true;
if SAVE_PLOT
    font_size = 16;
    ax = gca;
    ax.FontSize = font_size; 
    %set(h,'FontSize',font_size);
    set(fig,'Units','inches');
    screenposition = get(fig,'Position');
    set(fig,...
        'PaperPosition',[0 0 screenposition(3:4)],...
        'PaperSize',[screenposition(3:4)]);
    
    print -dpdf -vector 'mpc_ilqr_traj_epsi8.pdf'
end
%}