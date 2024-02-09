% mpc using ilqr

clear;clc;

model = model_register('1dcos');
model.name
fprintf('initial state = %d \n', model.X0);
fprintf('final state = %d \n', model.Xg);

fprintf('Horizon = %d \n', model.horizon);
fprintf('dt = %d \n', model.dt);

%{
% open loop ilqr
load('u_guess_1dcos_T150.mat');
u_guess = u_nom;
maxIte = 200;

[x_nom, u_nom, cost] = ILQR(model, model.X0, model.Xg, u_guess, model.horizon,...
                            model.Q, model.R, model.Qf, maxIte); %trajectory optimization using iLQR.
%}

%% MPC using ilqr solver

epsilon = 8;

maxIte = 100;
n_samples = 5;
x_data = zeros(n_samples, model.horizon+1);
for ns = 1:n_samples
    x_mpc = zeros(model.nx, model.horizon+1);
    u_mpc = zeros(model.nu, model.horizon);
    x_mpc(:,1) = model.X0;
    load('data/u_guess_1dcos_T200.mat');
    %u_guess = zeros(model.nu, model.horizon);
    %u_nom = u_guess
    cost_mpc = 0;
    
    tic;
    for t = 0:model.horizon-1
        
        fprintf('t= %d \n',t);
        T = model.horizon - t;
        
        u_guess = u_nom(:,end-T+1:end);
        
        [x_nom, u_nom, cost] = ILQR(model, x_mpc(:,t+1), model.Xg, u_guess, T,...
                                model.Q, model.R, model.Qf, maxIte); %trajectory optimization using iLQR.
        
        u_mpc(:,t+1) = u_nom(:,1);
        
        state_err = compute_state_error(x_nom(:,1), model.Xg, model.name);
    
        cost_mpc = cost_mpc + (0.5*state_err'*model.Q*state_err + ... 
                                        0.5*u_mpc(:,t+1)'*model.R*u_mpc(:,t+1)); %incremental cost
                                    
        x_mpc(:,t+2) = model.state_prop(t+1, x_mpc(:,t+1), u_mpc(:,t+1), model, epsilon); % state propagate.
     
    end
    x_data(ns,:) = x_mpc;

    time_taken = toc;
    state_err = compute_state_error(x_mpc(:,model.horizon+1), model.Xg, model.name);
    cost_mpc = cost_mpc + (0.5*state_err'*model.Qf*state_err); %terminal cost
    fprintf('Time taken %d \n', time_taken);
end


%% 
%plot_trajectory(x_mpc, u_mpc, model.horizon,0,model.name,model.dt);
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