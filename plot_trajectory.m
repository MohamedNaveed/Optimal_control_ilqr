function [] = plot_trajectory(x_nom, u_nom, horizon, T_term, modelName)

T = horizon + T_term;

%% plot traj
if strcmp(modelName, 'pendulum')
    figure;
    timesteps = 0:T;
    subplot(3,1,1);
    plot(timesteps, x_nom(1,:),'LineWidth',2);
    ylabel('$\theta$','Interpreter','latex');
    subplot(3,1,2);
    plot(timesteps, x_nom(2,:),'LineWidth',2);
    ylabel('$\dot{\theta}$','Interpreter','latex');
    subplot(3,1,3);
    plot(timesteps(1:T), u_nom, 'LineWidth',2);
    ylabel('u');
    xlabel('Time steps');

elseif strcmp(modelName, 'cartpole')
    figure;
    
    timesteps = 0:T;
    subplot(3,2,1);
    plot(timesteps, x_nom(1,:),'LineWidth',2);
    %xline(horizon, 'LineWidth',2);
    grid on;
    ylabel('$x$','Interpreter','latex');
    subplot(3,2,2);
    plot(timesteps, x_nom(2,:),'LineWidth',2);
    %xline(horizon, 'LineWidth',2);
    grid on;
    ylabel('$\dot{x}$','Interpreter','latex');
    subplot(3,2,3);
    plot(timesteps, x_nom(3,:),'LineWidth',2);
    %xline(horizon, 'LineWidth',2);
    grid on;
    ylabel('$\theta$','Interpreter','latex');
    subplot(3,2,4);
    plot(timesteps, x_nom(4,:),'LineWidth',2);
    %xline(horizon, 'LineWidth',2);
    grid on;
    ylabel('$\dot{\theta}$','Interpreter','latex');
    subplot(3,2,5:6);
    plot(timesteps(1:T), u_nom, 'LineWidth',2);
    %xline(horizon, 'LineWidth',2);
    grid on;
    ylabel('u');
    xlabel('Time steps');


end

