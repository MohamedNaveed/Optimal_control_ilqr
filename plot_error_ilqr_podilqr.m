clc;clear;

load('cartpole_ilqr_pod_ilqr_comp.mat');

error_control = (u_nom_podilqr - u_nom)./abs(u_nom);
error_control(30) = 0;
error_control = rmoutliers(error_control,2);
error_state = (x_nom_podilqr - x_nom)./abs(x_nom);
error_state = rmoutliers(error_state,2);
T = length(u_nom);
SAVE_PLOT = true;

fig = figure;

    timesteps = 0:T;
    subplot(3,2,1);
    hold on;
    plot(timesteps, error_state(1,:),'LineWidth',2);
    grid on;
    ylabel('error $x$','Interpreter','latex');
    
    subplot(3,2,2);
    hold on;
    plot(timesteps, error_state(2,:),'LineWidth',2);
    grid on;
    ylabel('error $\dot{x}$','Interpreter','latex');
    
    subplot(3,2,3);
    hold on;
    plot(timesteps, error_state(3,:),'LineWidth',2);
    grid on;
    ylabel('error $\theta$','Interpreter','latex');
    
    subplot(3,2,4);
    hold on;
    plot(timesteps, error_state(4,:),'LineWidth',2);
    grid on;
    ylabel('error $\dot{\theta}$','Interpreter','latex');
    
    subplot(3,2,5:6);
    hold on;
    plot(timesteps(1:T), error_control, 'LineWidth',2);
    grid on;
    ylabel('u');
    xlabel('Time steps');
    
    ax = gca; % current axes
    ax.FontSize = font_size;
   
    
    if SAVE_PLOT
        set(fig,'Visible', 'off');
        set(fig,'Units','inches');
        fig.Position = [100,100,5,5];
        screenposition = get(fig,'Position');
        set(fig,...
            'PaperPosition',[0 0 screenposition(3:4)],...
            'PaperSize',[screenposition(3:4)]);
        print -dpdf -painters '/home/naveed/Documents/Optimal_control_ilqr/plots/comp_ilqr_podilqr.pdf'
    end
