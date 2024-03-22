function [] = plot_trajectory(x_nom, u_nom, horizon, T_term, modelName, modeldt, Xg)

if ~exist('modeldt','var')
    modeldt = 1;
end

T = horizon + T_term;
font_size = 14;
SAVE_PLOT = false;

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
    
    
    fig = figure;

    timesteps = 0:T;
    subplot(3,2,1);
    hold on;
    plot(timesteps, x_nom(1,:),'LineWidth',2);
    %xline(horizon, 'LineWidth',2);
    y = ylim; % current y-axis limits
    plot([horizon horizon],[y(1) y(2)],'k','LineWidth',1)
    grid on;
    ylabel('$x$','Interpreter','latex');
    
    subplot(3,2,2);
    hold on;
    plot(timesteps, x_nom(2,:),'LineWidth',2);
    y = ylim; % current y-axis limits
    plot([horizon horizon],[y(1) y(2)],'k','LineWidth',1)
    %xline(horizon, 'LineWidth',2);
    grid on;
    ylabel('$\dot{x}$','Interpreter','latex');
    
    subplot(3,2,3);
    hold on;
    plot(timesteps, x_nom(3,:),'LineWidth',2);
    y = ylim; % current y-axis limits
    plot([horizon horizon],[y(1) y(2)],'k','LineWidth',1)
    %xline(horizon, 'LineWidth',2);
    grid on;
    ylabel('$\theta$','Interpreter','latex');
    
    subplot(3,2,4);
    hold on;
    plot(timesteps, x_nom(4,:),'LineWidth',2);
    y = ylim; % current y-axis limits
    plot([horizon horizon],[y(1) y(2)],'k','LineWidth',1)
    %xline(horizon, 'LineWidth',2);
    grid on;
    ylabel('$\dot{\theta}$','Interpreter','latex');
    
    subplot(3,2,5:6);
    hold on;
    plot(timesteps(1:T), u_nom, 'LineWidth',2);
    y = ylim; % current y-axis limits
    plot([horizon horizon],[y(1) y(2)],'k','LineWidth',1)
    %xline(horizon, 'LineWidth',2);
    grid on;
    ylabel('u');
    xlabel('Time steps');
    
    ax = findobj(gcf,'type','axes'); % current axes
    set(ax, 'FontSize', font_size);
   
    
    if SAVE_PLOT
        set(fig,'Visible', 'off');
        set(fig,'Units','inches');
        fig.Position = [100,100,5,5];
        screenposition = get(fig,'Position');
        set(fig,...
            'PaperPosition',[0 0 screenposition(3:4)],...
            'PaperSize',[screenposition(3:4)]);
        print -dpdf -painters '/home/naveed/Dropbox/Research/Manuscripts/CDC23/cartpole_response_T_4.pdf'
    end

elseif strcmp(modelName, 'car')
    
    
    fig = figure;

    timesteps = 0:T;
    subplot(3,2,1);
    hold on;
    plot(timesteps, x_nom(1,:),'LineWidth',2);
    yline(Xg(1),'LineWidth',2)
    %xline(horizon, 'LineWidth',2);
    y = ylim; % current y-axis limits
    %plot([horizon horizon],[y(1) y(2)],'k','LineWidth',1)
    grid on;
    ylabel('$x$','Interpreter','latex');
    
    subplot(3,2,2);
    hold on;
    plot(timesteps, x_nom(2,:),'LineWidth',2);
    yline(Xg(2),'LineWidth',2)
    y = ylim; % current y-axis limits
    %plot([horizon horizon],[y(1) y(2)],'k','LineWidth',1)
    %xline(horizon, 'LineWidth',2);
    grid on;
    ylabel('$y$','Interpreter','latex');
    
    subplot(3,2,3);
    hold on;
    plot(timesteps, x_nom(3,:),'LineWidth',2);
    yline(Xg(3),'LineWidth',2)
    y = ylim; % current y-axis limits
    %plot([horizon horizon],[y(1) y(2)],'k','LineWidth',1)
    %xline(horizon, 'LineWidth',2);
    grid on;
    ylabel('$\theta$','Interpreter','latex');
    
    subplot(3,2,4);
    hold on;
    plot(timesteps, x_nom(4,:),'LineWidth',2);
    yline(Xg(4),'LineWidth',2)
    y = ylim; % current y-axis limits
    %plot([horizon horizon],[y(1) y(2)],'k','LineWidth',1)
    %xline(horizon, 'LineWidth',2);
    grid on;
    ylabel('$\phi$','Interpreter','latex');
    
    subplot(3,2,5);
    hold on;
    plot(timesteps(1:T), u_nom(1,:), 'LineWidth',2);
    y = ylim; % current y-axis limits
    plot([horizon horizon],[y(1) y(2)],'k','LineWidth',1)
    %xline(horizon, 'LineWidth',2);
    grid on;
    ylabel('$v$','Interpreter','latex');
    xlabel('Time steps');

    subplot(3,2,6);
    hold on;
    plot(timesteps(1:T), u_nom(2,:), 'LineWidth',2);
    y = ylim; % current y-axis limits
    plot([horizon horizon],[y(1) y(2)],'k','LineWidth',1)
    %xline(horizon, 'LineWidth',2);
    grid on;
    ylabel('$\omega$','Interpreter','latex');
    xlabel('Time steps');
    
    ax = findobj(gcf,'type','axes'); % current axes
    set(ax, 'FontSize', font_size);
   
    
    if SAVE_PLOT
        set(fig,'Visible', 'off');
        set(fig,'Units','inches');
        fig.Position = [100,100,5,5];
        screenposition = get(fig,'Position');
        set(fig,...
            'PaperPosition',[0 0 screenposition(3:4)],...
            'PaperSize',[screenposition(3:4)]);
        print -dpdf -painters '/home/naveed/Dropbox/Research/Manuscripts/CDC23/car_response_T_4.pdf'
    end

    figure;

    plot(x_nom(1,:), x_nom(2,:),'LineWidth',2);
    %xline(horizon, 'LineWidth',2);
    ylim([0,4]); % current y-axis limits
    xlim([0,1]);
    grid on;
    xlabel('$x$','Interpreter','latex');
    ylabel('$y$','Interpreter','latex');

elseif strcmp(modelName, 'unicycle')
    
    
    fig = figure;

    timesteps = 0:T;
    subplot(3,2,1);
    hold on;
    plot(timesteps, x_nom(1,:),'LineWidth',2);
    yline(Xg(1),'LineWidth',2)
    %xline(horizon, 'LineWidth',2);
    y = ylim; % current y-axis limits
    plot([horizon horizon],[y(1) y(2)],'k','LineWidth',1)
    grid on;
    ylabel('$x$','Interpreter','latex');
    
    subplot(3,2,2);
    hold on;
    plot(timesteps, x_nom(2,:),'LineWidth',2);
    yline(Xg(2),'LineWidth',2)
    y = ylim; % current y-axis limits
    plot([horizon horizon],[y(1) y(2)],'k','LineWidth',1)
    %xline(horizon, 'LineWidth',2);
    grid on;
    ylabel('$y$','Interpreter','latex');
    
    subplot(3,2,3);
    hold on;
    plot(timesteps, x_nom(3,:),'LineWidth',2);
    yline(Xg(3),'LineWidth',2)
    y = ylim; % current y-axis limits
    plot([horizon horizon],[y(1) y(2)],'k','LineWidth',1)
    %xline(horizon, 'LineWidth',2);
    grid on;
    ylabel('$\theta$','Interpreter','latex');

    
    subplot(3,2,5);
    hold on;
    plot(timesteps(1:T), u_nom(1,:), 'LineWidth',2);
    y = ylim; % current y-axis limits
    plot([horizon horizon],[y(1) y(2)],'k','LineWidth',1)
    %xline(horizon, 'LineWidth',2);
    grid on;
    ylabel('$v$','Interpreter','latex');
    xlabel('Time steps');

    subplot(3,2,6);
    hold on;
    plot(timesteps(1:T), u_nom(2,:), 'LineWidth',2);
    y = ylim; % current y-axis limits
    plot([horizon horizon],[y(1) y(2)],'k','LineWidth',1)
    %xline(horizon, 'LineWidth',2);
    grid on;
    ylabel('$\omega$','Interpreter','latex');
    xlabel('Time steps');
    
    ax = findobj(gcf,'type','axes'); % current axes
    set(ax, 'FontSize', font_size);
    
    if SAVE_PLOT
        set(fig,'Visible', 'off');
        set(fig,'Units','inches');
        fig.Position = [100,100,5,5];
        screenposition = get(fig,'Position');
        set(fig,...
            'PaperPosition',[0 0 screenposition(3:4)],...
            'PaperSize',[screenposition(3:4)]);
        print -dpdf -painters '/home/naveed/Dropbox/Research/Manuscripts/CDC23/unicycle_response_T_4.pdf'
    end

    figure;

    plot(x_nom(1,:), x_nom(2,:),'LineWidth',2);
    %xline(horizon, 'LineWidth',2);
    %ylim([3.5,4.5]); % current y-axis limits
    %xlim([0.8,1.2]);
    grid on;
    xlabel('$x$','Interpreter','latex');
    ylabel('$y$','Interpreter','latex');

elseif strcmp(modelName,'1dcos')
    fig = figure;
    timesteps = (0:T)*modeldt;
    plot(timesteps, x_nom,'LineWidth',2);
    ylabel('state');
    xlabel('time');
    grid on;
    
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

    figure;
    plot(timesteps(1:T), u_nom, 'LineWidth',2);
    ylabel('u');
    xlabel('Time steps');
end

