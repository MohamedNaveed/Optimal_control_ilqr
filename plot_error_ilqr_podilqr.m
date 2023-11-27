clc;clear;

load('cartpole_ilqr_pod_ilqr_comp_cost812.mat');

error_control = (u_nom_podilqr - u_nom)./abs(u_nom);
error_control(30) = 0;
error_control = rmoutliers(error_control,2);
error_state = (x_nom_podilqr - x_nom)./abs(x_nom);
error_state = rmoutliers(error_state,2);
T = length(u_nom);
SAVE_PLOT = true;
font_size = 14;
%%

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
yticks_values = [0, 2e-4, 4e-4, 6e-4];
yticks(yticks_values);
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
ylabel('error $u$','Interpreter','latex');
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
    print -dpdf -painters '/home/naveed/Dropbox/Research/Manuscripts/PODILQR/comp_ilqr_podilqr.pdf'
end
