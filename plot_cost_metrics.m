function [] = plot_cost_metrics(T_vec, cost_ilqr,...
                            total_cost, exp_CTG_vec, true_CTG_vec,...
                            norm_final_state_error, infinite_horizon, SAVE_PLOT)

font_size = 14;
M = 0.05; %30 for car 
idx = find(exp_CTG_vec < M, 1);


fig = figure;
grid on;
hold on;
%plot(T_vec, total_cost,'b','Marker','.','MarkerSize',17,'LineWidth',4, 'DisplayName', strcat('Transfer + ', string(newline), 'regulation'));
plot(T_vec, cost_ilqr,'b','Marker','.','MarkerSize',17,'LineWidth',3, 'DisplayName', 'Finite horizon');
plot(T_vec, infinite_horizon.total_cost*ones(size(T_vec)), '--','Color','[0.4660 0.6740 0.1880]','LineWidth',3, 'DisplayName', 'Infinite horizon');
xline(T_vec(idx),'LineWidth',3, 'DisplayName', 'First hitting time');
%ylim([200,1000]);
%ylim([215,270]);
xticks([0, 30, 60, 100, 200])
xlabel('Transfer time $T$','FontSize',font_size,'Interpreter','latex');
ylabel('Cost','FontSize',font_size,'Interpreter','latex');
h = legend('Location', 'northeast');
ax = gca; % current axes
ax.FontSize = font_size;
set(h,'FontSize',font_size,'Interpreter','latex'); %'Position',[0.6 0.2410 0.1 0.1]);

if SAVE_PLOT
    set(fig,'Visible', 'off');
    set(fig,'Units','inches');
    fig.Position = [80,80,4.5,3];
    screenposition = get(fig,'Position');
    set(fig,...
        'PaperPosition',[0 0 screenposition(3:4)],...
        'PaperSize',[screenposition(3:4)]);
    print -dpdf -vector '/home/naveed/Documents/Optimal_control_ilqr/plots/car/total_cost_case1.pdf'
end


fig = figure;
%semilogy(T_vec, true_CTG_vec,'b','Marker','.','MarkerSize',17,'LineWidth',4, 'DisplayName', 'Actual regulation');
hold on;
grid on;
plot(T_vec, exp_CTG_vec,'r','Marker','.','MarkerSize',17,'LineWidth',3, 'DisplayName', 'Terminal Cost');
%ylim([-50,3500]);
%ylim([-0.2,3]);
yline(M,'--','Color','k','LineWidth',3, 'DisplayName', '$M$')
%xticks([0, 30, 60, 100, 200])
xticks([0, 400, 800, 1000, 1500])
xline(T_vec(idx),'LineWidth',3, 'DisplayName', 'First hitting time');
xlabel('Transfer time $T$','FontSize',font_size,'Interpreter','latex');
%ylabel('Regulation Cost','FontSize',font_size,'Interpreter','latex');
ylabel('Terminal Cost $\phi(x_T)$','FontSize',font_size,'Interpreter','latex');
h = legend('Interpreter','latex');
ax = gca; % current axes
ax.FontSize = font_size;
set(h,'FontSize',font_size); %'Position',[0.6 0.2410 0.1 0.1]);

if SAVE_PLOT
    set(fig,'Visible', 'off');
    set(fig,'Units','inches');
    fig.Position = [80,80,4.5,3];
    screenposition = get(fig,'Position');
    set(fig,...
        'PaperPosition',[0 0 screenposition(3:4)],...
        'PaperSize',[screenposition(3:4)]);
    print -dpdf -vector '/home/naveed/Documents/Optimal_control_ilqr/plots/car/terminal_cost_case1.pdf'
end
%{
fig =figure;
grid on;
hold on;
plot(T_vec, norm_final_state_error,'k','Marker','.','MarkerSize',17,'LineWidth', 3);
xlabel('Transfer time $T$','FontSize',font_size,'Interpreter','latex');
ylabel('L2 norm of state error','FontSize',font_size,'Interpreter','latex');
%title('Error in the final state of finite horizon controller');

ax = gca; % current axes
ax.FontSize = font_size;
if SAVE_PLOT
    set(fig,'Visible', 'off');
    set(fig,'Units','inches');
    fig.Position = [80,80,4.5,3];
    screenposition = get(fig,'Position');
    set(fig,...
        'PaperPosition',[0 0 screenposition(3:4)],...
        'PaperSize',[screenposition(3:4)]);
    print -dpdf -vector '/home/naveed/Documents/Optimal_control_ilqr/plots/car/error_car_wtc.pdf'
end
%}
end

