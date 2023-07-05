function [] = plot_cost_metrics(T_vec, cost_ilqr,...
                            total_cost, exp_CTG_vec, true_CTG_vec,...
                            norm_final_state_error, infinite_horizon, SAVE_PLOT)

font_size = 14;



fig = figure;
grid on;
hold on;
plot(T_vec, total_cost,'b','Marker','.','MarkerSize',17,'LineWidth',4, 'DisplayName', strcat('Transfer + ', string(newline), 'regulation'));
%plot(T_vec, cost_ilqr,'r-.','Marker','.','MarkerSize',15,'LineWidth',3, 'DisplayName', 'Finite horizon');
plot(T_vec, infinite_horizon.total_cost*ones(size(T_vec)), '--','Color','[0.4660 0.6740 0.1880]','LineWidth',3, 'DisplayName', 'Infinite horizon');

%ylim([200,1000]);
%ylim([-30,500]);
xlabel('Transfer time T','FontSize',font_size);
ylabel('Cost','FontSize',font_size);
h = legend();
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
    print -dpdf -painters '/home/naveed/Dropbox/Research/Manuscripts/CDC23/total_cost_pendulum_X0_0.pdf'
end

fig = figure;
plot(T_vec, true_CTG_vec,'b','Marker','.','MarkerSize',17,'LineWidth',4, 'DisplayName', 'Actual regulation');
hold on;
grid on;
plot(T_vec, exp_CTG_vec,'r-.','LineWidth',3, 'DisplayName', 'Expected regulation');
%ylim([0,500]);
%ylim([0,400]);
xlabel('Transfer time T','FontSize',font_size);
ylabel('Regulation Cost','FontSize',font_size);
h = legend();
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
    print -dpdf -painters '/home/naveed/Dropbox/Research/Manuscripts/CDC23/terminal_cost_pendulum_X0_0.pdf'
end

fig =figure;
grid on;
hold on;
plot(T_vec, norm_final_state_error,'k','Marker','.','MarkerSize',17,'LineWidth', 3);
xlabel('Transfer time T','FontSize',font_size);
ylabel('L2 norm of state error','FontSize',font_size);
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
    print -dpdf -painters '/home/naveed/Dropbox/Research/Manuscripts/CDC23/error_pendulum_X0_0.pdf'
end

end

