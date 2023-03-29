function [] = plot_cost_metrics(T_vec, cost_ilqr,...
                            total_cost, exp_CTG_vec, true_CTG_vec,...
                            norm_final_state_error, SAVE_PLOT)

font_size = 10;


fig = figure;
plot(T_vec, total_cost,'b','Marker','.','MarkerSize',15,'LineWidth',3, 'DisplayName', 'Infinite horizon');
hold on;
plot(T_vec, cost_ilqr,'r-.','LineWidth',2, 'DisplayName', 'Finite horizon');
ylim([200,1000]);
xlabel('$T$','Interpreter','latex','FontSize',font_size);
ylabel('Cost','FontSize',font_size);
h = legend();
set(h,'FontSize',font_size); %'Position',[0.6 0.2410 0.1 0.1]);
set(fig,'Visible', 'off');

if SAVE_PLOT
    set(fig,'Units','inches');
    fig.Position = [80,80,4.5,2.5];
    screenposition = get(fig,'Position');
    set(fig,...
        'PaperPosition',[0 0 screenposition(3:4)],...
        'PaperSize',[screenposition(3:4)]);
    print -dpdf -painters 'C:\Users\mohdn\Dropbox\Research\Manuscripts\CDC23\total_cost_cartpole_X0_0.pdf'
end

fig = figure;
plot(T_vec, true_CTG_vec,'b','Marker','.','MarkerSize',15,'LineWidth',3, 'DisplayName', 'Actual');
hold on;
plot(T_vec, exp_CTG_vec,'r-.','LineWidth',2, 'DisplayName', 'Expected');
ylim([0,500]);
xlabel('$T$','Interpreter','latex','FontSize',font_size);
ylabel('Terminal Cost-to-go','FontSize',font_size);
h = legend();
set(h,'FontSize',font_size); %'Position',[0.6 0.2410 0.1 0.1]);
set(fig,'Visible', 'off');
if SAVE_PLOT
    set(fig,'Units','inches');
    fig.Position = [80,80,4.5,2.5];
    screenposition = get(fig,'Position');
    set(fig,...
        'PaperPosition',[0 0 screenposition(3:4)],...
        'PaperSize',[screenposition(3:4)]);
    print -dpdf -painters 'C:\Users\mohdn\Dropbox\Research\Manuscripts\CDC23\terminal_cost_cartpole_X0_0.pdf'
end

fig =figure;
plot(T_vec, norm_final_state_error,'k','Marker','.','MarkerSize',15,'LineWidth', 2);
xlabel('$T$','Interpreter','latex','FontSize',font_size);
ylabel('L2 norm of state error','Interpreter','latex','FontSize',font_size);
%title('Error in the final state of finite horizon controller');
set(fig,'Visible', 'off');
if SAVE_PLOT
    set(fig,'Units','inches');
    fig.Position = [80,80,4.5,2.5];
    screenposition = get(fig,'Position');
    set(fig,...
        'PaperPosition',[0 0 screenposition(3:4)],...
        'PaperSize',[screenposition(3:4)]);
    print -dpdf -painters 'C:\Users\mohdn\Dropbox\Research\Manuscripts\CDC23\error_cartpole_X0_0.pdf'
end

end

