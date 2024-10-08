clc;
clear;
load('data/traj_iterations_car_wobs_straight_down.mat');

% Ellipse parameters
obstacle_params;

fig = figure('Visible', 'off'); % Create figure without displaying
hold on;

% Plot start point
plot(model.X0(1), model.X0(2), 'Marker', '.', 'Color', 'r', ...
    'MarkerSize', 15, 'DisplayName', 'Start');

% Plot goal point
plot(model.Xg(1), model.Xg(2), 'Marker', '.', 'Color', 'g', ...
    'MarkerSize', 15, 'DisplayName', 'Goal');

% Plot ellipses

h1 = plot_ellipse(c_obs_1, E_obs_1(1:2,1:2), 'k'); % First ellipse
set(h1, 'DisplayName', 'Obstacle'); % Set legend entry for the first ellipse
h2 = plot_ellipse(c_obs_2, E_obs_2(1:2,1:2), 'k'); % Second ellipse
set(h2, 'HandleVisibility', 'off'); % Hide legend entry for the second ellipse

% Customize plot
ylim([-3, 3]); % current y-axis limits
xlim([-1, 20]);
grid on;
legend();
xlabel('$x$ [m]', 'Interpreter', 'latex');
ylabel('$y$ [m]', 'Interpreter', 'latex');
ax = findobj(gcf, 'type', 'axes'); % current axes
set(ax, 'FontSize', 14);

x_nom = x_traj_ite_ilqr(:,:,1);
% Plot trajectory
h_traj = plot(x_nom(1,:), x_nom(2,:), 'Marker', '.', 'MarkerSize', 10, ...
    'LineWidth', 2, 'DisplayName', 'Trajectory', 'Color', 'b');

% GIF file name
filename = 'trajectory_animation_wobs_straight_down.gif';

% Loop to create animation frames and save as GIF
for k = 1:size(x_traj_ite_ilqr, 3)
    % Update trajectory plot
    x_nom = x_traj_ite_ilqr(:, :, k);
    set(h_traj, 'XData', x_nom(1, :), 'YData', x_nom(2, :));
    
    % Capture frame
    frame = getframe(fig);
    im = frame2im(frame);
    [imind, cm] = rgb2ind(im, 256);
    
    % Write to GIF file
    if k == 1
        imwrite(imind, cm, filename, 'gif', 'Loopcount', inf, 'DelayTime', 0.1);
    else
        imwrite(imind, cm, filename, 'gif', 'WriteMode', 'append', 'DelayTime', 0.1);
    end
    
    % Pause for a short duration to create the animation effect
    %pause(0.1);
end

hold off;
