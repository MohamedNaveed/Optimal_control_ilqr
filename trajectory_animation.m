clc;
clear;
load('data/traj_iterations_car_wobs.mat');

% Ellipse parameters
%c_obs_1 = [5; 1; 0; 0];
c_obs_1 = [2.25; 3]; % Ellipse center 1
c_obs_2 = [17.75; 3]; % Ellipse center 2
E_obs_1 = [(1/2.5)^2, 0; 0, 1]; % Parameters of the ellipse
E_obs_2 = E_obs_1; % Same parameters for the second ellipse

% Initialize video writer
videoFile = 'trajectory_animation_wobs.avi';
v = VideoWriter(videoFile, 'Motion JPEG AVI');
v.FrameRate = 10; % Set frame rate
%v.Quality = 95; % Set higher quality
open(v);


fig = figure('Visible', 'off'); % Create figure without displaying
hold on;

% Plot start point
plot(model.X0(1), model.X0(2), 'Marker', '.', 'Color', 'r', ...
    'MarkerSize', 15, 'DisplayName', 'Start');

% Plot goal point
plot(model.Xg(1), model.Xg(2), 'Marker', '.', 'Color', 'g', ...
    'MarkerSize', 15, 'DisplayName', 'Goal');

%Plot ellipses

h1 = plot_ellipse(c_obs_1, E_obs_1, 'k'); % Blue ellipse
set(h1, 'DisplayName', 'Obstacle'); % Set legend entry for the first ellipse
h2 = plot_ellipse(c_obs_2, E_obs_2, 'k'); % Blue ellipse
set(h2, 'HandleVisibility', 'off');


% Customize plot
ylim([-1, 5]); % current y-axis limits
xlim([-1, 20]);
grid on;
legend();
xlabel('$x$ [m]', 'Interpreter', 'latex');
ylabel('$y$ [m]', 'Interpreter', 'latex');
ax = findobj(gcf,'type','axes'); % current axes
set(ax, 'FontSize', 14);

x_nom = x_traj_ite_ilqr(:,:,1);
% Plot trajectory
h_traj = plot(x_nom(1,:), x_nom(2,:), 'Marker', '.', 'MarkerSize', 10, ...
    'LineWidth', 2, 'DisplayName', 'Trajectory', 'Color','b');

% Capture frame and write to video
frame = getframe(fig);
writeVideo(v, frame);
% Pause for a short duration to create the animation effect
pause(0.1);

% Animation loop
for k = 2:size(x_traj_ite_ilqr, 3)
    % Update trajectory plot
    x_nom = x_traj_ite_ilqr(:,:,k);
    set(h_traj, 'XData', x_nom(1, :), 'YData', x_nom(2, :));
    
    % Capture frame and write to video
    frame = getframe(fig);
    writeVideo(v, frame);
    % Pause for a short duration to create the animation effect
    pause(0.1);
end

hold off;

% Close the video file
close(v);