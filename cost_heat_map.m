% Example code to plot the heatmap using imagesc function
clc; clear;
% Define model (for demonstration)
model = model_register('car');

n = 250;
x = linspace(-1, 20, n);
y = linspace(-1, 5, n);

heatMap = zeros(n, n);

% Calculate heatmap values
for i = 1:n
    for j = 1:n
        [state_cost, ~, ~] = model.l([x(i); y(j);0;0], model.Xg, model.Q, false);
        heatMap(i, j) = state_cost;
    end
end

% Plot heatmap using imagesc function
figure;
imagesc(x, y, heatMap');
colormap jet; % Choose colormap (optional)
colorbar; % Show color bar
xlabel('X-axis');
ylabel('Y-axis');
title('Cost Heatmap');
% Reverse the y-axis tick labels
set(gca,'YDir','normal'); % Ensure y-axis is normal (lower values at bottom)
