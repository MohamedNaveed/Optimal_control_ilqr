function h = plot_ellipse(center, E, color)
    theta = linspace(0, 2*pi, 100);
    ellipse_x = cos(theta);
    ellipse_y = sin(theta);
    ellipse_points = [ellipse_x; ellipse_y];
    [V, D] = eig(pinv(E)); % Compute the eigenvalues and eigenvectors
    transform = V * sqrt(D); % Transformation matrix

    % Apply the transformation and translate
    ellipse_transformed = transform * ellipse_points;
    ellipse_transformed(1, :) = ellipse_transformed(1, :) + center(1);
    ellipse_transformed(2, :) = ellipse_transformed(2, :) + center(2);

    % Plot the ellipse
    h = plot(ellipse_transformed(1, :), ellipse_transformed(2, :), 'Color', color, 'LineWidth', 2);
end