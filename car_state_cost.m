function [cost, lx, lxx] = car_state_cost(x, modelXg, Q, calc_gradients)
    % Function to compute cost, gradients, and Hessians for car state
    % Inputs:
    % - x: current state
    % - modelXg: goal state
    % - Q: weight matrix
    % - calc_gradients: boolean flag to calculate gradients and Hessians
    % Outputs:
    % - cost: computed cost
    % - lx: gradient of the cost (if calc_gradients is true)
    % - lxx: Hessian of the cost (if calc_gradients is true)
    
    state_err = (x - modelXg);
    %c_obs_1 = [5; 1; 0; 0];
    c_obs_1 = [2.25; 3; 0; 0]; % Ellipse center
    c_obs_2 = [17.75; 3; 0; 0];
    E_obs_1 = [(1/2.5)^2, 0, 0, 0; 
               0, 1, 0, 0;
               0, 0, 0, 0;
               0, 0, 0, 0]; % Parameters of the ellipse
    E_obs_2 = E_obs_1; 

    M = 1000; % Obstacle penalty factor

    obs_1 = (c_obs_1 - x)' * E_obs_1 * (c_obs_1 - x);
    obs_2 = (c_obs_2 - x)' * E_obs_2 * (c_obs_2 - x);
    obstacle_cost = M * (exp(-obs_1) + exp(-obs_2));

    cost = (0.5 * state_err' * Q * state_err) + obstacle_cost;

    if calc_gradients
        lx = state_err' * Q - 2 * M * (exp(-obs_1) * (x - c_obs_1)' * E_obs_1 ...
                + exp(-obs_2) * (x - c_obs_2)' * E_obs_2); % (1 x nx) vector

        lxx = Q - 2 * M * (exp(-obs_1) * (E_obs_1 - ...
                2 * (E_obs_1 * (x - c_obs_1)) * (E_obs_1 * (x - c_obs_1))') + ...
                exp(-obs_2) * (E_obs_2 - ...
                2 * (E_obs_2 * (x - c_obs_2)) * (E_obs_2 * (x - c_obs_2))')); % (nx x nx) vector
    else
        lx = [];
        lxx = [];
    end
end
