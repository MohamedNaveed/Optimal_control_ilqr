function [cost, lx, lxx] = cartpole_state_cost(x, modelXg, Q, calc_gradients)
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
    cost = (0.5 * state_err' * Q * state_err);
    lx = state_err' * Q;
    lxx = Q;
end
