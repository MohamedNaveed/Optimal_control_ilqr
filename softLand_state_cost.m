function [cost, lx, lxx] = softLand_state_cost(x, modelXg, Q, calc_gradients)
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
    z = x(9);

    
    obstacle_cost = exp(-10*z) - 1;
    %obstacle_cost = -M * (log(g_1) + log(g_2));
    cost = (0.5 * state_err' * Q * state_err) + obstacle_cost;

    if calc_gradients

        lx = Q*state_err - [0;0;0;0;0;0;0;0;10*exp(-10*z);0;0;0];
        
        lxx = Q + diag([0,0,0,0,0,0,0,0,1,0,0,0])*100*exp(-10*z);
    else
        lx = [];
        lxx = [];
    end
end