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
    
    CONSIDER_OBSTACLES = false;
    state_err = (x - modelXg);

    if CONSIDER_OBSTACLES

        obstacle_params;
    
    
        g_1 = (c_obs_1 - x)' * E_obs_1 * (c_obs_1 - x) - 1; %g(x)
        g_2 = (c_obs_2 - x)' * E_obs_2 * (c_obs_2 - x) - 1;
        obstacle_cost = M * (exp(-g_1) + exp(-g_2));
        %obstacle_cost = -M * (log(g_1) + log(g_2));
        cost = (0.5 * state_err' * Q * state_err) + obstacle_cost;
    
        if calc_gradients
    
            gx_1 =  2 * (x - c_obs_1)' * E_obs_1; %dg/dx
            gx_2 = 2 * (x - c_obs_2)' * E_obs_2;
            gxx_1 = 2 * E_obs_1; %d^2g/dx^2
            gxx_2 = 2 * E_obs_2;
    
            lx = state_err' * Q - M *(exp(-g_1) * gx_1 + exp(-g_2) * gx_2); % (1 x nx) vector
            %lx = state_err' * Q - M * (gx_1/g_1 + gx_2/g_2);
            
            lxx = Q - M * (exp(-g_1) * (gxx_1 - gx_1'*gx_1) +...
                           exp(-g_2) * (gxx_2 - gx_2'*gx_2)); % (nx x nx) vector
    
            %lxx = Q - M * (gxx_1/g_1 - gx_1'*gx_1/(g_1^2) + ...
            %            gxx_2/g_2 - gx_2'*gx_2/(g_2^2));
        else
            lx = [];
            lxx = [];
        end

    else 
        cost = (0.5 * state_err' * Q * state_err);

        if calc_gradients
    
            lx = state_err' * Q; 
            
            lxx = Q;
        else
            lx = [];
            lxx = [];
        end
    end
end
