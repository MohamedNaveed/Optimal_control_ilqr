function [cost_timestep, cost_to_go, time_step] = calc_cost(x_nom, u_nom, xg, horizon, Q, R, QT, modelName, beta)
% forward_pass

cost = 0;
cost_timestep = zeros(1,horizon);

for i=1:horizon
    
    state_err = compute_state_error(x_nom(:,i), xg, modelName);
    
    cur_cost = 0.5*state_err'*Q*state_err + 0.5*u_nom(:,i)'*R*u_nom(:,i);
    cost = cost + (beta^i)*cur_cost;
    
    cost_timestep(i) = (beta^i)*cur_cost;
   
end
%terminal cost
state_err = compute_state_error(x_nom(:,horizon+1), xg, modelName);
cur_cost = (beta^(horizon+1))*0.5*state_err'*QT*state_err;
cost = cost + cur_cost;
cost_timestep(horizon+1) = cur_cost;

flag = 1;
for i=1:horizon+1
    
    cost_to_go(i) = sum(cost_timestep(i:end));
    if ((cost_to_go(i) <= 0.1) && (flag == 1))
        time_step = i;
        flag = 0;
    end
end

end