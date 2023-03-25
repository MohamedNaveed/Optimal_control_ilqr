function [cost_timestep] = calc_cost(x_nom, u_nom, xg, horizon, Q, R, QT)
% forward_pass

cost = 0;
cost_timestep = zeros(1,horizon);

for i=1:horizon
    
    state_err = x_nom(:,i) - xg;
    state_err(1) = atan2(sin(state_err(1)),cos(state_err(1)));
    
    cur_cost = 0.5*state_err'*Q*state_err + 0.5*u_nom(:,i)'*R*u_nom(:,i);
    cost = cost + cur_cost;
    
    cost_timestep(i) = cur_cost;
   
end

%terminal cost
%{
state_err = (x_nom(:,horizon+1) - xg);
state_err(1) = atan2(sin(state_err(1)),cos(state_err(1)));
cur_cost = 0.5*state_err'*QT*state_err;
cost = cost + cur_cost;
cost_timestep(horizon+1) = cur_cost;
%}
end