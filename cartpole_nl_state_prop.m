function [state_n] = cartpole_nl_state_prop(t, state, U, model, varargin)

if (~isempty(varargin)) && (strcmp(varargin{1}, 'ode45'))
    t_span = [(t-1)*model.dt, t*model.dt];
    
    [temp, X_out] = ode45(@(t,y) cartpole_nl_ode(t,y,U, model), t_span, state);
    %X_out(end,3) = atan2(sin(X_out(end,3)), cos(X_out(end,3)));
    state_n = X_out(end,:);

else

    [temp, X_out] = forward_euler(t, state, U, model);
    state_n = X_out;

end

end