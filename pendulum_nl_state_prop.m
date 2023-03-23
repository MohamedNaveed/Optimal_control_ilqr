function [state_n] = pendulum_nl_state_prop(t, state,U)

model = pendulum_model();



%{
state_n(1) = state(1) + state(2)*model.dt;
state_n(2) = state(2) - model.g*sin(state(1))*model.dt/model.L +...
                    U*model.dt/(model.m*(model.L^2));

state_n(1) = atan2(sin(state_n(1)), cos(state_n(1)));
%}


t_span = [(t-1)*model.dt, t*model.dt];

[temp, X_out] = ode45(@(t,y) pendulum_nl_ode(t,y,U), t_span, state);
X_out(end,1) = atan2(sin(X_out(end,1)), cos(X_out(end,1)));

state_n = X_out(end,:);

end