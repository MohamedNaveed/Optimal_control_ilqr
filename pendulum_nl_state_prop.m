function [state_n] = pendulum_nl_state_prop(state,U)

model = pendulum_model();

state_n = zeros(2,1);

state_n(1) = state(1) + state(2)*model.dt;
state_n(2) = state(2) - model.g*sin(state(1))*model.dt/model.L +...
                    U*model.dt/(model.m*(model.L^2));

state_n(1) = atan2(sin(state_n(1)), cos(state_n(1)));

end