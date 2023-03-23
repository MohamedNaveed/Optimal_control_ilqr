function [state_dot] = pendulum_nl_ode(t, state, U)
    model = pendulum_model();
    
    state_dot = zeros(2,1);
    
    state_dot(1) = state(2);
    state_dot(2) = - model.g*sin(state(1))/model.L + U/(model.m*(model.L^2));
    
    
    
end