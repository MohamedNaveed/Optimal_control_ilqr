function [state_dot] = unicycle_nl_ode(t, state, U, model)
   
    f = zeros(model.nx, 1);
    g = zeros(model.nx, 1);

    x = state(1);
    y = state(2);
    theta = state(3);

    
    f(1) = 0;
    f(2) = 0;
    f(3) = 0;
    
    g(1) = cos(theta)*U(1);
    g(2) = sin(theta)*U(1);
    g(3) = U(2);

    state_dot = f + g;
    
end