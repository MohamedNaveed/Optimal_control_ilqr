function [state_dot] = car_nl_ode(t, state, U, model)
   
    f = zeros(model.nx, 1);
    g = zeros(model.nx, 1);

    x = state(1);
    y = state(2);
    theta = state(3);
    v = state(4);
    
    f(1) = 0;
    f(2) = 0;
    f(3) = 0;
    f(4) = 0;
    
    g(1) = cos(theta)*v; %x_dot = v*cos(theta)
    g(2) = sin(theta)*v;%y_dot = v*sin(theta)
    g(3) = tan(U(2))*v/model.L;
    g(4) = U(1);

    state_dot = f + g;
    
end