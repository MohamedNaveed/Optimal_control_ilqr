function [x_out] = RK4(t, x, u, model)
%Runge Kutta implementation.

    k1 = model.nl_ode(t, x, u, model);
    k2 = model.nl_ode(t + model.dt/2, x + k1*model.dt/2, u, model);
    k3 = model.nl_ode(t + model.dt/2, x + k2*model.dt/2, u, model);
    k4 = model.nl_ode(t + model.dt, x + k3*model.dt, u, model);

    x_out = x + model.dt*(k1 + 2*k2 + 2*k3 + k4)/6;


end