function [x_dot] = poly_2d_nl_ode(t, x, u, model)

m1 = 1;
m2 = 0.8;
m3 = 0.75;

x_dot = [x(2);
         -(m1*x(1) + m2*(x(1)^2) + m3*(x(1)^3)) + u];



end