function [Ac, Bc] = cartpole_eqs(model)

% calculation of jacobians for carpole 

syms x x_dot theta theta_dot u f g

X = [x; x_dot; theta; theta_dot];


term1 = model.m/(model.M + model.m - model.m*(cos(theta)^2)); %intermediate variable
term2 = term1*(model.g*sin(theta)*cos(theta) - model.L*sin(theta)*(theta_dot^2));

f(1) = x_dot;
f(2) = term2;
f(3) = theta_dot;
f(4) = (model.g*sin(theta)/model.L) + (cos(theta)/model.L)*term2;

g(1) = 0;
g(2) = term1*u/model.m;
g(3) = 0;
g(4) = (cos(theta)/model.L)*term1*u/model.m;

state_dot = f + g;

Ac = jacobian(state_dot, X); %continuous time A, B 
Bc = jacobian(state_dot, u);

end