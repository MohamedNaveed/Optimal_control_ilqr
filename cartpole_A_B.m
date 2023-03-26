function [A,B] = cartpole_A_B(model, X_bar, U_bar)

x = X_bar(1);
x_dot = X_bar(2);
theta = X_bar(3);
theta_dot = X_bar(4);

u = U_bar;

Ac = subs(model.Ac);
Bc = subs(model.Bc);

A = double(eye(size(Ac,1)) + Ac*model.dt);
B = double(Bc*model.dt);

end

