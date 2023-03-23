function [A,B] = pendulum_A_B(X, U)

model = pendulum_model();

A = [1, model.dt; 0 1] + [0 0;-model.g*cos(X(1))*model.dt/model.L, 0];

B = [0; model.dt/(model.m*(model.L^2))];

end

