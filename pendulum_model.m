function [model] = pendulum_model()

model.m = 0.5;
model.L = 0.5;
model.u_max = 1;
model.dt = 0.1;
model.nx = 2;
model.nu = 1;
model.g = 9.81;

end