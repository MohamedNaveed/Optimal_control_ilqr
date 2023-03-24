function [model] = model_register(modelName)

%% pendulum
if strcmp(modelName, 'pendulum')
    model.m = 0.5;
    model.L = 0.5;
    model.u_max = 1;
    model.dt = 0.1;
    model.nx = 2;
    model.nu = 1;
    model.g = 9.81;
    model.X0 = [0;0];  %theta (rad), thetadot (rad/s)
    model.Xg = [180*pi/180;0];
    model.Q = eye(model.nx);
    model.R = 2*10^0 * eye(model.nu);
    U_term = 0;
    [A,B] = pendulum_A_B(model.Xg, U_term);
    model.A = A;
    model.B = B;
    
end

