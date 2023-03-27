function [model] = model_register(modelName)

%% pendulum
if strcmp(modelName, 'pendulum')
    model.name = 'pendulum';
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
    % model around the equilibrium at the upright
    U_term = 0;
    [A,B] = pendulum_A_B(model, model.Xg, U_term);
    model.A = A;
    model.B = B;
    model.state_prop = @pendulum_nl_state_prop;
    model.cal_A_B = @pendulum_A_B;

elseif  strcmp(modelName, 'cartpole')
    model.name = 'cartpole';
    model.M = 1;
    model.m = 0.01;
    model.L = 0.6;
    model.u_max = 1;
    model.dt = 0.1;
    model.nx = 4;
    model.nu = 1;
    model.g = 9.81;
    model.alpha = 1;
    model.Xg = [0;0;0*pi/180;0]; %x, xdot, theta(rad), thetadot(rad/s)
    model.X0 = [0;0;45*pi/180;0];% pole bottom is pi
    model.R = eye(model.nu);
    model.Q = 0.1*eye(model.nx);
    [Ac, Bc] = cartpole_eqs(model);
    model.Ac = Ac; % continuous time linearised model (symbolic)
    model.Bc = Bc;
    % model around the equilibrium at the upright
    U_term = 0;
    [A, B] = cartpole_A_B(model, model.Xg, U_term);
    model.A = A;
    model.B = B;
    model.state_prop = @cartpole_nl_state_prop;
    model.cal_A_B = @cartpole_A_B;

end

