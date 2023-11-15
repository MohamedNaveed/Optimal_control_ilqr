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
    model.X0 = [0*pi/180;0];  %theta (rad), thetadot (rad/s)
    model.Xg = [180*pi/180;0];
    model.Q = eye(model.nx);
    model.R = 2*10^0 * eye(model.nu);
    model.Qf = eye(model.nx);
    model.alpha = 1;
    % model around the equilibrium at the upright
    U_term = 0;
    [A,B] = pendulum_A_B(model, model.Xg, U_term);
    model.A = A;
    model.B = B;
    model.state_prop = @pendulum_nl_state_prop;
    model.cal_A_B = @pendulum_A_B;
    model.horizon = 30;
    
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
    model.X0 = [0;0;180*pi/180;0];% pole bottom is pi
    model.R = eye(model.nu);
    model.Q = 0.1*eye(model.nx);
    model.Qf = 100*eye(model.nx);
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
    model.C = [1 0 0 0; 0 1 0 0];
    model.nz = 2; %number of outputs
    model.q = 2; %value of q required for ARMA model.
    model.nZ = model.q*model.nz + (model.q-1)*model.nu; %information-state dimension
    model.horizon = 30; %time horizon of the finite-horizon OCP
    
elseif strcmp(modelName, '1dcos')
    
    model.name = '1dcos';
    model.u_max = 1;
    model.nx = 1;
    model.nu = 1;
    model.alpha = 1;
    model.Xg = 0;
    model.X0 = 1;
    model.state_prop = @cos1d_state_prop;
    model.cal_A_B = @cos1d_A_B;
    model.horizon = 200;
    model.dt = 1.0/model.horizon;
    model.R = 1*model.dt;
    model.Q = 100*model.dt;
    model.Qf = 500;
    
end

