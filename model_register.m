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
    model.u_max = 5;
    model.dt = 0.1;
    model.nx = 4;
    model.nu = 1;
    model.Cov = diag([0.4321,2.8743,3.6426,1.5945]); %noise covariance. 
    model.g = 9.81;
    model.alpha = 1;
    model.beta = 1;
    model.Xg = [0;0;0*pi/180;0]; %x, xdot, theta(rad), thetadot(rad/s)
    model.X0 = [0;0;180*pi/180;0];% pole bottom is pi
    model.R = 0.05*eye(model.nu);
    model.Q = diag([10,40,1,1.5]);%10*eye(model.nx);
    model.Qf = diag([2700,9000,2700,2700]);%1000*eye(model.nx);
    model.l = @cartpole_state_cost;
    %[Ac, Bc] = cartpole_eqs(model);
    %model.Ac = Ac; % continuous time linearised model (symbolic)
    %model.Bc = Bc;
    model.nl_ode = @cartpole_nl_ode;
    model.state_prop = @cartpole_nl_state_prop;
    model.cal_A_B = @cartpole_A_B;
    % model around the equilibrium at the upright
    U_term = 0;
    [A, B] = cartpole_A_B(model, model.Xg, U_term);
    model.A = A;
    model.B = B;
    model.C = [1 0 0 0; 0 0 1 0];
    
    model.nz = 2; %number of outputs
    model.q = 2; %value of q required for ARMA model.
    model.nZ = model.q*model.nz + (model.q-1)*model.nu; %information-state dimension
    model.CC = [eye(model.nz), zeros(model.nz, model.nZ - model.nz)];%information state system C
    model.horizon = 30; %time horizon of the finite-horizon OCP
    model.nSim = 500;
    model.ptb = 0.0001;
    model.statePtb = 0.001;

elseif  strcmp(modelName, 'car')
    model.name = 'car';
    model.L = 0.58; %length of the car
    model.u_max = 7;
    
    model.dt = 0.1;
    model.nx = 4; %[x,y,theta,velocity]
    model.nu = 2; %[acceleration,steering angle(rad)]
    model.alpha = 0.7;
    model.beta = 1; %discount factor.
    model.Cov = eye(model.nx); %noise covariance. 
    % lane change goal
    model.Xg = [10;3;0;3]; %[x,y,theta (rad),velocity]
    %model.Xg = [15;0;0;3];
    model.X0 = [0; 0; 0; 3];% 
    model.R = [1, 0; 0, 100];%1*eye(model.nu);
    model.Q = [1, 0, 0, 0; 0, 1, 0, 0; 0, 0, 0.01, 0;0, 0, 0, 0.01];
    model.Qf =100*[1, 0, 0, 0; 0, 1, 0, 0;0, 0, 2, 0;0, 0, 0, 1];
    %[Ac, Bc] = cartpole_eqs(model);
    %model.Ac = Ac; % continuous time linearised model (symbolic)
    %model.Bc = Bc;
    model.nl_ode = @car_nl_ode;
    model.state_prop = @nl_state_prop;
    model.cal_A_B = @car_A_B;
    model.l = @car_state_cost;
    % model around the equilibrium at the upright
    U_term = [0;0];
    [A, B] = car_A_B(model, model.Xg, U_term);
    model.A = A;
    model.B = B;
    model.C = [1 0 0 0; 0 0 1 0];
    
    model.nz = 2; %number of outputs
    model.q = 1; %value of q required for ARMA model.
    model.nZ = model.q*model.nz + (model.q-1)*model.nu; %information-state dimension
    model.CC = [eye(model.nz), zeros(model.nz, model.nZ - model.nz)];%information state system C
    model.horizon = 30; %time horizon of the finite-horizon OCP
    model.nSim = 500;
    model.ptb = 0.0001;
    model.statePtb = 0.001;

elseif  strcmp(modelName, 'unicycle')
    model.name = 'unicycle';
    model.u_max = 7;
    model.dt = 0.1;
    model.nx = 3; %[x,y,theta]
    model.nu = 2; %[v,omega]
    model.alpha = 1;
    model.Xg = [1;4;0]; %[x,y,theta (rad)]
    model.X0 = -100*model.Xg;%[-3;-12;0];% 
    model.R = 1*eye(model.nu);
    model.Q = 5*eye(model.nx);
    model.Qf = 100*eye(model.nx);
    model.beta = 1; %discount factor.
    %[Ac, Bc] = cartpole_eqs(model);
    %model.Ac = Ac; % continuous time linearised model (symbolic)
    %model.Bc = Bc;
    model.nl_ode = @unicycle_nl_ode;
    model.state_prop = @nl_state_prop; %same as car for unicyle
    model.cal_A_B = @unicycle_A_B;
    % model around the equilibrium at the upright
    U_term = [0;0];
    [A, B] = unicycle_A_B(model, model.Xg, U_term);
    model.A = A;
    model.B = B;
    model.C = [1 0 0 ; 0 1 0; 0 0 1];
    
    model.nz = size(model.C,1); %number of outputs
    model.q = 1; %value of q required for ARMA model.
    model.nZ = model.q*model.nz + (model.q-1)*model.nu; %information-state dimension
    model.CC = [eye(model.nz), zeros(model.nz, model.nZ - model.nz)];%information state system C
    model.horizon = 30; %time horizon of the finite-horizon OCP
    model.nSim = 500;
    model.ptb = 0.0001;
    model.statePtb = 0.001;



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
    model.q = 1;

elseif  strcmp(modelName, 'poly_2d') %Vadali Sharma system
    model.name = 'poly_2d';
    model.u_max = 7;
    model.dt = 0.001;
    model.nx = 2; %[x,y,theta]
    model.nu = 1; %[v,omega]
    model.alpha = 1;
    model.Xg = [0;0]; %[x,xdot]
    model.X0 = [0.1;0.2];
    model.R = 1*eye(model.nu)*model.dt;
    model.Q = 1*eye(model.nx)*model.dt;
    model.Qf = 100*eye(model.nx);
    model.beta = 1; %discount factor.
    model.nl_ode = @poly_2d_nl_ode;
    model.state_prop = @nl_state_prop; %same as car for unicyle
    model.cal_A_B = @A_B;
    
    model.q = 1; %value of q required for ARMA model.
    model.horizon = 3.0/model.dt; %time horizon of the finite-horizon OCP
    model.nSim = 500;
    model.ptb = 0.0001;
    model.statePtb = 0.001;

elseif  strcmp(modelName, 'poly_1d') %Vadali Sharma system
    model.name = 'poly_2d';
    model.u_max = 7;
    model.dt = 0.001;
    model.nx = 1; %[x,y,theta]
    model.nu = 1; %[v,omega]
    model.alpha = 1;
    model.Xg = [0;0]; %[x,xdot]
    model.X0 = [0.1];
    model.R = 1*eye(model.nu)*model.dt;
    model.Q = 1*eye(model.nx)*model.dt;
    model.Qf = 0*eye(model.nx);
    model.beta = 1; %discount factor.
    model.nl_ode = @poly_2d_nl_ode;
    model.state_prop = @nl_state_prop; %same as car for unicyle
    model.cal_A_B = @A_B;
    
    model.q = 1; %value of q required for ARMA model.
    model.horizon = 3.0/model.dt; %time horizon of the finite-horizon OCP
    model.nSim = 500;
    model.ptb = 0.0001;
    model.statePtb = 0.001;
end

