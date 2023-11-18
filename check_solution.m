clear;clc;

model = model_register('cartpole');
model.name
fprintf('initial state = %d \n', model.X0);
fprintf('final state = %d \n', model.Xg);

fprintf('Horizon = %d \n', model.horizon);
fprintf('dt = %d \n', model.dt);
problem = 'output'; % 'full state' | 'output' %type of cost

% open loop ilqr
if strcmp(problem,'output')

    load('cartpole_output_T30.mat');
    %load('podilqr_cartpole_output_T30.mat');
    model.Q = 10*eye(model.nz);  % cost only on outputs
    model.Qf = 1000*eye(model.nz); % cost only on outputs
    
    Q_X = model.C'*model.Q*model.C; % converting to the equivalent Q matrix for state x
    Qf_X = model.C'*model.Qf*model.C;
    
    [state_nc, control_error, Qu, kt] = check_NC_conditions(model, model.X0, model.Xg, x_nom, u_nom, model.horizon,...
                                Q_X, model.R, Qf_X);

elseif strcmp(problem,'full state')

    load('cartpole_full_state_T30.mat');
    [state_nc, control_error, Qu, kt] = check_NC_conditions(model, model.X0, model.Xg,...
                             x_nom, u_nom, model.horizon,...
                                model.Q, model.R, model.Qf);

else
    %TODO
    load('podilqr_cartpole_output_T30.mat');

    %%
    model.Q = 10*eye(model.nz); 
    model.Qf = 1000*eye(model.nz);
    
    % converting to the equivalent Q matrix for state Z
    Q_Z = model.CC'*model.Q*model.CC;
    Qf_Z = model.CC'*model.Qf*model.CC;

    [state_nc, control_error, Qu, kt] = check_NC_conditions(model, model.X0, model.Xg,...
                                        x_nom, u_nom, model.horizon,...
                                        Q_Z, model.R, Qf_Z);
end

control_error
Qu
kt