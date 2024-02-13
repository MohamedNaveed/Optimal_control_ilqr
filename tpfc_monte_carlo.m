% TPFC with ILQR.
% mpc using ilqr

clear;clc;

model = model_register('cartpole');
model.q = 1; %full state
model.name
fprintf('initial state = %d \n', model.X0);
fprintf('final state = %d \n', model.Xg);

fprintf('Horizon = %d \n', model.horizon);
fprintf('dt = %d \n', model.dt);

WRITE_CSVFILE = false; 
output_filename = 'test.csv';
header = ["epsilon", "Average Cost", "Cost variance", "Time taken"];
writematrix(header,output_filename,'WriteMode','append');

epsi_range = [0.01];

n_mc_runs = 1;

cost_vec = zeros(length(epsi_range),1);
var_vec = zeros(length(epsi_range),1);

if strcmp(model.name,'cartpole')
        load('data/cartpole_init_guess_T30_dt_01.mat');
elseif strcmp(model.name,'car')
    load('data/car_u_guess_T30.mat')
end
maxIte = 500;
u_guess = u_nom;

[x_nom, u_nom, cost, K] = ILQR(model, model.X0, model.Xg, u_guess, model.horizon,...
                            model.Q, model.R, model.Qf, maxIte); %trajectory optimization using iLQR.

for it = 1:length(epsi_range)
    
    epsilon = epsi_range(it)
    cost_vec_mc = zeros(n_mc_runs,1);
    tic;
    %parfor (n_mc = 1:n_mc_runs, num_cores)
    for (n_mc = 1:n_mc_runs)
        n_mc
        cost_vec_mc(n_mc) = tpfc_ilqr(model, epsilon, x_nom, u_nom, K);
    end
 
    time_taken = toc;
    cost_vec_mc = rmoutliers(cost_vec_mc);
    cost_vec(it) = mean(cost_vec_mc);
    var_vec(it) = var(cost_vec_mc);

    data = [epsilon, cost_vec(it), var_vec(it), time_taken/n_mc_runs];
    writematrix(data,output_filename,'WriteMode','append');
    
end





