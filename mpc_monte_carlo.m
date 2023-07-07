% mpc using ilqr

clear;clc;

model = model_register('1dcos');
model.name
fprintf('initial state = %d \n', model.X0);
fprintf('final state = %d \n', model.Xg);

fprintf('Horizon = %d \n', model.horizon);
fprintf('dt = %d \n', model.dt);

SAVE_FILE = true;

epsi_range = [0,1]; %0:0.2:2.0

n_mc_runs = 2; %100

cost_vec = zeros(length(epsi_range),1);
var_vec = zeros(length(epsi_range),1);

num_cores = 2;%40

for it = 1:length(epsi_range)
    
    epsilon = epsi_range(it);
    cost_vec_mc = zeros(n_mc_runs,1);
    
    parfor (n_mc = 1:n_mc_runs, num_cores)
       
        cost_vec_mc(n_mc) = mpc_ilqr(model, epsilon);
    end
    
    cost_vec(it) = mean(cost_vec_mc);
    var_vec(it) = var(cost_vec_mc);
    
end

if SAVE_FILE    
    save("mpc_1dcos_e0_1.mat");
end



