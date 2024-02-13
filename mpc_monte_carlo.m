% mpc using ilqr

clear;clc;

model = model_register('cartpole');
model.q = 1; %full state
model.name
fprintf('initial state = %d \n', model.X0);
fprintf('final state = %d \n', model.Xg);

fprintf('Horizon = %d \n', model.horizon);
fprintf('dt = %d \n', model.dt);

SAVE_MAT_FILE = false;
WRITE_CSVFILE = false; 
output_filename = 'test.csv';
header = ["epsilon", "Average Cost", "Cost variance", "Time taken"];
writematrix(header,output_filename,'WriteMode','append');

epsi_range = [0.01];

n_mc_runs = 1;

cost_vec = zeros(length(epsi_range),1);
var_vec = zeros(length(epsi_range),1);

num_cores = 1;

for it = 1:length(epsi_range)
    
    epsilon = epsi_range(it)
    cost_vec_mc = zeros(n_mc_runs,1);
    tic;
    %parfor (n_mc = 1:n_mc_runs, num_cores)
    for (n_mc = 1:n_mc_runs)
        n_mc
        cost_vec_mc(n_mc) = mpc_ilqr(model, epsilon);
    end
 
    time_taken = toc;
    cost_vec_mc = rmoutliers(cost_vec_mc);
    cost_vec(it) = mean(cost_vec_mc);
    var_vec(it) = var(cost_vec_mc);

    data = [epsilon, cost_vec(it), var_vec(it), time_taken/n_mc_runs];
    writematrix(data,output_filename,'WriteMode','append');
    
end

if SAVE_MAT_FILE    
    save("mpc_car_e02_04_T200.mat");
end



