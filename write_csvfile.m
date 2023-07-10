% write cost data file in csv format.
load('mpc_1dcos_e0_2_T200.mat');
output_filename = 'mpc_1dcos_T200.csv';

header = ["epsilon", "Average Cost", "Cost variance", "Time taken"];
data = [epsi_range', cost_vec, var_vec, zeros(length(cost_vec),1)];

writematrix(header, output_filename,'WriteMode','append');
writematrix(data,output_filename,'WriteMode','append');

