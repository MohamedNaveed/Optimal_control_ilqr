function [AA, BB] = arma_fit(model, delta_u, delta_z)

nu = model.nu; % number of control inputs
nz = model.nz; % number of outputs
n = model.nx; %states in the system
q = model.q; % steps to loop back

t_steps = model.horizon+1; % horizon

no_rollouts = model.nSim; 

U = delta_u(:,1:no_rollouts); 
y_matrix = delta_z(1:nz*t_steps,1:no_rollouts); 

%% build data (V) matrix and calculate the ARMA parameters

alpha_beta = zeros(nz*t_steps, q*(nz + nu));

rank_V = [];
ID_time_idxs = q+1:t_steps;
AA = zeros(q*nz + (q-1)*nu, q*nz + (q-1)*nu,t_steps);
BB = zeros(q*nz + (q-1)*nu,nu,t_steps);

for k = ID_time_idxs
    
    V = build_data_mat_ltv(U, y_matrix, q, nu, nz, k, no_rollouts);
    
    alpha_beta((k-1)*nz + 1: k*nz,:) = y_matrix((k - 1)*nz + 1: (k)*nz, :)*pinv(V);  
    
    rank_V = [rank_V rank(V)];

    alpha = alpha_beta((k-1)*nz + 1: k*nz,1:q*nz);
    beta = alpha_beta((k-1)*nz + 1: k*nz,q*nz + 1:end);

    % POD2C realization
 
    %building AA eq. 22
    
    
    AA(1:nz,:,k-1) = [alpha, beta(:, nu+1:end)]; 
    
    AA(nz + 1: q*nz, 1:(q-1)*nz, k-1) = eye((q-1)*nz);
    
    AA (q*nz + nu + 1:end, q*nz + 1:q*nz + (q-2)*nu,k-1) = eye((q-2)*nu);
    
    %building BB eq. 22
    
    
    
    BB(1:nz,:,k-1) = beta(:,1:nu);
    
    BB(q*nz + 1:q*nz + nu,:,k-1) = eye(nu);
    
end

%fprintf('Estimated ARMA parameters\n\n');


end




