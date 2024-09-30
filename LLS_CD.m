function [A, B] = LLS_CD(model, X_bar, U_bar)

pert = 0.1; %perturbation std dev.
N = 10; % number of samples
delta_x_new = zeros(model.nx, N);
delta_x = normrnd(0,pert,[model.nx,N]);
delta_u = normrnd(0,pert,[model.nu,N]);

for ns = 1:N
    
    temp1 = model.state_prop(0, X_bar + delta_x(:,ns), U_bar + delta_u(:,ns), model);
    temp2 = model.state_prop(0, X_bar - delta_x(:,ns), U_bar - delta_u(:,ns), model);
    delta_x_new(:,ns) = (temp1 - temp2)./2; %finding the central difference

end
delta_Y = [delta_x; delta_u];
sol = delta_x_new*delta_Y'/(delta_Y*delta_Y');
%sol = delta_x_new*delta_Y'/(((N-1)*pert^2).*eye(model.nx+model.nu));

A = sol(:, 1:model.nx);
B = sol(:, model.nx+1:end);
end