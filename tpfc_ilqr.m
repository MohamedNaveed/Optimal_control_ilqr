function [cost_tpfc] = tpfc_ilqr(model, epsilon, x_nom, u_nom, K)
%{
if model.horizon == 150
    load('data/u_guess_1dcos_T150.mat');
elseif model.horizon == 200
    load('data/u_guess_1dcos_T200.mat');
end
%}

x_tpfc = zeros(model.nx, model.horizon+1);
x_tpfc(:,1) = model.X0;

u_tpfc = zeros(model.nu, model.horizon);

cost_tpfc = 0;

for t = 1:model.horizon
    
    state_err = compute_state_error(x_tpfc(:,t), x_nom(:,t), model.name);
    u_tpfc(:,t) = u_nom(:,t) - K(:,:,t)*state_err ;
    
    state_err = compute_state_error(x_tpfc(:,t), model.Xg, model.name);

    cost_tpfc = cost_tpfc + (0.5*state_err'*model.Q*state_err + ... 
                                    0.5*u_tpfc(:,t)'*model.R*u_tpfc(:,t)); %incremental cost
                                
    x_tpfc(:,t+1) = model.state_prop(t, x_tpfc(:,t), u_tpfc(:,t), model, epsilon); % state propagate.
 
end

state_err = compute_state_error(x_tpfc(:,model.horizon+1), model.Xg, model.name);
cost_tpfc = cost_tpfc + (0.5*state_err'*model.Qf*state_err); %terminal cost


end

