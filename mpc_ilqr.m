function [cost_mpc] = mpc_ilqr(model, epsilon)
%{
if model.horizon == 150
    load('data/u_guess_1dcos_T150.mat');
elseif model.horizon == 200
    load('data/u_guess_1dcos_T200.mat');
end
%}
if strcmp(model.name,'cartpole')
    load('data/cartpole_init_guess_T30_dt_01.mat');
elseif strcmp(model.name,'car')
    load('data/car_u_guess_T30.mat')
end
x_mpc = zeros(model.nx, model.horizon+1);
x_mpc(:,1) = model.X0;

u_mpc = zeros(model.nu, model.horizon);
maxIte = 100;
cost_mpc = 0;

for t = 0:model.horizon-1
    
    %fprintf('t= %d \n',t);
    T = model.horizon - t;
    
    % finding a better initial guess using the linear feedback
    if t > 0
        x_tpfc = zeros(model.nx, model.horizon-t+1);
        u_tpfc = zeros(model.nu, model.horizon-t);
        x_tpfc(:,1) = x_mpc(:,t+1);
          
        for j = 1:model.horizon-t
        
            state_err = compute_state_error(x_tpfc(:,j), x_nom(:,j+1), model.name);
            u_tpfc(:,j) = u_nom(:,j+1) - K(:,:,j+1)*state_err ;
                                        
            x_tpfc(:,j+1) = model.state_prop(t, x_tpfc(:,j), u_tpfc(:,j), model); % state propagate.
     
        end

        u_guess = u_tpfc;
    else
        u_guess = u_nom;
    end
    
    [x_nom, u_nom, cost, K] = ILQR(model, x_mpc(:,t+1), model.Xg, u_guess, T,...
                            model.Q, model.R, model.Qf, maxIte); %trajectory optimization using iLQR.
    
    u_mpc(:,t+1) = u_nom(:,1);
    
    state_err = compute_state_error(x_nom(:,1), model.Xg, model.name);

    cost_mpc = cost_mpc + (0.5*state_err'*model.Q*state_err + ... 
                                    0.5*u_mpc(:,t+1)'*model.R*u_mpc(:,t+1)); %incremental cost
                                
    x_mpc(:,t+2) = model.state_prop(t+1, x_mpc(:,t+1), u_mpc(:,t+1), model, epsilon); % state propagate.
 
end

state_err = compute_state_error(x_mpc(:,model.horizon+1), model.Xg, model.name);
cost_mpc = cost_mpc + (0.5*state_err'*model.Qf*state_err); %terminal cost


end

