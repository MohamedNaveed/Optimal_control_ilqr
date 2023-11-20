function [x_nom, Z_nom, u_nom, delta_u, delta_z, cost, At, Bt] =...
        POD_ILQR(model, x0, xg, u_nom, horizon,...
                    Q, R, QT, maxIte)

x_nom = zeros(model.nx,horizon+1); x_nom(:,1) = x0;
z_nom = zeros(model.nz,horizon+1); z_nom(:,1) = model.C*x0;
Z_nom = zeros(model.nZ, horizon+1);
zg = model.C*xg; % goal state in terms of observation
Zg = zeros(model.nZ,1); %goal Information-state
Zg = [zg; zg; zeros(model.nu,1)]; %TODO 
delta_z = zeros(model.nz,horizon+1);
delta_u = zeros(model.nu, horizon);
Sk = zeros(model.nZ,model.nZ,horizon+1); Sk(:,:,horizon+1) = QT;
vk = zeros(model.nZ,horizon+1);
K = zeros(model.nu,model.nZ,horizon);
Kv = zeros(model.nu,model.nZ,horizon);
Ku = zeros(model.nu,model.nu,horizon);
Quu = zeros(model.nu,model.nu,horizon);
kt  = zeros(model.nu,horizon);
At = zeros(model.nZ,model.nZ,horizon);
Bt = zeros(model.nZ,model.nu,horizon);
criteria = true;
conv_rate = [0,100,200]; %convergence rate variable. initialized with random values.

alpha = model.alpha; %step size
iter = 1;
idx = 1;
change_cost_crit = 1;%parameter used to check if the current solution should be kept or ignored.
cost0 = -1;
    
%% forward pass

while iter <= maxIte && criteria
    forward_flag = true;
    while forward_flag

        cost_new = 0;
        x_new(:,1) = x0;
        z_new(:,1) = model.C*x0;
        %Z_new(:,1) = zeros(model.nZ,1);
        u_new = u_nom;

        for i=1:horizon

            if i < model.q
                x_new(:,i+1) = model.state_prop(i, x_new(:,i), u_new(:,i), model);
                z_new(:,i+1) = model.C*x_new(:,i+1);
                Z_new(:,i) = zeros(model.nZ,1);

            else
                Z_new(:,i) = build_info_state(z_new,u_new,model.q,i,model.nZ);
                state_err = compute_state_error(Z_new(:,i), Z_nom(:,i), model.name);
    
                u_new(:,i) = u_nom(:,i) - K(:,:,i)*state_err + ...
                                        alpha*kt(:,i);
    
                state_err = compute_state_error(Z_new(:,i), Zg, model.name);
    
                cost_new = cost_new + (0.5*state_err'*Q*state_err + ... 
                                        0.5*u_new(:,i)'*R*u_new(:,i));
    
                x_new(:,i+1) = model.state_prop(i, x_new(:,i), u_new(:,i), model);
                z_new(:,i+1) = model.C*x_new(:,i+1);
            end
        end
        Z_new(:,horizon+1) = build_info_state(z_new,u_new,model.q,horizon+1,model.nZ);
        state_err = compute_state_error(Z_new(:,horizon+1), Zg, model.name);

        cost_new = cost_new + 0.5*state_err'*QT*state_err;

        if iter > 1
            change_cost_crit = (cost_new - cost(iter - 1))/delta_J;
            %delta_j helps to reach the solution that satisfies the
            %Necessary conditions
            %change_cost_crit = (cost_new - cost(iter - 1));
        end

        if (change_cost_crit > 0.0 || alpha < 10^-7) %10^-5
        %if (change_cost_crit <= 0 || alpha < 10^-5)
            fprintf('change_cost_crit = %d\n', change_cost_crit);
            forward_flag = false;
            cost(iter) = cost_new;
            x_nom = x_new;
            u_nom = u_new;
            z_nom = z_new;
            Z_nom = Z_new;
            state_err = compute_state_error(Z_nom(:,horizon+1), Zg, model.name);

            vk(:,horizon+1) = QT*(state_err);

            if alpha<0.000005
                alpha=0.000005;
            end
        else
            alpha = 0.99*alpha; %0.99
        end

    end

    x_traj_ite_ilqr(:,:,iter) = x_nom;
    u_traj_ite_ilqr(:,:,iter) = u_nom;
    
    state_err = compute_state_error(Z_nom(:,end), Zg, model.name);

    state_error_norm = norm(state_err);
    fprintf('iter = %d; state_error_norm=%d; cost=%d; lr=%d \n',iter,...
                state_error_norm,cost_new, alpha);
    
    %% sysid - arma
    %delta_u = model.ptb*1*randn(model.nu*(horizon+1),model.nSim);
    %{
    mean = zeros(1,model.nu*(horizon));
    Sigma = model.ptb*diag(abs(u_nom));
    delta_u = mvnrnd(mean, Sigma, model.nSim);
    delta_u = delta_u';
    delta_x0 = model.statePtb*randn(model.nx,model.nSim);%zeros(model.nx,model.nSim);%
    delta_z = zeros(model.nz*(horizon+1),model.nSim);
    delta_z(1:model.nz,:) = model.C*delta_x0;
    
    for sim_iter = 1:model.nSim
        x_temp = x0 + delta_x0(:,sim_iter);
        
        for t_step = 1: horizon
            
            u_temp = u_nom(:,t_step) + delta_u((t_step-1)*model.nu + 1: ...
                                            t_step*model.nu, sim_iter);
            x_temp = model.state_prop(t_step, x_temp, u_temp, model); %x_nom/x_temp
            z_temp = model.C*x_temp';
            delta_z(t_step*model.nz + 1: (t_step+1)*model.nz,sim_iter) =...
                                        z_temp - z_nom(:,t_step+1);

        end

    end
    %}
    %[At, Bt] = arma_fit(model, delta_u, delta_z, x_nom, u_nom);
    [At, Bt] = arma_fit(model, x_nom, u_nom);
    %% backward pass
    delta_J=0;

    for i=horizon:-1:1

        % find perturbation matrices

        % gains
        Quu(:,:,i) = Bt(:,:,i)'*Sk(:,:,i+1)*Bt(:,:,i) + R;
        if min(eig(Quu(:,:,i))) <= 0
            disp('Quu is not positive definite')
        end

        kpreinv = inv(Quu(:,:,i));
        K(:,:,i) = kpreinv*Bt(:,:,i)'*Sk(:,:,i+1)*At(:,:,i);
        Kv(:,:,i) = kpreinv*Bt(:,:,i)';
        Ku(:,:,i) = kpreinv*R;
        Sk(:,:,i) = At(:,:,i)'*Sk(:,:,i+1)*(At(:,:,i)-Bt(:,:,i)*K(:,:,i)) + Q;

        state_err = compute_state_error(Z_nom(:,i), Zg, model.name);
        vk(:,i) = (At(:,:,i)-Bt(:,:,i)*K(:,:,i))'*vk(:,i+1)-K(:,:,i)'*R*u_nom(:,i)+Q*state_err;

        kt(:,i) = - Kv(:,:,i)*vk(:,i+1) - Ku(:,:,i)*u_nom(:,i); 
        Qu = R*u_nom(:,i) + Bt(:,:,i)'*vk(:,i+1);  
        delta_J = delta_J + (alpha*kt(:,i)'*Qu + alpha^2/2*kt(:,i)'*Quu(:,:,i)*kt(:,i));
    end

    if cost0 == -1
        cost0 = cost(1);
    end

    if iter >= 2
        conv_rate(idx) = abs((cost(iter-1)-cost(iter)));%/cost0);
        idx = idx + 1;
    end
    if idx > length(conv_rate)
        idx = 1;
    end
    iter = iter + 1; 

    %rate_conv_diff = abs(conv_rate(1) - conv_rate(2)) + abs(conv_rate(2) - conv_rate(3));
    cost_change = sum(conv_rate);

    if ((abs(cost_change) < 0.00001) || iter == maxIte)
        criteria = false;
        %disp('converged');

    end

end

end