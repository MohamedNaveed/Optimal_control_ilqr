function [outputArg1,outputArg2] = POD_ILQR(inputArg1,inputArg2)

x_nom = zeros(model.nx,horizon+1); x_nom(:,1) = x0;
Z_nom = zeros(model.nZ, horizon+1);
delta_z = zeros(model.nz,horizon+1);
delta_u = zeros(Model.nu, horizon);
Sk = zeros(model.nZ,model.nx,horizon+1); Sk(:,:,horizon+1) = QT;
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
z = 1;
cost0 = -1;
    
%% forward pass

while iter <= maxIte && criteria
    forward_flag = true;
    while forward_flag

        cost_new = 0;
        x_new(:,1) = x0;
        u_new = u_nom;

        for i=q:horizon

            state_err = compute_state_error(x_new(:,i), x_nom(:,i), model.name);

            u_new(:,i) = u_nom(:,i) - K(:,:,i)*state_err + ...
                                    alpha*kt(:,i);

            state_err = compute_state_error(x_new(:,i), xg, model.name);

            cost_new = cost_new + (0.5*state_err'*Q*state_err + ... 
                                    0.5*u_new(:,i)'*R*u_new(:,i));

            x_new(:,i+1) = model.state_prop(i, x_new(:,i), u_new(:,i), model);
        end

        state_err = compute_state_error(x_new(:,horizon+1), xg, model.name);

        cost_new = cost_new + 0.5*state_err'*QT*state_err;

        if iter > 1
            z = (cost(iter - 1) - cost_new)/delta_j;
        end

        if (z >= -0.6 || alpha < 10^-5)
            forward_flag = false;
            cost(iter) = cost_new;
            x_nom = x_new;
            u_nom = u_new;
            state_err = compute_state_error(x_nom(:,horizon+1), xg, model.name);

            vk(:,horizon+1) = QT*(state_err);

            if alpha<0.005
                alpha=0.005;
            end
        else
            alpha = 0.99*alpha;
        end

    end

    x_traj_ite_ilqr(:,:,iter) = x_nom;
    u_traj_ite_ilqr(:,:,iter) = u_nom;

    state_err = compute_state_error(x_nom(:,end), xg, model.name);

    state_error_norm = norm(state_err);
    [iter state_error_norm cost_new]

    %% backward pass
    delta_j=0;

    for i=horizon:-1:1

        % find perturbation matrices
        [A, B] = model.cal_A_B(model, x_nom(:,i), u_nom(:,i));
        At(:,:,i) = A;
        Bt(:,:,i) = B;

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

        state_err = compute_state_error(x_nom(:,i), xg, model.name);
        vk(:,i) = (At(:,:,i)-Bt(:,:,i)*K(:,:,i))'*vk(:,i+1)-K(:,:,i)'*R*u_nom(:,i)+Q*state_err;

        kt(:,i) = - Kv(:,:,i)*vk(:,i+1) - Ku(:,:,i)*u_nom(:,i); 
        Qu = R*u_nom(:,i) + Bt(:,:,i)'*vk(:,i+1);  
        delta_j = delta_j + (alpha*kt(:,i)'*Qu + alpha^2/2*kt(:,i)'*Quu(:,:,i)*kt(:,i));
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

    if ((abs(cost_change) < 0.001) || iter == maxIte)
        criteria = false;
        %disp('converged');

    end

end

end