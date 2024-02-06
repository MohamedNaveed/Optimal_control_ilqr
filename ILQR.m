function [x_nom, u_nom, cost] = ILQR(model, x0, xg, u_nom, horizon,...
                    Q, R, QT, maxIte)

%% variables

x_nom = zeros(model.nx,horizon+1); x_nom(:,1) = x0;
Sk = zeros(model.nx,model.nx,horizon+1); Sk(:,:,horizon+1) = QT; %CTG - hessian
vk = zeros(model.nx,horizon+1);%CTG - jacobian
K = zeros(model.nu,model.nx,horizon); %feedback gain for control acting on delta x
Kv = zeros(model.nu,model.nx,horizon);
Ku = zeros(model.nu,model.nu,horizon);
Quu = zeros(model.nu,model.nu,horizon);
kt  = zeros(model.nu,horizon);% feedback gain for control constant term
At = zeros(model.nx,model.nx,horizon);
Bt = zeros(model.nx,model.nu,horizon);
criteria = true;
conv_rate = [0,100,200]; %convergence rate variable. initialized with random values.

alpha = model.alpha; %step size
iter = 1;
idx = 1;
change_cost_crit = 1; %parameter used to check if the current solution should be kept or ignored.
cost0 = -1;
    
%% forward pass

while iter <= maxIte && criteria
    forward_flag = true;
    while forward_flag

        cost_new = 0;
        x_new(:,1) = x0;
        u_new = u_nom;

        for i=1:horizon

            if i < model.q
                x_new(:,i+1) = model.state_prop(i, x_new(:,i), u_new(:,i), model);

            else

                state_err = compute_state_error(x_new(:,i), x_nom(:,i), model.name);
    
                u_new(:,i) = u_nom(:,i) - K(:,:,i)*state_err + ...
                                        alpha*kt(:,i);
                %fprintf("u_nom = %d; u_new = %d \n", u_nom(:,i), u_new(:,i));
                state_err = compute_state_error(x_new(:,i), xg, model.name);
    
                cost_new = cost_new + (0.5*state_err'*Q*state_err + ... 
                                        0.5*u_new(:,i)'*R*u_new(:,i));
    
                x_new(:,i+1) = model.state_prop(i, x_new(:,i), u_new(:,i), model);
            end
        end

        state_err = compute_state_error(x_new(:,horizon+1), xg, model.name);

        cost_new = cost_new + 0.5*state_err'*QT*state_err;

        if iter > 1
            change_cost_crit = (cost_new - cost(iter - 1))/delta_J;
            %delta_j helps to reach the solution that satisfies the
            %Necessary conditions

            %change_cost_crit = (cost_new - cost(iter - 1));
        end

        if (change_cost_crit > 0.0 || alpha < 10^-5)
        %if (change_cost_crit > 0.0) %refer IROS2012 Todorov 
        %if (change_cost_crit <= 0 || alpha < 10^-5)
            %fprintf('change_cost_crit = %d\n', change_cost_crit);   
            forward_flag = false;
            cost(iter) = cost_new;
            x_nom = x_new;
            u_nom = u_new;
            state_err = compute_state_error(x_nom(:,horizon+1), xg, model.name);

            vk(:,horizon+1) = QT*(state_err);

            %if alpha < e-14
            %    alpha=0.00000005;
            %end
        else
            alpha = 0.99*alpha;
            %fprintf('alpha = %d \n', alpha);
        end

    end

    x_traj_ite_ilqr(:,:,iter) = x_nom;
    u_traj_ite_ilqr(:,:,iter) = u_nom;

    state_err = compute_state_error(x_nom(:,end), xg, model.name);

    state_error_norm = norm(state_err);
    %fprintf('iter = %d; state_error_norm=%d; cost=%d; lr=%d \n',iter,...
    %            state_error_norm,cost_new, alpha);
    %[iter state_error_norm cost_new]

    %% backward pass
    delta_J=0; %expected total cost reduction

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
        K(:,:,i) = kpreinv*Bt(:,:,i)'*Sk(:,:,i+1)*At(:,:,i);%fb gain
        Kv(:,:,i) = kpreinv*Bt(:,:,i)';
        Ku(:,:,i) = kpreinv*R;
        Sk(:,:,i) = At(:,:,i)'*Sk(:,:,i+1)*(At(:,:,i)-Bt(:,:,i)*K(:,:,i)) + Q;

        state_err = compute_state_error(x_nom(:,i), xg, model.name);
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

    if ((abs(cost_change) < 0.0001) || iter == maxIte)
    %if (iter == maxIte)
        criteria = false;
        %disp('converged');

    end

end

end

