function [state_nc, control_error, Qu, kt] = check_NC_conditions(model, x0, xg, x_nom, u_nom, horizon,...
                    Q, R, QT)

x_new = zeros(model.nx,horizon+1); x_new(:,1) = x0;
u_new = zeros(model.nu,horizon); 
Sk = zeros(model.nx,model.nx,horizon+1); Sk(:,:,horizon+1) = QT; %CTG - hessian
vk = zeros(model.nx,horizon+1);%CTG - jacobian
K = zeros(model.nu,model.nx,horizon); %feedback gain for control acting on delta x
Kv = zeros(model.nu,model.nx,horizon);
Ku = zeros(model.nu,model.nu,horizon);
Quu = zeros(model.nu,model.nu,horizon);
Qu = zeros(model.nu,horizon+1);
control_nc = zeros(model.nu,horizon+1);
kt  = zeros(model.nu,horizon);% feedback gain for control constant term
At = zeros(model.nx,model.nx,horizon);
Bt = zeros(model.nx,model.nu,horizon);
lamda = zeros(model.nx,horizon+1); %costate
alpha = model.alpha; %step size
idx = 1;

    
%% forward pass


cost_new = 0;
x_new(:,1) = x0;

for i=1:horizon

    state_err = compute_state_error(x_new(:,i), xg, model.name);
    
    cost_new = cost_new + (0.5*state_err'*Q*state_err + ... 
                                0.5*u_nom(:,i)'*R*u_nom(:,i));
    x_new(:,i+1) = model.state_prop(i, x_new(:,i), u_nom(:,i), model);

end

state_err = compute_state_error(x_new(:,horizon+1), xg, model.name);

cost_new = cost_new + 0.5*state_err'*QT*state_err;


%% backward pass
delta_j=0;
state_err = compute_state_error(x_new(:,horizon+1), xg, model.name);
vk(:,horizon+1) = QT*(state_err);
lamda(:,horizon+1) = QT*(state_err);

for i=horizon:-1:1

    % find perturbation matrices
    [A, B] = model.cal_A_B(model, x_new(:,i), u_nom(:,i));
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

    state_err = compute_state_error(x_new(:,i), xg, model.name);
    vk(:,i) = (At(:,:,i)-Bt(:,:,i)*K(:,:,i))'*vk(:,i+1) - ...
            K(:,:,i)'*R*u_nom(:,i) + Q*state_err;
    lamda(:,i) = Q*state_err + At(:,:,i)'*lamda(:,i+1); %costate evolution
    u_new(:,i) = -inv(R)*Bt(:,:,i)'*lamda(:,i+1);

    kt(:,i) = - Kv(:,:,i)*vk(:,i+1) - Ku(:,:,i)*u_nom(:,i); 
    Qu(:,i) = R*u_nom(:,i) + Bt(:,:,i)'*vk(:,i+1);  
    control_nc(:,i) = R*u_nom(:,i) + Bt(:,:,i)'*lamda(:,i+1);
    delta_j = delta_j + (alpha*kt(:,i)'*Qu + alpha^2/2*kt(:,i)'*Quu(:,:,i)*kt(:,i));
end

%% necessary conditions error

state_nc = x_new - x_nom;
control_error = u_new - u_nom
end