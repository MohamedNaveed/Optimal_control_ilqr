function [x_nom, u_nom, cost] = ILQR(model, x0, xg, u_nom, horizon,...
                    Q, R, QT, maxIte)

%% variables
%R = 2*10^0 * eye(Model.nu);
%Q = eye(Model.nx);
%QT = 100*eye(Model.nx);

x_nom = zeros(model.nx,horizon+1); x_nom(:,1) = x0;
Sk = zeros(model.nx,model.nx,horizon+1); Sk(:,:,horizon+1) = QT;
vk = zeros(model.nx,horizon+1);
K = zeros(model.nu,model.nx,horizon);
Kv = zeros(model.nu,model.nx,horizon);
Ku = zeros(model.nu,model.nu,horizon);
Quu = zeros(model.nu,model.nu,horizon);
kt  = zeros(model.nu,horizon);
At = zeros(model.nx,model.nx,horizon);
Bt = zeros(model.nx,model.nu,horizon);
criteria = true;
conv_rate = ones(3,1);

alpha = 1;
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
    
    for i=1:horizon
        
        state_err = (x_new(:,i) - x_nom(:,i));
        state_err(1) = atan2(sin(state_err(1)),cos(state_err(1)));
        
        u_new(:,i) = u_nom(:,i) - K(:,:,i)*state_err + ...
                                alpha*kt(:,i);
        
        state_err = x_new(:,i) - xg;
        state_err(1) = atan2(sin(state_err(1)),cos(state_err(1)));
        
        cost_new = cost_new + 0.5*state_err'*Q*state_err + ... 
                                0.5*u_new(:,i)'*R*u_new(:,i);
                            
        x_new(:,i+1) = model.state_prop(i, x_new(:,i), u_new(:,i), model.dt);
    end
    state_err = (x_new(:,horizon+1) - xg);
    state_err(1) = atan2(sin(state_err(1)),cos(state_err(1)));
    
    cost_new = cost_new + 0.5*state_err'*QT*state_err;
    
    if iter > 1
        z = (cost(iter - 1) - cost_new)/delta_j;
    end
    
    if (z >= -0.6 || alpha < 10^-5)
        forward_flag = false;
        cost(iter) = cost_new;
        x_nom = x_new;
        u_nom = u_new;
        state_err = (x_nom(:,horizon+1)-xg);
        state_err(1) = atan2(sin(state_err(1)),cos(state_err(1)));
        vk(:,horizon+1) = QT*(state_err);

        if alpha<0.05
            alpha=0.05;
        end
    else
        alpha = 0.99*alpha;
    end
    
end

x_traj_ite_ilqr(:,:,iter) = x_nom;
u_traj_ite_ilqr(:,:,iter) = u_nom;

state_err = x_nom(:,end)-xg;
state_err(1) = atan2(sin(state_err(1)),cos(state_err(1)));
state_error = norm(state_err);
[iter state_error cost_new]

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
    
    state_err = (x_nom(:,i)-xg);
    state_err(1) = atan2(sin(state_err(1)),cos(state_err(1)));
    vk(:,i) = (At(:,:,i)-Bt(:,:,i)*K(:,:,i))'*vk(:,i+1)-K(:,:,i)'*R*u_nom(:,i)+Q*state_err;
    
    kt(:,i) = - Kv(:,:,i)*vk(:,i+1) - Ku(:,:,i)*u_nom(:,i); 
    Qu = R*u_nom(:,i) + Bt(:,:,i)'*vk(:,i+1);  
    delta_j = delta_j + (alpha*kt(:,i)'*Qu + alpha^2/2*kt(:,i)'*Quu(:,:,i)*kt(:,i));
end

if cost0 == -1
    cost0 = cost(1);
end

if iter >= 2
    conv_rate(idx) = abs((cost(iter-1)-cost(iter))/cost0);
    idx = idx + 1;
end
if idx > size(conv_rate,1)
    idx = 1;
end
iter = iter + 1; 

rate_conv_diff = abs(conv_rate(1) - conv_rate(2)) + abs(conv_rate(2) - conv_rate(3));


if ((abs(rate_conv_diff) < 0.001) && iter == maxIte)
    criteria = false;
    disp('converged');
    
end

end

end

