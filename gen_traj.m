function [x] = gen_traj(x0, u, model)

    x = zeros(size(x0,1), size(u,2)+1);
    x(:,1) = x0;
    for i=1:model.horizon
        x(:,i+1) = model.state_prop(i, x(:,i), u(:,i), model);
    end
end