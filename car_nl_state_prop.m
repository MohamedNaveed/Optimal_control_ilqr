function [state_n] = car_nl_state_prop(t, state, U, model, epsilon)


if ~exist('epsilon','var')
    w = zeros(model.nx,1);
else
    w = epsilon*model.u_max*sqrt(model.dt)*normrnd(0,1);
end

X_out = forward_euler(t, state, U, model);
state_n = X_out + w;



end