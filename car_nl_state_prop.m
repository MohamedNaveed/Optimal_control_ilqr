function [state_n] = car_nl_state_prop(t, state, U, model, epsilon)


if ~exist('epsilon','var')
    w = zeros(model.nx,1);
else
    mu = zeros(model.nx,1);
    w = epsilon*sqrt(model.dt)*mvnrnd(mu, model.Cov);
    w = reshape(w,[model.nx,1]);
end

X_out = RK4(t, state, U, model);
state_n = X_out + w;



end