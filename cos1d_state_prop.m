function [state_n] = cos1d_state_prop(t, state, U, model, epsilon)

if ~exist('epsilon','var')
    w = 0;
else
    w = epsilon*model.u_max*sqrt(model.dt)*normrnd(0,1);
end

state_n = state + (-cos(state) + U)*model.dt + w;

end