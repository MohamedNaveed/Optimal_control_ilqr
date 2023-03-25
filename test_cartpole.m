function [] = test_cartpole(model, X0)

T = 100;
X = zeros(model.nx, T+1);
X(:,1) = X0;

for i = 1:T
    U = 0;
    X(:,i+1) = cartpole_nl_state_prop(i, X(:,i), U, model);
end

%% plotting
timesteps = 0:T;
figure(1);
subplot(2,2,1);
plot(timesteps, X(1,:));
ylabel('x');
subplot(2,2,2);
plot(timesteps, X(2,:));
ylabel('xdot');
subplot(2,2,3);
plot(timesteps, X(3,:));
ylabel('theta');
subplot(2,2,4);
plot(timesteps, X(4,:));
ylabel('thetadot');
end

