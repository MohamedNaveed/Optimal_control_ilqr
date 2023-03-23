function [] = test_pendulum(X0)

T = 50;
X = zeros(2, T+1);
X(:,1) = X0;

for i = 1:T
    U = 0;
    X(:,i+1) = pendulum_nl_state_prop(i, X(:,i), U);
end

%% plotting
timesteps = 0:T;
figure(1);
subplot(2,1,1);
plot(timesteps, X(1,:));
subplot(2,1,2);
plot(timesteps, X(2,:));
end

