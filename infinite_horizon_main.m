%% Infinite horizon main
clear;clc;
T = 50;

X0 = [0,0];  %theta (rad), thetadot (rad/s)
Xg = [170*pi/180,0];

model = pendulum_model();

X = zeros(2, T+1);
X(:,1) = Xg;

for i = 1:T
   t_span = [(i-1)*model.dt, i*model.dt];
   U = 0;
   [temp, X_out] = ode45(@(t,y) pendulum_nl_ode(t,y,U), t_span, X(:,i));
   X_out(end,1) = atan2(sin(X_out(end,1)), cos(X_out(end,1)));
   X(:,i+1) = X_out(end,:);
end

%% plotting
timesteps = 0:T;
figure(1);
subplot(2,1,1);
plot(timesteps, X(1,:));
subplot(2,1,2);
plot(timesteps, X(2,:));
