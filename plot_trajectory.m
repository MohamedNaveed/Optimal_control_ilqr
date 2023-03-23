function [] = plot_trajectory(x_nom, u_nom, T)

%% plot traj
figure;
timesteps = 0:T;
subplot(3,1,1);
plot(timesteps, x_nom(1,:),'LineWidth',2);
ylabel('$\theta$','Interpreter','latex');
subplot(3,1,2);
plot(timesteps, x_nom(2,:),'LineWidth',2);
ylabel('$\dot{\theta}$','Interpreter','latex');
subplot(3,1,3);
plot(timesteps(1:T), u_nom, 'LineWidth',2);
ylabel('u');
xlabel('Time steps');


end

