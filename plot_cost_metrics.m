function [] = plot_cost_metrics(T_vec, cost_ilqr,...
                            total_cost, exp_CTG_vec, true_CTG_vec)

                        
figure;
plot(T_vec, total_cost,'b','Marker','o','LineWidth',3, 'DisplayName', 'Total cost');
hold on;
plot(T_vec, cost_ilqr,'r--','LineWidth',2, 'DisplayName', 'iLQR cost');
xlabel('Finite horizon T');
ylabel('Cost');
legend();
                        
figure;
plot(T_vec, exp_CTG_vec,'b','Marker','o','LineWidth',3, 'DisplayName', 'Expected CTG');
hold on;
plot(T_vec, true_CTG_vec,'r--','LineWidth',2, 'DisplayName', 'Actual CTG');
xlabel('Finite horizon T');
ylabel('Cost');
legend();
end

